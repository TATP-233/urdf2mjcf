import os
import sys
import argparse
from pathlib import Path
from typing import Iterable, List, Tuple

import math
import re
from tqdm import tqdm
import multiprocessing as mp
from concurrent.futures import ProcessPoolExecutor, as_completed


def _ensure_occ_available() -> None:
    try:
        # Quick import check without polluting namespace
        import OCC  # type: ignore  # noqa: F401
    except Exception as exc:  # pragma: no cover - runtime guard
        msg = (
            "未检测到 pythonocc-core：请先安装 OpenCASCADE 的 Python 绑定。\n"
            "推荐安装命令：\n"
            "  pip install pythonocc-core\n\n"
            "若使用 conda：\n"
            "  conda install -c conda-forge pythonocc-core\n\n"
            f"原始错误：{exc}"
        )
        print(msg, file=sys.stderr)
        sys.exit(1)


def _read_step_one_shape(step_path: Path):
    """Load STEP file and return a single compound shape.

    Notes:
        We use STEPControl_Reader to obtain a combined TopoDS_Shape.
        Assembly hierarchy and names are not preserved here, but we will
        split by solids which usually correspond to parts.
    """
    from OCC.Core.STEPControl import STEPControl_Reader  # type: ignore
    from OCC.Core.IFSelect import IFSelect_RetDone  # type: ignore

    reader = STEPControl_Reader()
    status = reader.ReadFile(str(step_path))
    if status != IFSelect_RetDone:
        raise RuntimeError(f"读取STEP失败: {step_path}")
    # Transfer all roots
    ok = reader.TransferRoots()
    if not ok:
        raise RuntimeError("STEP转换失败：未能传输根节点")
    shape = reader.OneShape()
    return shape


def _iterate_solids(shape) -> Iterable:
    from OCC.Core.TopAbs import TopAbs_SOLID  # type: ignore
    from OCC.Core.TopExp import TopExp_Explorer  # type: ignore

    exp = TopExp_Explorer(shape, TopAbs_SOLID)
    while exp.More():
        yield exp.Current()
        exp.Next()


def _mesh_shape(shape, linear_deflection: float, angular_deflection_rad: float, relative: bool = True) -> None:
    from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh  # type: ignore

    # Perform mesh generation; parallel=True
    mesher = BRepMesh_IncrementalMesh(
        shape, float(linear_deflection), bool(relative), float(angular_deflection_rad), True
    )
    # In some versions Perform is implicit, but call to be explicit.
    try:
        mesher.Perform()
    except Exception:
        pass


def _export_stl(shape, out_path: Path) -> None:
    from OCC.Core.StlAPI import StlAPI_Writer  # type: ignore

    writer = StlAPI_Writer()
    # Binary STL is default; use ASCII if needed via writer.SetASCIIMode(True)
    if not writer.Write(shape, str(out_path)):
        raise RuntimeError(f"写入STL失败: {out_path}")


def _export_step(shape, out_path: Path) -> None:
    """Export shape to STEP format."""
    from OCC.Core.STEPControl import STEPControl_Writer  # type: ignore
    from OCC.Core.IFSelect import IFSelect_RetDone  # type: ignore

    writer = STEPControl_Writer()
    writer.Transfer(shape, 0)  # 0 for the first shape
    status = writer.Write(str(out_path))
    if status != IFSelect_RetDone:
        raise RuntimeError(f"写入STEP失败: {out_path}")

def _export_obj(shape, out_path: Path) -> None:
    """Export shape directly to OBJ format."""
    stl_path = out_path.with_suffix('.stl')
    _export_stl(shape, stl_path)
    _stl_to_obj(stl_path, out_path)
    stl_path.unlink()

def _stl_to_obj(stl_path: Path, obj_path: Path) -> None:
    # Convert STL to OBJ via trimesh to keep dependencies light
    try:
        import trimesh
    except Exception as exc:  # pragma: no cover
        raise RuntimeError(
            f"需要 trimesh 将 STL 转为 OBJ，请安装：pip install trimesh\n原始错误：{exc}"
        ) from exc

    mesh = trimesh.load(str(stl_path), force='mesh')
    if mesh.is_empty:
        raise RuntimeError(f"从STL加载网格为空：{stl_path}")
    mesh.export(str(obj_path))


def _process_single_solid(args_tuple: Tuple[int, object, Path, str, float, float, bool, float]) -> Tuple[str, Path]:
    """处理单个实体的函数，用于多进程

    参数最后一位 scale_factor：导出时应用的尺度（原文件单位->米）。
    """
    idx, solid, out_dir, export_format, linear_deflection, angular_deflection_rad, relative, scale_factor = args_tuple
    
    part_name = f"part_{idx:03d}"
    
    primary_path: Path
    
    # 若需要缩放，先缩放该 solid (局部原点缩放)
    if abs(scale_factor - 1.0) > 1e-15:
        try:
            solid = _scale_shape(solid, scale_factor)
        except Exception as exc:  # pragma: no cover
            raise RuntimeError(f"缩放 solid 失败: {exc}")

    if export_format.lower() == "stl":
        # 直接导出STL格式
        _mesh_shape(solid, linear_deflection, angular_deflection_rad, relative)
        primary_path = out_dir / f"{part_name}.stl"
        _export_stl(solid, primary_path)
    elif export_format.lower() == "obj":
        # 直接导出OBJ格式
        _mesh_shape(solid, linear_deflection, angular_deflection_rad, relative)
        primary_path = out_dir / f"{part_name}.obj"
        _export_obj(solid, primary_path)
    elif export_format.lower() == "stp":
        # 直接导出STP格式
        primary_path = out_dir / f"{part_name}.stp"
        _export_step(solid, primary_path)
    else:
        raise ValueError("export_format 应为 'stl' | 'obj' | 'stp'")

    return (part_name, primary_path)

def _detect_step_unit(step_file: Path) -> Tuple[bool, float, str]:
    """检测 STEP 文件中的长度单位, 返回 (found, scale_to_meter, description)

    仅基于简单的文本扫描 SI_UNIT(.PREFIX.,.METRE.) / SI_UNIT(.METRE.)。
    若找到 MILLI/CENTI/DECI/KILO/MICRO/NANO 等前缀则换算, 否则若出现 .METRE. 视为米。
    可能存在更复杂自定义单位 (CONVERSION_BASED_UNIT), 此处暂不解析。
    """
    text = step_file.read_text(errors="ignore")[:200000]  # 限制前 200k 字符
    # 统一大写便于匹配
    up = text.upper()
    # 正则抓取 SI_UNIT(...) 内容
    matches = re.findall(r"SI_UNIT\(([^)]*)\)", up)
    scale_map = {
        "MILLI": 1e-3,
        "CENTI": 1e-2,
        "DECI": 1e-1,
        "KILO": 1e3,
        "MICRO": 1e-6,
        "NANO": 1e-9,
    }
    for m in matches:
        # 例: ".MILLI.,.METRE." 或 ".METRE."
        parts = [p.strip(" .") for p in m.split(',') if p.strip()]
        # 只关注包含 METRE 的定义
        if any(p == "METRE" for p in parts):
            prefix = None
            for p in parts:
                if p in scale_map:
                    prefix = p
                    break
            if prefix:
                return True, scale_map[prefix], f"{prefix.lower()}metre (scale {scale_map[prefix]})"
            else:
                return True, 1.0, "metre"
    return False, 1.0, "unknown"


def _scale_shape(shape, factor: float):
    if abs(factor - 1.0) < 1e-15:
        return shape
    from OCC.Core.gp import gp_Trsf, gp_Pnt  # type: ignore
    from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform  # type: ignore

    trsf = gp_Trsf()
    trsf.SetScale(gp_Pnt(0.0, 0.0, 0.0), float(factor))
    t = BRepBuilderAPI_Transform(shape, trsf, True)
    return t.Shape()


def _compute_shape_aabb(shape) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    from OCC.Core.Bnd import Bnd_Box  # type: ignore
    from OCC.Core.BRepBndLib import brepbndlib_Add  # type: ignore

    box = Bnd_Box()
    # 使用三角化, 包含自体公差
    brepbndlib_Add(shape, box)
    xmin, ymin, zmin, xmax, ymax, zmax = box.Get()
    return (xmin, ymin, zmin), (xmax, ymax, zmax)


def split_and_export(
    step_file: Path,
    out_dir: Path,
    export_format: str = "stl",
    linear_deflection: float = 0.2,
    angular_deflection_deg: float = 20.0,
    relative: bool = True,
    max_workers: int = None,
    scale_factor: float = 1.0,
) -> List[Tuple[str, Path]]:
    """Split STEP into solids and export each as files.

    Args:
        step_file: 输入的STEP文件路径
        out_dir: 输出目录
        export_format: 导出格式 ("stl", "obj", "stp")
        linear_deflection: 线性细分容差
        angular_deflection_deg: 角度细分容差(度)
        relative: 是否使用相对容差
        max_workers: 最大进程数，None表示使用CPU核心数的一半

    Returns:
        List of tuples (part_name, exported_path) for the primary format.
    """
    _ensure_occ_available()

    step_file = step_file.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    shape = _read_step_one_shape(step_file)

    # 计算整体 AABB: 使用一个缩放后的临时副本 (不修改原 shape, 后续每个 part 单独缩放)
    if abs(scale_factor - 1.0) > 1e-15:
        scaled_for_aabb = _scale_shape(shape, scale_factor)
    else:
        scaled_for_aabb = shape
    global_min, global_max = _compute_shape_aabb(scaled_for_aabb)
    print(
        f"整体模型 AABB (m): min=({global_min[0]:.6g}, {global_min[1]:.6g}, {global_min[2]:.6g}) "
        f"max=({global_max[0]:.6g}, {global_max[1]:.6g}, {global_max[2]:.6g})"
    )

    # Mesh parameters
    angular_deflection_rad = math.radians(angular_deflection_deg)

    solids = list(_iterate_solids(shape))
    if not solids:
        # fallback: export the whole shape as a single part
        solids = [shape]

    results: List[Tuple[str, Path]] = []
    
    # 设置默认进程数
    if max_workers is None:
        max_workers = max(1, mp.cpu_count() // 2)
    
    if len(solids) > 1:
        # 使用多进程处理
        max_workers = min(max_workers, len(solids))
        
        # 准备参数
        args_list = [
            (idx, solid, out_dir, export_format, linear_deflection, angular_deflection_rad, relative, scale_factor)
            for idx, solid in enumerate(solids, start=1)
        ]
        
        # 使用进度条显示多进程处理进度
        with ProcessPoolExecutor(max_workers=max_workers) as executor:
            futures = [executor.submit(_process_single_solid, args) for args in args_list]
            
            for future in tqdm(as_completed(futures), total=len(futures), desc="处理零件"):
                try:
                    result = future.result()
                    results.append(result)
                except Exception as e:
                    print(f"处理零件时出错: {e}", file=sys.stderr)
    else:
        # 单进程处理，使用进度条
        for idx, solid in enumerate(tqdm(solids, desc="处理零件"), start=1):
            part_name = f"part_{idx:03d}"
            
            primary_path: Path
            
            # 单进程同样应用缩放
            if abs(scale_factor - 1.0) > 1e-15:
                solid_scaled = _scale_shape(solid, scale_factor)
            else:
                solid_scaled = solid

            if export_format.lower() == "stl":
                # 直接导出STL格式
                _mesh_shape(solid_scaled, linear_deflection, angular_deflection_rad, relative)
                primary_path = out_dir / f"{part_name}.stl"
                _export_stl(solid_scaled, primary_path)
            elif export_format.lower() == "obj":
                # 直接导出OBJ格式
                _mesh_shape(solid_scaled, linear_deflection, angular_deflection_rad, relative)
                primary_path = out_dir / f"{part_name}.obj"
                _export_obj(solid_scaled, primary_path)
            elif export_format.lower() == "stp":
                # 直接导出STP格式（不需要网格化）
                primary_path = out_dir / f"{part_name}.stp"
                _export_step(solid_scaled, primary_path)
            else:
                raise ValueError("export_format 应为 'stl' | 'obj' | 'stp'")

            results.append((part_name, primary_path))

    return results

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="读取STEP(.stp/.step)文件，将所有零件拆分并导出为STL/OBJ，并输出整体 AABB。"
    )
    parser.add_argument("input", type=str, help="输入STEP文件路径(.stp/.step)")
    parser.add_argument(
        "output", type=str, nargs='?', default="./output_parts", 
        help="输出目录或模型路径（默认: ./output_parts)"
    )
    parser.add_argument(
        "-f",
        "--format",
        type=str,
        choices=["stl", "obj", "stp"],
        default="stl",
        help="导出格式（stl/obj/stp，默认: stl）",
    )
    parser.add_argument(
        "--linear-deflection",
        type=float,
        default=0.2,
        help="线性细分容差（越小越精细，默认 0.2）",
    )
    parser.add_argument(
        "--angular-deflection-deg",
        type=float,
        default=20.0,
        help="角度细分容差(度，越小越精细，默认 20°)",
    )
    parser.add_argument(
        "--absolute",
        action="store_true",
        help="使用绝对容差（默认使用相对模式）",
    )
    parser.add_argument(
        "--max-workers",
        type=int,
        default=None,
        help="最大进程数（默认使用CPU核心数的一半）",
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=None,
        help="当文件未检测到单位信息时用于整体缩放到米 (例如毫米文件可传 0.001)。若文件包含单位则忽略。",
    )
    return parser.parse_args()

def main() -> int:
    ns = parse_args()
    step_path = Path(ns.input)
    if not step_path.exists():
        print(f"输入文件不存在: {step_path}", file=sys.stderr)
        return 2
    
    out_dir = Path(ns.output)

    # 单位检测
    unit_found, unit_scale, unit_desc = _detect_step_unit(step_path)
    if unit_found:
        if ns.scale is not None:
            print("检测到文件包含单位信息，忽略 --scale 参数。", file=sys.stderr)
        applied_scale = unit_scale
        print(f"检测到单位: {unit_desc} -> 统一转换为米 (scale={unit_scale})")
    else:
        if ns.scale is not None:
            applied_scale = float(ns.scale)
            print(f"未检测到单位信息，使用传入 scale={applied_scale}")
        else:
            applied_scale = 1.0
            print("未检测到单位信息，未提供 --scale，假定已为米。")

    try:
        results = split_and_export(
            step_file=step_path,
            out_dir=out_dir,
            export_format=ns.format,
            linear_deflection=ns.linear_deflection,
            angular_deflection_deg=ns.angular_deflection_deg,
            relative=not ns.absolute,
            max_workers=ns.max_workers,
            scale_factor=applied_scale,
        )
    except Exception as exc:
        print(f"处理失败：{exc}", file=sys.stderr)
        return 1

    print(f"已导出 {len(results)} 个零件至: {out_dir}")
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())


