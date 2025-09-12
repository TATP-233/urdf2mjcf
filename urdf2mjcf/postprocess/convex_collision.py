import argparse
import logging
import multiprocessing as mp
import os
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Optional, Tuple, List

import trimesh
from urdf2mjcf.utils import save_xml

logger = logging.getLogger(__name__)


def process_single_mesh(mesh_info: Tuple[str, str, Path]) -> Optional[Tuple[str, List[Tuple[str, str]]]]:
    """处理单个 mesh：生成并保存其凸包，并返回新的 mesh 资产信息。
    
    Args:
        mesh_info: (mesh_name, mesh_file_path, mesh_dir)的元组
        
    Returns:
        如果成功处理，返回 (mesh_name, [(part_name, part_file)])，其中仅包含一个凸包 part
        如果处理失败或无需处理，返回 None
    """
    mesh_name, mesh_file_relative, mesh_dir = mesh_info
    
    # 正确解析mesh文件路径
    mesh_file = None
    # Handle regular paths (relative to URDF file or absolute)
    if Path(mesh_file_relative).is_absolute():
        mesh_file = Path(mesh_file_relative)
    else:
        mesh_file = mesh_dir / mesh_file_relative
    
    if mesh_file is None or not mesh_file.exists():
        logger.warning(f"Mesh file {mesh_file_relative} does not exist at {mesh_file}.")
        return None
    
    logger.info(f"Processing mesh {mesh_name}: generating convex hull")
    
    # 加载 mesh
    try:
        mesh_data = trimesh.load(mesh_file, force="mesh")
    except Exception as e:
        logger.error(f"Failed to load mesh {mesh_file}: {e}")
        return None
    
    # 生成凸包并导出
    try:
        convex_hull = mesh_data.convex_hull
    except Exception as e:
        logger.error(f"Failed to compute convex hull for mesh {mesh_name}: {e}")
        return None

    # 创建存放凸包的文件夹（与原 parts 结构一致的层级，便于管理）
    mesh_stem = mesh_file.stem  # 文件名不带扩展名
    hull_dir = mesh_file.parent / f"{mesh_stem}_convex"
    hull_dir.mkdir(exist_ok=True)

    # 保存凸包为 STL
    try:
        part_filename = f"{mesh_stem}_convex.stl"
        part_file = hull_dir / part_filename
        convex_hull.export(part_file)

        # 相对于 meshdir 的路径，保持原始目录结构
        original_dir = str(Path(mesh_file_relative).parent)  # 例如: meshes/leg
        relative_part_path = f"{original_dir}/{mesh_stem}_convex/{part_filename}"
        part_name = f"{mesh_stem}_convex"
        logger.info(f"Saved convex hull to {relative_part_path}")
        return (mesh_name, [(part_name, relative_part_path)])
    except Exception as e:
        logger.error(f"Failed to save convex hull of mesh {mesh_name}: {e}")
        return None

def convex_collision_assets(mjcf_path: str | Path, root: ET.Element, max_processes: Optional[int] = None) -> None:
    """对 MJCF 文件中 collision 类型的 mesh geom 用其凸包替换。

    Args:
        mjcf_path: MJCF 文件的路径
        root: MJCF 文件的根元素
        max_processes: 最大进程数，如果为None则自动计算为CPU核心数-4
    """
    mjcf_path = Path(mjcf_path)
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    compiler.attrib["meshdir"] = "."

    mesh_dir = mjcf_path.parent / compiler.attrib["meshdir"]

    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")

    # 收集现有的 mesh 资产
    mesh_assets: dict[str, str] = {}
    for mesh in asset.findall("mesh"):
        mesh_assets[mesh.attrib["name"]] = mesh.attrib["file"]
    
    # 收集需要进行凸分解的 collision geom
    collision_geoms = []
    for body in root.findall(".//body"):
        for geom in body.findall("geom"):
            if (geom.attrib.get("class") == "collision" and 
                geom.attrib.get("type") == "mesh" and
                "mesh" in geom.attrib):
                collision_geoms.append((body, geom))
    
    # 收集所有需要处理的唯一mesh
    unique_meshes = set()
    for body, geom in collision_geoms:
        mesh_name = geom.attrib["mesh"]
        if mesh_name in mesh_assets:
            unique_meshes.add(mesh_name)
        else:
            logger.warning(f"Mesh {mesh_name} not found in assets.")
    
    if not unique_meshes:
        logger.info("No meshes to process for convex decomposition")
        return
    
    # 准备多进程处理的数据
    mesh_info_list = []
    for mesh_name in unique_meshes:
        mesh_info_list.append((mesh_name, mesh_assets[mesh_name], mesh_dir))
    
    # 获取CPU核心数并设置进程数
    cpu_count = os.cpu_count() or 4  # 如果获取失败，默认使用4核
    if max_processes is None:
        max_processes = max(1, cpu_count - 4)  # 至少使用1个进程
    else:
        max_processes = max(1, max_processes)  # 确保至少使用1个进程
    actual_processes = min(max_processes, len(mesh_info_list))  # 不超过任务数量
    
    logger.info(f"Detected {cpu_count} CPU cores, using {actual_processes} processes for convex hull generation")
    
    # 使用多进程处理mesh
    new_mesh_parts = {}
    if actual_processes == 1:
        # 单进程处理
        for mesh_info in mesh_info_list:
            result = process_single_mesh(mesh_info)
            if result:
                mesh_name, part_info = result
                new_mesh_parts[mesh_name] = part_info
    else:
        # 多进程处理
        with mp.Pool(processes=actual_processes) as pool:
            results = pool.map(process_single_mesh, mesh_info_list)
            
            # 收集结果
            for result in results:
                if result:
                    mesh_name, part_info = result
                    new_mesh_parts[mesh_name] = part_info
    
    logger.info(f"Successfully processed {len(new_mesh_parts)} meshes with convex hull replacement")
    
    # 更新 asset 部分
    for mesh_name, part_info in new_mesh_parts.items():
        # 删除原始 mesh
        # for mesh in asset.findall("mesh"):
        #     if mesh.attrib["name"] == mesh_name:
        #         asset.remove(mesh)
        #         break
        
        # 添加新的 part meshes
        for part_name, part_file in part_info:
            new_mesh = ET.SubElement(asset, "mesh")
            new_mesh.attrib["name"] = part_name
            new_mesh.attrib["file"] = part_file
    
    # 更新 geom 部分
    for body, geom in collision_geoms:
        mesh_name = geom.attrib.get("mesh")
        if mesh_name in new_mesh_parts:
            # 删除原始 geom
            body.remove(geom)
            
            # 为每个 part 添加新的 geom
            for part_name, _ in new_mesh_parts[mesh_name]:
                new_geom = ET.SubElement(body, "geom")
                # 复制原始 geom 的属性
                for attr_name, attr_value in geom.attrib.items():
                    if attr_name != "mesh" and attr_name != "name":
                        new_geom.attrib[attr_name] = attr_value
                
                # 设置新的名称和 mesh
                original_name = geom.attrib.get("name", f"{mesh_name}_collision")
                new_geom.attrib["name"] = f"{original_name}_{part_name}"
                new_geom.attrib["mesh"] = part_name


def convex_collision(mjcf_path: str | Path, max_processes: Optional[int] = None) -> None:
    """对 MJCF 文件进行“碰撞体替换为凸包”的处理。

    Args:
        mjcf_path: MJCF 文件的路径
        max_processes: 最大进程数，如果为None则自动计算为CPU核心数-4
    """
    tree = ET.parse(mjcf_path)
    root = tree.getroot()
    convex_collision_assets(mjcf_path, root, max_processes)
    save_xml(mjcf_path, tree)

def main() -> None:
    parser = argparse.ArgumentParser(description="对 MJCF 文件中的 collision mesh 用其凸包替换")
    parser.add_argument("mjcf_path", type=Path, help="MJCF 文件的路径")
    parser.add_argument("--processes", type=int, help="指定使用的进程数量，默认为CPU核心数-4")
    args = parser.parse_args()
    
    logging.basicConfig(level=logging.INFO)
    convex_collision(args.mjcf_path, args.processes)

if __name__ == "__main__":
    main()