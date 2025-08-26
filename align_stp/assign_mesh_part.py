"""根据 mapping.json 判断一个独立 mesh 属于 OBJ 模型的哪个 part 并做坐标转换保存。

使用流程:
    python -m align_stp.assign_mesh_part \
        --mesh path/to/mesh.obj \
        --obj path/to/multi_part.obj \
        --mapping path/to/mapping.json \
        --out-dir output_dir \
        --save-local  # 额外保存到 part 局部坐标系

主要步骤:
1. 读取输入 mesh, 计算其 AABB。
2. 读取 mapping.json 中每个 part 的 AABB, 先做 AABB “包含” 粗筛 (要求 mesh AABB 被 part AABB 完全包裹, 允许 eps 松弛)。
3. 对候选 part, 解析 OBJ (trimesh.Scene) 中对应子网格, 做精细非凸检测:
   - 采样输入 mesh 顶点 (<= --sample 点)
   - 使用 part_mesh.contains(points) (光线投射) 统计包含比例
   - 得分 = inside_ratio, 若都为 0 再用质心距离作为备选
4. 选出最佳 part.
5. 将输入 mesh:
   - 保持在全局坐标 (原样)
   - 若指定 --save-local, 使用 mapping 中 transform_4x4 的逆矩阵变换到该 part 局部坐标系
6. 保存结果并输出 JSON 描述。

注意: mapping.json 中 transform_4x4 视为 part->world 变换 (列/行主假设为常见行主 4x4)。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Sequence, Tuple

import numpy as np

try:
    import trimesh  # type: ignore
except Exception as exc:  # pragma: no cover
    print("需要 trimesh 依赖: pip install trimesh", file=sys.stderr)
    raise


@dataclass
class PartInfo:
    name: str
    aabb_min: np.ndarray  # (3,)
    aabb_max: np.ndarray  # (3,)
    transform: np.ndarray  # (4,4) part->world

    def contains_aabb(self, other_min: np.ndarray, other_max: np.ndarray, eps: float = 0.0) -> bool:
        """判断 other AABB 是否被当前 part AABB 完整包含 (带 eps 宽松)。"""
        return bool(
            np.all(self.aabb_min - eps <= other_min) and np.all(other_max <= self.aabb_max + eps)
        )

def load_mapping(path: Path) -> List[PartInfo]:
    data = json.loads(path.read_text())
    parts: List[PartInfo] = []
    for p in data.get("parts", []):
        try:
            name = p["name"]
            aabb = p["aabb"]
            t = p["transform_4x4"]
        except KeyError:
            continue
        aabb_min = np.array(aabb["min"], dtype=float)
        aabb_max = np.array(aabb["max"], dtype=float)
        transform = np.array(t, dtype=float).reshape(4, 4)
        parts.append(PartInfo(name=name, aabb_min=aabb_min, aabb_max=aabb_max, transform=transform))
    return parts

def compute_aabb(mesh: "trimesh.Trimesh") -> Tuple[np.ndarray, np.ndarray]:
    # trimesh 有 bounds
    return mesh.bounds[0].copy(), mesh.bounds[1].copy()

def load_mesh_any(path: Path) -> "trimesh.Trimesh":
    loaded = trimesh.load(str(path), force="mesh", skip_materials=True)
    if isinstance(loaded, trimesh.Scene):  # 退化
        if not loaded.geometry:  # pragma: no cover
            raise ValueError(f"空场景: {path}")
        meshes = [g for g in loaded.geometry.values() if isinstance(g, trimesh.Trimesh)]
        loaded = trimesh.util.concatenate(meshes)
    if not isinstance(loaded, trimesh.Trimesh):  # pragma: no cover
        raise TypeError("无法加载为 mesh")
    return loaded


def load_obj_parts(obj_path: Path):
    scene = trimesh.load(str(obj_path), force='scene', skip_materials=True)
    if isinstance(scene, trimesh.Trimesh):  # 单 mesh
        return {scene.metadata.get('name', 'mesh'): scene}
    mapping = {}
    for name, geom in scene.geometry.items():
        clean = str(name)
        if clean in mapping:
            # 合并重复名称
            mapping[clean] = trimesh.util.concatenate([mapping[clean], geom])
        else:
            mapping[clean] = geom
    return mapping


def pick_best_part(candidates: List[PartInfo], obj_parts: dict, input_mesh: "trimesh.Trimesh", sample: int) -> PartInfo:
    if len(candidates) == 1:
        return candidates[0]
    # 采样输入 mesh 点
    verts = input_mesh.vertices
    if len(verts) > sample:
        idx = np.random.default_rng(0).choice(len(verts), size=sample, replace=False)
        pts = verts[idx]
    else:
        pts = verts

    best = None
    best_score = -1.0
    centroid = input_mesh.center_mass if hasattr(input_mesh, 'center_mass') else verts.mean(axis=0)

    for part in candidates:
        # 找几何
        # 允许名称前后缀不一致: 用包含 / 精确 匹配
        geom = None
        if part.name in obj_parts:
            geom = obj_parts[part.name]
        else:
            # 尝试部分匹配 (第一次命中即用)
            for n, g in obj_parts.items():
                if part.name in n or n in part.name:
                    geom = g
                    break
        if geom is None:
            continue
        try:
            inside = geom.contains(pts)  # bool array
            ratio = float(np.count_nonzero(inside)) / len(pts)
        except Exception:  # contains 可能失败 (例如网格有孔)
            ratio = 0.0

        if ratio == 0.0:
            # 回退: 用质心距离 + AABB 体积占比
            gmin, gmax = geom.bounds
            center_g = 0.5 * (gmin + gmax)
            dist = np.linalg.norm(center_g - centroid)
            # 负距离作为得分 (越近越好)
            score = -dist
        else:
            score = ratio * 100.0  # 提高优先级

        if score > best_score:
            best_score = score
            best = part

    if best is None:
        raise RuntimeError("未能匹配到任何候选 part (细化阶段全部失败)")
    return best


def transform_to_local(mesh: "trimesh.Trimesh", transform_part_to_world: np.ndarray) -> "trimesh.Trimesh":
    # 我们希望得到 part 局部坐标: x_local = T^{-1} * x_world
    T_inv = np.linalg.inv(transform_part_to_world)
    verts_h = np.hstack([mesh.vertices, np.ones((len(mesh.vertices), 1))])
    verts_local = (T_inv @ verts_h.T).T[:, :3]
    local_mesh = mesh.copy()
    local_mesh.vertices[:] = verts_local
    return local_mesh


def save_mesh(mesh: "trimesh.Trimesh", path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(str(path))


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="基于 mapping.json 将 mesh 分配到 OBJ 的某个 part 并可转换到局部坐标")
    parser.add_argument("mesh", help="输入独立 mesh (obj/stl 等)")
    parser.add_argument("--obj", required=True, help="包含多个 part 的多组 OBJ")
    parser.add_argument("-m", "--mapping", required=True, help="mapping.json 路径")
    parser.add_argument("-o", "--outdir", required=False, default=None, help="输出目录")
    parser.add_argument("--epsilon", type=float, default=1e-4, help="AABB 包含判断松弛 eps")
    parser.add_argument("--sample", type=int, default=500, help="精细检测采样点上限")
    parser.add_argument("--save-local", action="store_true", help="保存到 part 局部坐标系的 mesh")
    parser.add_argument("--suffix-local", default="_local", help="局部坐标输出文件名后缀")
    args = parser.parse_args(argv)

    mesh_path = Path(args.mesh)
    obj_path = Path(args.obj)
    mapping_path = Path(args.mapping)
    if args.outdir is None:
        out_dir = mesh_path.parent
    else:
        out_dir = Path(args.outdir)

    if not mesh_path.exists():
        parser.error(f"mesh 不存在: {mesh_path}")
    if not obj_path.exists():
        parser.error(f"obj 不存在: {obj_path}")
    if not mapping_path.exists():
        parser.error(f"mapping 不存在: {mapping_path}")

    print("[1/6] 读取 mapping ...")
    parts = load_mapping(mapping_path)
    if not parts:
        parser.error("mapping.json 未包含 parts")
    print(f"    共 {len(parts)} 个 part")

    print("[2/6] 加载输入 mesh ...")
    input_mesh = load_mesh_any(mesh_path)
    in_min, in_max = compute_aabb(input_mesh)
    print(f"    输入 AABB min={in_min}, max={in_max}")

    print("[3/6] AABB 包含粗筛 ...")
    candidates = [p for p in parts if p.contains_aabb(in_min, in_max, eps=args.epsilon)]
    print(f"    候选数量(包含关系): {len(candidates)}")
    if not candidates:
        print("未找到完全包裹 mesh AABB 的 part (尝试放宽 eps 或检查输入)", file=sys.stderr)
        return 2

    print("[4/6] 加载 OBJ 子部分几何 ...")
    obj_parts = load_obj_parts(obj_path)
    print(f"    OBJ 子几何数量: {len(obj_parts)}")

    print("[5/6] 精细非凸检测 ...")
    best_part = pick_best_part(candidates, obj_parts, input_mesh, sample=args.sample)
    print(f"    选定 part: {best_part.name}")

    print("[6/6] 保存结果 ...")
    out_dir.mkdir(parents=True, exist_ok=True)
    base_name = mesh_path.stem
    world_out = out_dir / f"{base_name}_as_{best_part.name}{mesh_path.suffix}"
    save_mesh(input_mesh, world_out)

    local_out = None
    if args.save_local:
        local_mesh = transform_to_local(input_mesh, best_part.transform)
        local_out = out_dir / f"{base_name}_as_{best_part.name}{args.suffix_local}{mesh_path.suffix}"
        save_mesh(local_mesh, local_out)

    # 输出一个 JSON 描述
    summary = {
        "input_mesh": str(mesh_path),
        "matched_part": best_part.name,
        "world_output": str(world_out),
        "local_output": str(local_out) if local_out else None,
        "aabb_input": {"min": in_min.tolist(), "max": in_max.tolist()},
    }
    (out_dir / f"{base_name}_assign_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False))
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
