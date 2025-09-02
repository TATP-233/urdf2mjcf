"""根据 mapping.json 判断一个独立 mesh 属于 OBJ 模型的哪个 part 并做坐标转换保存。

使用流程示例:
    单个 mesh:
        python single_mesh.obj \\
            -g path/to/merged_model/visual.obj \\
            -m path/to/mapping.json \\
            -o output_dir

    多子部件 OBJ (自动拆分各 geometry):
        python multi_parts.obj \\
            -g path/to/merged_model/visual.obj \\
            -m path/to/mapping.json \\
            -o output_dir

主要步骤:
1. 读取输入 mesh (若为多子部件 OBJ 则展开)，对每个子部件计算其 AABB。
2. 读取 mapping.json 中每个 part 的 AABB, 先做 AABB “包含” 粗筛 (要求 mesh AABB 被 part AABB 完全包裹, 允许 eps 松弛)。
3. 对候选 part, 解析 OBJ (trimesh.Scene) 中对应子网格, 做精细非凸检测:
   - 采样输入 mesh 顶点 (<= --sample 点)
   - 使用 part_mesh.contains(points) (光线投射) 统计包含比例
   - 得分 = inside_ratio, 若都为 0 再用质心距离作为备选
4. 选出最佳 part.
5. 将每个匹配到的子部件 mesh 根据其 part 的 transform_4x4 变换到局部坐标系后保存: out_dir / <part_name> / <子部件名>.obj；保留材质 (若存在)。
6. 输出 JSON 描述 (多子部件时列出所有映射结果)。

注意: mapping.json 中 transform_4x4 视为 part->world 变换 (列/行主假设为常见行主 4x4)。
"""

from __future__ import annotations

import sys
import json
import argparse
from pathlib import Path
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np
import multiprocessing as mp

try:
    import trimesh
    from trimesh.exchange import obj as obj_io
    from trimesh.visual.material import SimpleMaterial  # type: ignore
except Exception as exc:
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


def _raw_load_obj_separate(path: Path) -> dict:
    """使用底层 load_obj 强制拆分 (object/group) 且不按材质合并, 返回 name->Trimesh.

    与默认 trimesh.load 不同: 这里设置 split_object=True, split_group=True, group_material=False,
    这样即使材质相同也不会被合并。"""
    with path.open('r', errors='ignore') as f:
        data = obj_io.load_obj(
            f,
            split_object=True,
            split_group=True,
            group_material=False,
            skip_materials=False,
            merge_vertices=False,
        )
    geoms = data.get('geometry', {})
    out = {}
    # geoms 可能是 dict{name: dict|Trimesh}
    if isinstance(geoms, dict):
        iterable = geoms.items()
    else:  # 兼容列表情形
        iterable = enumerate(geoms)
    for key, entry in iterable:
        name = str(key)
        g = entry
        if isinstance(g, trimesh.Trimesh):
            out[name] = g
            continue
        if isinstance(g, dict):
            verts = g.get('vertices')
            faces = g.get('faces')
            if verts is None or faces is None:
                continue
            try:
                # 若存在视觉/材质信息传入
                visual = g.get('visual') if 'visual' in g else None
                tm = trimesh.Trimesh(vertices=verts, faces=faces, visual=visual, process=False)
                tm.metadata['name'] = str(name)
                out[name] = tm
            except Exception:
                continue
    return out

def load_obj_parts(obj_path: Path):
    """加载 group OBJ，确保不因同材质合并。"""
    try:
        mapping = _raw_load_obj_separate(obj_path)
        if mapping:
            return mapping
    except Exception:
        pass  # 回退
    # 回退到普通方式 (可能发生合并)
    scene = trimesh.load(str(obj_path), force='scene', skip_materials=True)
    if isinstance(scene, trimesh.Trimesh):
        return {scene.metadata.get('name', 'mesh'): scene}
    mapping = {}
    for name, geom in scene.geometry.items():
        clean = str(name)
        if clean in mapping:
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


def _sanitize_obj_mtl(obj_path: Path):
    """确保: 1) mtllib 引用的 mtl 文件名 == obj 基名; 2) 删除所有纹理贴图引用 (map_*) 行。

    若原先生成的 mtl 名称不同则重命名并更新 obj 文件中的 mtllib 行。随后清理 mtl。"""
    if not obj_path.exists():
        return
    try:
        text = obj_path.read_text(errors='ignore').splitlines()
    except Exception:
        return
    mtllib_line_idx = None
    mtllib_name = None
    for i, line in enumerate(text):
        if line.startswith('mtllib '):
            mtllib_line_idx = i
            parts_line = line.strip().split(maxsplit=1)
            if len(parts_line) == 2:
                mtllib_name = parts_line[1].strip()
            break
    desired_mtl = obj_path.stem + '.mtl'
    mtl_path = None
    if mtllib_name:
        mtl_path = obj_path.parent / mtllib_name
    # 如果没有 mtllib 行但生成了默认 material.mtl, 也尝试处理
    if mtllib_name is None:
        # 可能存在 material.mtl 或 与 obj 同名 mtl
        cand1 = obj_path.parent / 'material.mtl'
        cand2 = obj_path.with_suffix('.mtl')
        if cand1.exists() and not cand2.exists():
            mtllib_name = cand1.name
            mtl_path = cand1
            # 插入一行 mtllib
            text.insert(0, f'mtllib {desired_mtl}')
            mtllib_line_idx = 0
        elif cand2.exists():
            mtllib_name = cand2.name
            mtl_path = cand2
            # 确保引用存在
            if not any(l.startswith('mtllib ') for l in text[:5]):
                text.insert(0, f'mtllib {desired_mtl}')
                mtllib_line_idx = 0
    # 重命名 mtl (若需要)
    if mtl_path and mtl_path.exists() and mtl_path.name != desired_mtl:
        target = obj_path.parent / desired_mtl
        try:
            if target.exists():
                target.unlink()
            mtl_path.rename(target)
            mtl_path = target
        except Exception:
            pass


def _parse_original_mtls(obj_path: Path) -> dict:
    """解析输入 OBJ 引用的 mtl 文件, 返回 {material_name: [lines]} 去除贴图行(map_*)。
    若找不到 mtllib 则返回空。"""
    base_dir = obj_path.parent
    try:
        lines = obj_path.read_text(errors='ignore').splitlines()
    except Exception:
        return {}
    mtl_files = []
    for l in lines:
        if l.startswith('mtllib '):
            name = l.split(maxsplit=1)[1].strip()
            if name:
                for part in name.split():  # 支持多个
                    mtl_files.append(base_dir / part)
    materials = {}
    for mtl in mtl_files:
        if not mtl.exists():
            continue
        try:
            mlines = mtl.read_text(errors='ignore').splitlines()
        except Exception:
            continue
        current = None
        buffer = []
        def _commit():
            if current and buffer:
                materials[current] = list(buffer)
        for raw in mlines:
            s = raw.strip()
            if s.lower().startswith('map_'):
                continue  # 删除贴图行
            if s.startswith('newmtl '):
                _commit()
                current = s.split(maxsplit=1)[1].strip()
                buffer = [raw]
            else:
                buffer.append(raw)
        _commit()
    return materials


def _parse_original_material_order(obj_path: Path) -> List[str]:
    """解析原始 OBJ 中出现的 usemtl 顺序 (去重保序)。"""
    order = []
    seen = set()
    try:
        for line in obj_path.read_text(errors='ignore').splitlines():
            if line.startswith('usemtl '):
                name = line.split(maxsplit=1)[1].strip()
                if name and name not in seen:
                    seen.add(name)
                    order.append(name)
    except Exception:
        pass
    return order


def _parse_obj_material_map(path: Path) -> dict:
    """解析 OBJ 中 o/g/usemtl -> material 统计映射 (返回 name->dominant_material)。"""
    try:
        lines = path.read_text(errors='ignore').splitlines()
    except Exception:
        return {}
    current_obj = None
    current_group = None
    current_mtl = None
    counts = {}
    def add_face():
        if current_mtl is None:
            return
        keys = []
        if current_obj and current_group:
            keys.append(current_group)
            keys.append(current_obj)
        elif current_obj:
            keys.append(current_obj)
        elif current_group:
            keys.append(current_group)
        for k in keys:
            d = counts.setdefault(k, {})
            d[current_mtl] = d.get(current_mtl, 0) + 1
    for line in lines:
        if not line or line.startswith('#'):
            continue
        if line.startswith('o '):
            current_obj = line.split(maxsplit=1)[1].strip()
        elif line.startswith('g '):
            current_group = line.split(maxsplit=1)[1].strip()
        elif line.startswith('usemtl '):
            current_mtl = line.split(maxsplit=1)[1].strip()
        elif line.startswith('f '):
            add_face()
    mapping = {}
    for k, d in counts.items():
        if not d:
            continue
        mat = max(d.items(), key=lambda kv: kv[1])[0]
        mapping[k] = mat
    return mapping


def _parse_per_component_material_segments(obj_path: Path) -> dict:
    """解析原始整体 OBJ, 记录每个 object/group 内部按 usemtl 分段的面数量序列。
    返回: {component_name: [ (material_name, face_count), ... ]}
    component_name 优先使用 group 名, 若无 group 则使用 object 名。
    """
    try:
        lines = obj_path.read_text(errors='ignore').splitlines()
    except Exception:
        return {}
    segments: dict[str, list[tuple[str, int]]] = {}
    current_obj = None
    current_group = None
    current_mat = None
    current_count = 0
    def flush():
        nonlocal current_count
        if current_mat is None or current_count == 0:
            return
        key = current_group or current_obj
        if not key:
            return
        lst = segments.setdefault(key, [])
        lst.append((current_mat, current_count))
        current_count = 0
    for line in lines:
        if not line or line.startswith('#'):
            continue
        if line.startswith('o '):
            flush()
            current_obj = line.split(maxsplit=1)[1].strip()
            current_group = None
            current_mat = None
            current_count = 0
        elif line.startswith('g '):
            flush()
            current_group = line.split(maxsplit=1)[1].strip()
            current_mat = None
            current_count = 0
        elif line.startswith('usemtl '):
            flush()
            current_mat = line.split(maxsplit=1)[1].strip()
        elif line.startswith('f '):
            # 只有在有材质与对象/组时计数
            if (current_group or current_obj) and current_mat:
                current_count += 1
    flush()
    return segments


def _restore_per_face_material_segments(obj_path: Path, segments: dict, component_name: str, original_materials: dict):
    """将单个局部导出的 OBJ (只有顶点/法线/纹理 + faces) 按原始 segments 重写 usemtl 分段。

    要求: segments[component_name] 总面数 == 当前文件面数。若不一致则跳过。
    在重写过程中会移除原有 usemtl 行, 重新按段插入。保留顶点/法线/纹理/对象行顺序。
    重写后更新对应 mtl (若提供 original_materials)。
    """
    seg_list = None
    # 允许模糊匹配: 精确, 或 component_name 是 key 子串/反之 (取第一个匹配)
    if component_name in segments:
        seg_list = segments[component_name]
    else:
        for k in segments.keys():
            if component_name in k or k in component_name:
                seg_list = segments[k]
                break
    if not seg_list:
        return
    try:
        lines = obj_path.read_text(errors='ignore').splitlines()
    except Exception:
        return
    face_indices = [i for i,l in enumerate(lines) if l.startswith('f ')]
    if not face_indices:
        return
    total_faces_current = len(face_indices)
    total_faces_expected = sum(c for _,c in seg_list)
    if total_faces_current != total_faces_expected:
        # 面数量不一致，放弃重建 (可能处理过程改变了拓扑)
        return
    # 构造新内容
    new_lines = []
    face_counter = 0
    seg_ptr = 0
    seg_face_acc = 0
    next_seg_threshold = seg_list[0][1] if seg_list else None
    # 先移除所有原 usemtl
    for l in lines:
        if l.startswith('usemtl '):
            continue
        new_lines.append(l)
    # 重新插入: 需要再次遍历并在对应 face 前插入 usemtl
    # 为简单, 我们二次遍历 new_lines, 替换 faces 部分
    final_lines = []
    seg_ptr = 0
    written_faces = 0
    current_material = seg_list[0][0] if seg_list else None
    seg_limit = seg_list[0][1] if seg_list else 0
    for l in new_lines:
        if l.startswith('f '):
            if current_material is not None and written_faces == 0:
                final_lines.append(f'usemtl {current_material}')
            elif current_material is not None and written_faces >= seg_limit:
                # 进入下一段
                seg_ptr += 1
                if seg_ptr >= len(seg_list):
                    # 超出, 保持最后材质
                    pass
                else:
                    current_material, seg_limit = seg_list[seg_ptr]
                    written_faces = 0
                    final_lines.append(f'usemtl {current_material}')
            final_lines.append(l)
            written_faces += 1
            continue
        final_lines.append(l)
    try:
        obj_path.write_text('\n'.join(final_lines).rstrip() + '\n')
    except Exception:
        return
    # 重建 mtl 仅包含涉及材质
    if original_materials and seg_list:
        used = []
        seen = set()
        for m,_ in seg_list:
            if m not in seen:
                seen.add(m)
                used.append(m)
        blocks = []
        for m in used:
            blk = original_materials.get(m)
            if not blk:
                continue
            for raw in blk:
                if raw.strip().lower().startswith('map_'):
                    continue
                blocks.append(raw)
        if blocks:
            try:
                obj_path.with_suffix('.mtl').write_text('\n'.join(blocks).rstrip() + '\n')
            except Exception:
                pass


def _rewrite_obj_material_names(obj_path: Path, original_order: List[str]):
    """将导出的 OBJ 中的 usemtl 名称映射到原始顺序名称。
    规则: 按导出文件中首次出现的 usemtl 顺序与 original_order 对位; 多余循环; 原顺序为空则不处理。"""
    if not original_order or not obj_path.exists():
        return
    try:
        lines = obj_path.read_text(errors='ignore').splitlines()
    except Exception:
        return
    export_names = []
    name_map = {}
    # 收集导出 usemtl 名称顺序
    for l in lines:
        if l.startswith('usemtl '):
            nm = l.split(maxsplit=1)[1].strip()
            if nm not in export_names:
                export_names.append(nm)
    if not export_names:
        return
    # 构建映射
    for idx, ename in enumerate(export_names):
        mapped = original_order[idx % len(original_order)]
        name_map[ename] = mapped
    changed = False
    for i, l in enumerate(lines):
        if l.startswith('usemtl '):
            nm = l.split(maxsplit=1)[1].strip()
            if nm in name_map and name_map[nm] != nm:
                lines[i] = f'usemtl {name_map[nm]}'
                changed = True
    if changed:
        try:
            obj_path.write_text('\n'.join(lines) + '\n')
        except Exception:
            pass


def _rebuild_mtl_from_original(obj_path: Path, original_materials: dict):
    """根据 obj 中实际使用的 usemtl 顺序，重建对应 mtl 文件内容 (移除贴图)。"""
    if not obj_path.exists() or not original_materials:
        return
    try:
        lines = obj_path.read_text(errors='ignore').splitlines()
    except Exception:
        return
    used = []
    seen = set()
    for l in lines:
        if l.startswith('usemtl '):
            nm = l.split(maxsplit=1)[1].strip()
            if nm and nm not in seen:
                seen.add(nm)
                used.append(nm)
    if not used:
        return
    mtl_path = obj_path.with_suffix('.mtl')
    blocks = []
    for nm in used:
        block = original_materials.get(nm)
        if not block:
            continue
        for raw in block:
            if raw.strip().lower().startswith('map_'):
                continue
            blocks.append(raw)
        if blocks and not blocks[-1].startswith('newmtl'):
            blocks.append('')
    if blocks:
        try:
            mtl_path.write_text('\n'.join(blocks).rstrip() + '\n')
        except Exception:
            pass


def _extract_first_usemtl(obj_path: Path) -> str | None:
    try:
        for line in obj_path.read_text(errors='ignore').splitlines():
            if line.startswith('usemtl '):
                return line.split(maxsplit=1)[1].strip()
    except Exception:
        return None
    return None


def _repair_merged_materials(merged_obj: Path, component_paths: list[Path], original_materials: dict):
    """尝试按组件顺序将 merged_obj 中的 usemtl 替换为各子件首个材质名，并用原始材质块重建 mtl。"""
    if not merged_obj.exists():
        return
    desired = []
    for p in component_paths:
        m = _extract_first_usemtl(Path(p))
        if m:
            desired.append(m)
    if not desired:
        return
    try:
        lines = merged_obj.read_text(errors='ignore').splitlines()
    except Exception:
        return
    new_lines = []
    mat_idx = 0
    pending_insert = None  # material name waiting to insert after an object line
    for line in lines:
        if line.startswith('o ') or line.startswith('g '):
            new_lines.append(line)
            if mat_idx < len(desired):
                pending_insert = desired[mat_idx]
            continue
        if line.startswith('usemtl '):
            if mat_idx < len(desired):
                new_lines.append(f'usemtl {desired[mat_idx]}')
                mat_idx += 1
            else:
                # 多余的保持
                new_lines.append(line)
            pending_insert = None
            continue
        # 插入等待的 usemtl
        if pending_insert and (line.startswith('v ') or line.startswith('vn ') or line.startswith('vt ') or line.startswith('f ')):
            new_lines.append(f'usemtl {pending_insert}')
            mat_idx += 1
            pending_insert = None
        new_lines.append(line)
    # 写回
    try:
        merged_obj.write_text('\n'.join(new_lines) + '\n')
    except Exception:
        pass
    # 重建 mtl 仅包含所需材质
    if original_materials:
        mtl_path = merged_obj.with_suffix('.mtl')
        blocks = []
        added = set()
        for m in desired:
            if m in added:
                continue
            block = original_materials.get(m)
            if not block:
                continue
            for raw in block:
                if raw.strip().lower().startswith('map_'):
                    continue
                blocks.append(raw)
            added.add(m)
        if blocks:
            try:
                mtl_path.write_text('\n'.join(blocks).rstrip() + '\n')
            except Exception:
                pass


def _rewrite_mtl_with_original(mtl_path: Path, original_materials: dict):
    """将导出后的 mtl 替换为原始属性(去贴图)。仅保留当前 mtl 中出现的 material 名称顺序。"""
    if not mtl_path or not mtl_path.exists() or not original_materials:
        return
    try:
        lines = mtl_path.read_text(errors='ignore').splitlines()
    except Exception:
        return
    order = []
    for l in lines:
        if l.startswith('newmtl '):
            name = l.split(maxsplit=1)[1].strip()
            order.append(name)
    if not order:
        return
    out_lines = []
    for name in order:
        src = original_materials.get(name)
        if src:
            out_lines.extend(src)
            if not src[-1].endswith('\n'):
                out_lines.append('')
    try:
        mtl_path.write_text('\n'.join(out_lines).rstrip() + '\n')
    except Exception:
        pass


def _adjust_face_indices(face_line: str, v_offset: int, vt_offset: int, vn_offset: int) -> str:
    """调整一个 f 行内的索引 (支持 v, v/vt, v//vn, v/vt/vn)。

    face_line: 原始例如 'f 1/2/3 4/5/6 7/8/9'。
    *_offset: 需要加到对应索引上的偏移量 (注意 OBJ 索引从 1 开始)。
    """
    try:
        parts = face_line.strip().split()
        if len(parts) < 4 or parts[0] != 'f':
            return face_line
        out_tokens = ['f']
        for token in parts[1:]:
            if '/' not in token:
                # 只有顶点
                vidx = int(token) + v_offset
                out_tokens.append(str(vidx))
                continue
            a = token.split('/')
            # 可能长度 2 或 3; 空字符串表示缺省
            v_str = a[0]
            vt_str = a[1] if len(a) >= 2 else ''
            vn_str = a[2] if len(a) >= 3 else ''
            def _add(idx_str: str, offset: int) -> str:
                if idx_str == '' or idx_str == '0':
                    return idx_str
                try:
                    return str(int(idx_str) + offset)
                except Exception:
                    return idx_str
            v_new = _add(v_str, v_offset)
            vt_new = _add(vt_str, vt_offset)
            vn_new = _add(vn_str, vn_offset)
            if vn_str != '' or (len(a) == 3):
                out_tokens.append(f"{v_new}/{vt_new}/{vn_new}")
            elif vt_str != '':
                out_tokens.append(f"{v_new}/{vt_new}")
            else:
                out_tokens.append(v_new)
        return ' '.join(out_tokens)
    except Exception:
        return face_line


def _manual_merge_component_objs(part_name: str, component_paths: List[Path], out_dir: Path, original_materials: dict) -> Path | None:
    """手工合并多个局部坐标子 OBJ，完整保留其内部的多材质 usemtl 分段。

    逻辑:
      1. 逐文件解析 v/vt/vn/f/usemtl/o/g 行, 累加顶点并调整 f 索引。
      2. 忽略子文件内部的 mtllib 行; 顶层写一个 mtllib <part_name>.mtl。
      3. 汇总所有使用到的材质名 (usemtl) 并基于 original_materials 重建 mtl 文件 (去贴图)。
    优点: 不再依赖 trimesh.Scene 导出, 避免材质折叠与首材质替换问题。
    """
    if not component_paths:
        return None
    merged_obj = out_dir / f"{part_name}.obj"
    merged_mtl = out_dir / f"{part_name}.mtl"
    out_dir.mkdir(parents=True, exist_ok=True)
    v_offset = 0
    vt_offset = 0
    vn_offset = 0
    used_materials: List[str] = []
    used_set = set()
    out_lines = [f"mtllib {merged_mtl.name}"]
    for comp_idx, comp_path in enumerate(component_paths):
        if not Path(comp_path).exists():
            continue
        try:
            lines = Path(comp_path).read_text(errors='ignore').splitlines()
        except Exception:
            continue
        out_lines.append(f"# component {comp_idx}: {Path(comp_path).name}")
        has_object = any(l.startswith('o ') for l in lines[:20])
        if not has_object:
            # 添加一个 object 名称, 避免后续材质漂移
            out_lines.append(f"o {Path(comp_path).stem}")
        current_material = None
        for line in lines:
            if not line or line.startswith('#'):
                continue
            if line.startswith('mtllib '):
                continue  # 忽略子文件 mtllib
            if line.startswith('v '):
                out_lines.append(line)
                v_offset += 1
                continue
            if line.startswith('vt '):
                out_lines.append(line)
                vt_offset += 1
                continue
            if line.startswith('vn '):
                out_lines.append(line)
                vn_offset += 1
                continue
            if line.startswith('usemtl '):
                current_material = line.split(maxsplit=1)[1].strip()
                out_lines.append(line)
                if current_material and current_material not in used_set:
                    used_set.add(current_material)
                    used_materials.append(current_material)
                continue
            if line.startswith('o ') or line.startswith('g '):
                # 保留结构; 避免顶层统一 object 影响 grouping
                out_lines.append(line)
                continue
            if line.startswith('f '):
                # faces 需要调整索引: 但是我们当前 v_offset 代表已经写入的数量, 所以需要使用 (当前累计 - 本行新增前)。
                # 因为我们在添加 v/vt/vn 时已经递增 offset, 这里需要减掉本文件新增的数量? 简化: 先在遍历前缓存 offsets.
                pass
        # 第二遍处理: 为了正确调整索引, 我们需要重新遍历, 所以改成两阶段: 先收集, 再写入.
    # 重新实现 (上面第一次实现中断) --------
    v_offset = 0
    vt_offset = 0
    vn_offset = 0
    out_lines = [f"mtllib {merged_mtl.name}"]
    used_materials.clear()
    used_set.clear()
    for comp_idx, comp_path in enumerate(component_paths):
        path_obj = Path(comp_path)
        if not path_obj.exists():
            continue
        try:
            lines = path_obj.read_text(errors='ignore').splitlines()
        except Exception:
            continue
        # 预扫描: 统计该组件内部的 v/vt/vn 数量
        v_count = sum(1 for l in lines if l.startswith('v '))
        vt_count = sum(1 for l in lines if l.startswith('vt '))
        vn_count = sum(1 for l in lines if l.startswith('vn '))
        out_lines.append(f"# component {comp_idx}: {path_obj.name}")
        has_object = any(l.startswith('o ') for l in lines[:20])
        if not has_object:
            out_lines.append(f"o {path_obj.stem}")
        # 写入几何 (直接复制 v/vt/vn)
        for l in lines:
            if l.startswith('mtllib '):
                continue
            if l.startswith('v ') or l.startswith('vt ') or l.startswith('vn '):
                out_lines.append(l)
        # 第二遍写拓扑和结构
        current_material = None
        for l in lines:
            if l.startswith('mtllib '):
                continue
            if l.startswith('usemtl '):
                current_material = l.split(maxsplit=1)[1].strip()
                out_lines.append(l)
                if current_material and current_material not in used_set:
                    used_set.add(current_material)
                    used_materials.append(current_material)
                continue
            if l.startswith('o ') or l.startswith('g '):
                out_lines.append(l)
                continue
            if l.startswith('f '):
                adjusted = _adjust_face_indices(l, v_offset, vt_offset, vn_offset)
                out_lines.append(adjusted)
        # 更新全局 offset
        v_offset += v_count
        vt_offset += vt_count
        vn_offset += vn_count
    try:
        merged_obj.write_text('\n'.join(out_lines).rstrip() + '\n')
    except Exception:
        return None
    # 构建 mtl
    if original_materials and used_materials:
        mtl_blocks = []
        for m in used_materials:
            block = original_materials.get(m)
            if not block:
                continue
            for raw in block:
                if raw.strip().lower().startswith('map_'):
                    continue
                mtl_blocks.append(raw)
        if mtl_blocks:
            try:
                merged_mtl.write_text('\n'.join(mtl_blocks).rstrip() + '\n')
            except Exception:
                pass
    return merged_obj if merged_obj.exists() else None


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="基于 mapping.json 将独立 mesh 或多子部件 OBJ 分配到对应 part 并输出局部坐标")
    parser.add_argument("mesh", help="输入 mesh (obj/stl 等); 若为 multi-geometry OBJ 将逐子部件处理")
    parser.add_argument("-g", "--groups", required=True, help="包含多个 part 的多组 OBJ (整体) 用于精细匹配")
    parser.add_argument("-m", "--mapping", required=True, help="mapping.json 路径")
    parser.add_argument("-o", "--outdir", required=False, default=None, help="输出目录 (默认: ./meshes_aligned)")
    parser.add_argument("--epsilon", type=float, default=5e-3, help="AABB 包含判断松弛 eps")
    parser.add_argument("--sample", type=int, default=500, help="精细检测采样点上限")
    parser.add_argument("-nw", "--num-workers", type=int, default=0, help="并行工作线程数 (1=单线程)")
    args = parser.parse_args(argv)

    mesh_path = Path(args.mesh)
    group_obj_path = Path(args.groups)
    mapping_path = Path(args.mapping)
    out_dir = Path(args.outdir) if args.outdir else (mesh_path.parent / "meshes_aligned")

    if not mesh_path.exists():
        parser.error(f"mesh 不存在: {mesh_path}")
    if not group_obj_path.exists():
        parser.error(f"group obj 不存在: {group_obj_path}")
    if not mapping_path.exists():
        parser.error(f"mapping 不存在: {mapping_path}")

    print("[1/7] 读取 mapping ...")
    parts = load_mapping(mapping_path)
    if not parts:
        parser.error("mapping.json 未包含 parts")
    print(f"    共 {len(parts)} 个 part")

    print("[2/7] 解析输入 mesh/scene ...")
    submeshes: List[Tuple[str, trimesh.Trimesh]] = []
    multi = False
    original_mtl_map = {}
    original_mtl_order: List[str] = []
    name_to_material = {}
    if mesh_path.suffix.lower() == '.obj':
        original_mtl_map = _parse_original_mtls(mesh_path)
        original_mtl_order = _parse_original_material_order(mesh_path)
        name_to_material = _parse_obj_material_map(mesh_path)
        original_component_segments = _parse_per_component_material_segments(mesh_path)
        # 使用底层拆分，保持最细粒度 (不按材质合并)
        try:
            split_geoms = _raw_load_obj_separate(mesh_path)
        except Exception:
            split_geoms = {}
        if len(split_geoms) > 1:
            multi = True
            for name, geom in split_geoms.items():
                clean_name = str(name).replace('/', '_')
                submeshes.append((clean_name, geom))
        elif len(split_geoms) == 1:
            # 只有一个, 仍视为单 mesh
            only_mesh = next(iter(split_geoms.values()))
            submeshes.append((mesh_path.stem, only_mesh))
        else:
            # 回退到普通加载
            m = load_mesh_any(mesh_path)
            submeshes.append((mesh_path.stem, m))
    else:
        original_component_segments = {}
        m = load_mesh_any(mesh_path)
        submeshes.append((mesh_path.stem, m))
    print(f"    子部件数量: {len(submeshes)} (multi={multi})")

    print("[3/7] 加载组 OBJ 子几何 ...")
    obj_parts = load_obj_parts(group_obj_path)
    print(f"    组 OBJ 几何数量: {len(obj_parts)}")

    assignments = []
    failures = []

    if args.num_workers == 0:
        args.num_workers = max(1, mp.cpu_count() // 2)
    print("[4/7] 遍历子部件并做 AABB 包含粗筛 + 精细匹配 ... (workers={})".format(args.num_workers))

    def _process_one(sub_name: str, sub_mesh: "trimesh.Trimesh"):
        in_min, in_max = compute_aabb(sub_mesh)
        candidates = [p for p in parts if p.contains_aabb(in_min, in_max, eps=args.epsilon)]
        if not candidates:
            return None, {
                'submesh': sub_name,
                'reason': 'no_part_contains_aabb',
                'aabb': {'min': in_min.tolist(), 'max': in_max.tolist()}
            }
        try:
            best_part = pick_best_part(candidates, obj_parts, sub_mesh, sample=args.sample)
        except Exception as exc:
            return None, {'submesh': sub_name, 'reason': f'match_error: {exc}'}
        local_mesh = transform_to_local(sub_mesh, best_part.transform)
        # 替换为原始材质名 (若存在且当前为占位)
        if name_to_material:
            orig_mat = None
            if sub_name in name_to_material:
                orig_mat = name_to_material[sub_name]
            else:
                for k, v in name_to_material.items():
                    if k in sub_name or sub_name in k:
                        orig_mat = v
                        break
            if orig_mat:
                try:
                    current_name = None
                    if hasattr(local_mesh.visual, 'material') and local_mesh.visual.material is not None:
                        try:
                            current_name = local_mesh.visual.material.name
                        except Exception:
                            pass
                    if (current_name is None) or str(current_name).startswith('material_'):
                        diffuse = None
                        try:
                            diffuse = local_mesh.visual.material.diffuse  # type: ignore
                        except Exception:
                            pass
                        local_mesh.visual.material = SimpleMaterial(name=orig_mat, diffuse=diffuse)
                except Exception:
                    pass
        part_dir = out_dir / best_part.name
        part_dir.mkdir(parents=True, exist_ok=True)
        out_path = part_dir / f"{sub_name}.obj"
        try:
            local_mesh.export(str(out_path))
        except Exception:
            try:
                local_mesh.visual = None  # type: ignore
                local_mesh.export(str(out_path))
            except Exception as exc2:
                return None, {'submesh': sub_name, 'reason': f'export_failed: {exc2}'}
        # 移除贴图并统一 mtl 名称
        _sanitize_obj_mtl(out_path)
        # 重写 OBJ 内 usemtl 为原始名称
        _rewrite_obj_material_names(out_path, original_mtl_order)
    # 使用原始材质属性替换 (仅写入当前使用的材质 block)
        mtl_file = out_path.with_suffix('.mtl')
        if mtl_file.exists():
            # 直接用原始所有材质(或子集)覆盖; 若子集需筛选可以基于 OBJ 中 usemtl 集合
            if original_mtl_map:
                # 获取当前 OBJ 用到的材质名集合
                try:
                    used = []
                    for l in out_path.read_text(errors='ignore').splitlines():
                        if l.startswith('usemtl '):
                            nm = l.split(maxsplit=1)[1].strip()
                            if nm and nm not in used:
                                used.append(nm)
                except Exception:
                    used = []
                # 组装写入
                out_lines = []
                source_names = used if used else original_mtl_order
                if not source_names:
                    source_names = list(original_mtl_map.keys())
                for nm in source_names:
                    block = original_mtl_map.get(nm)
                    if block:
                        out_lines.extend([ln for ln in block if not ln.strip().lower().startswith('map_')])
                        if out_lines and not out_lines[-1].startswith('newmtl'):
                            out_lines.append('')
                if out_lines:
                    try:
                        mtl_file.write_text('\n'.join(out_lines).rstrip() + '\n')
                    except Exception:
                        pass
        # 恢复原始 object/group 内部多材质分段 (若存在)
        if mesh_path.suffix.lower() == '.obj':
            try:
                _restore_per_face_material_segments(out_path, original_component_segments, sub_name, original_mtl_map)
            except Exception:
                pass
        # 再次去掉可能残留 map_ 行
        _rewrite_mtl_with_original(mtl_file, original_mtl_map)
        rec = {
            'submesh': sub_name,
            'matched_part': best_part.name,
            'output': str(out_path),
            'aabb_input': {'min': in_min.tolist(), 'max': in_max.tolist()}
        }
        return rec, None

    if args.num_workers == 1 or len(submeshes) == 1:
        for sub_name, sub_mesh in submeshes:
            rec, fail = _process_one(sub_name, sub_mesh)
            if rec:
                assignments.append(rec)
            if fail:
                failures.append(fail)
    else:
        from concurrent.futures import ThreadPoolExecutor, as_completed
        with ThreadPoolExecutor(max_workers=args.num_workers) as ex:
            futs = {ex.submit(_process_one, n, m): n for n, m in submeshes}
            for fut in as_completed(futs):
                try:
                    rec, fail = fut.result()
                    if rec:
                        assignments.append(rec)
                    if fail:
                        failures.append(fail)
                except Exception as exc:  # pragma: no cover
                    failures.append({'submesh': futs[fut], 'reason': f'worker_exception: {exc}'})

    merged_outputs = []
    unmatched_aggregate_path = None
    if multi and assignments:
        print("[5/8] 合并同 part 子部件 -> 生成整体 OBJ+MTL ...")
        # 按 part 分组
        from collections import defaultdict
        part_groups = defaultdict(list)
        for rec in assignments:
            part_groups[rec['matched_part']].append(rec['output'])
        for part_name, paths in part_groups.items():
            # 使用自定义合并保留全部 usemtl
            try:
                merged_path = _manual_merge_component_objs(part_name, [Path(p) for p in paths], out_dir, original_mtl_map)
                if merged_path:
                    merged_outputs.append({'part': part_name, 'obj': str(merged_path)})
                else:
                    print(f"    [WARN] part {part_name} 合并失败 (结果为空)", file=sys.stderr)
            except Exception as exc:
                print(f"    [WARN] 处理 part {part_name} 合并异常: {exc}", file=sys.stderr)
    else:
        print("[5/8] 跳过合并 (非多子部件或无成功匹配)")

    # 额外: 汇总未匹配子部件成一个 OBJ (保留原世界坐标与材质)
    if failures:
        try:
            fail_names = {f['submesh'] for f in failures if 'submesh' in f}
            if fail_names:
                scene_unmatched = trimesh.Scene()
                for sub_name, sub_mesh in submeshes:
                    if sub_name in fail_names:
                        try:
                            scene_unmatched.add_geometry(sub_mesh, node_name=sub_name)
                        except Exception:
                            pass
                if len(scene_unmatched.geometry) > 0:
                    unmatched_aggregate_path = out_dir / 'unmatched.obj'
                    try:
                        scene_unmatched.export(str(unmatched_aggregate_path))
                        _sanitize_obj_mtl(unmatched_aggregate_path)
                        _rebuild_mtl_from_original(unmatched_aggregate_path, original_mtl_map)
                        print(f"[额外] 未匹配聚合输出: {unmatched_aggregate_path}")
                    except Exception as exc:
                        print(f"[WARN] 未匹配聚合导出失败: {exc}", file=sys.stderr)
        except Exception as exc:  # pragma: no cover
            print(f"[WARN] 未匹配聚合步骤异常: {exc}", file=sys.stderr)

    print("[6/8] 汇总结果 ...")
    out_dir.mkdir(parents=True, exist_ok=True)
    summary = {
        'input_mesh': str(mesh_path),
        'multi_geometry': multi,
        'assignments': assignments,
        'failures': failures,
        'merged_parts': merged_outputs,
        'total_submeshes': len(submeshes),
        'matched': len(assignments),
        'unmatched': len(failures),
        'epsilon': args.epsilon,
        'unmatched_aggregate': str(unmatched_aggregate_path) if unmatched_aggregate_path else None,
    'material_mapping_size': len(name_to_material),
    }
    summary_path = out_dir / f"{mesh_path.stem}_assign_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False))

    print("[7/8] 统计: 成功 {}/{}".format(len(assignments), len(submeshes)))
    if failures:
        print("[8/8] 有未匹配子部件: {}".format(len(failures)), file=sys.stderr)
        if len(assignments) == 0:
            return 3
    else:
        print("[8/8] 全部子部件匹配成功")
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
