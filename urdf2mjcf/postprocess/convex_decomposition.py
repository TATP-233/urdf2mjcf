import argparse
import logging
import xml.etree.ElementTree as ET
from pathlib import Path

import coacd
import trimesh
from urdf2mjcf.utils import save_xml

logger = logging.getLogger(__name__)

def convex_decomposition_assets(mjcf_path: str | Path, root: ET.Element) -> None:
    """对 MJCF 文件中 collision 类型的 geom 进行凸分解。

    Args:
        mjcf_path: MJCF 文件的路径
        root: MJCF 文件的根元素
    """
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    compiler.attrib["meshdir"] = "."

    dir_path = mjcf_path.parent / compiler.attrib["meshdir"]

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
    
    # 存储已处理的 mesh，避免重复分解
    processed_meshes = set()
    new_mesh_parts = {}  # mesh_name -> [(part_name, part_file), ...]
    
    for body, geom in collision_geoms:
        mesh_name = geom.attrib["mesh"]
        
        # 如果已经处理过这个 mesh，跳过
        if mesh_name in processed_meshes:
            continue
            
        if mesh_name not in mesh_assets:
            logger.warning(f"Mesh {mesh_name} not found in assets.")
            continue
            
        mesh_file = dir_path / mesh_assets[mesh_name]
        if not mesh_file.exists():
            logger.warning(f"Mesh file {mesh_assets[mesh_name]} does not exist.")
            continue
        
        logger.info(f"Processing mesh {mesh_name} for convex decomposition")
        
        # 加载 mesh
        try:
            mesh_data = trimesh.load(mesh_file, force="mesh")
        except Exception as e:
            logger.error(f"Failed to load mesh {mesh_file}: {e}")
            continue
        
        # 执行凸分解
        try:
            mesh_coacd = coacd.Mesh(mesh_data.vertices, mesh_data.faces)
            parts = coacd.run_coacd(mesh_coacd)
            logger.info(f"Mesh {mesh_name} decomposed into {len(parts)} parts")
            
            # 如果只有一个part，说明本身就是凸的，无需处理
            if len(parts) <= 1:
                logger.info(f"Mesh {mesh_name} is already convex (only {len(parts)} part), skipping")
                processed_meshes.add(mesh_name)  # 标记为已处理，避免重复检查
                continue
                
        except Exception as e:
            logger.error(f"Failed to decompose mesh {mesh_name}: {e}")
            continue
        
        # 创建 parts 文件夹
        mesh_stem = mesh_file.stem  # 文件名不带扩展名
        parts_dir = mesh_file.parent / f"{mesh_stem}_parts"
        parts_dir.mkdir(exist_ok=True)
        
        # 保存每个 part
        part_info = []
        for i, part in enumerate(parts):
            try:
                convex_mesh = trimesh.Trimesh(vertices=part[0], faces=part[1])
                part_filename = f"{mesh_stem}_part{i+1}.stl"
                part_file = parts_dir / part_filename
                convex_mesh.export(part_file)
                
                # 相对于 meshdir 的路径，保持原始目录结构
                original_file_relative = mesh_assets[mesh_name]  # 例如: meshes/leg/leg_link1.stl
                original_dir = str(Path(original_file_relative).parent)  # 例如: meshes/leg
                relative_part_path = f"{original_dir}/{mesh_stem}_parts/{part_filename}"
                part_name = f"{mesh_stem}_part{i+1}"
                part_info.append((part_name, relative_part_path))
                
                logger.info(f"Saved part {i+1} to {relative_part_path}")
            except Exception as e:
                logger.error(f"Failed to save part {i+1} of mesh {mesh_name}: {e}")
        
        if part_info:
            new_mesh_parts[mesh_name] = part_info
            processed_meshes.add(mesh_name)
    
    # 更新 asset 部分
    for mesh_name, part_info in new_mesh_parts.items():
        # 删除原始 mesh
        for mesh in asset.findall("mesh"):
            if mesh.attrib["name"] == mesh_name:
                asset.remove(mesh)
                break
        
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


def convex_decomposition(mjcf_path: str | Path) -> None:
    """对 MJCF 文件进行凸分解处理。

    Args:
        mjcf_path: MJCF 文件的路径
    """
    tree = ET.parse(mjcf_path)
    root = tree.getroot()
    convex_decomposition_assets(mjcf_path, root)

    save_xml(mjcf_path, tree)


def main() -> None:
    parser = argparse.ArgumentParser(description="对 MJCF 文件中的 collision mesh 进行凸分解")
    parser.add_argument("mjcf_path", type=Path, help="MJCF 文件的路径")
    args = parser.parse_args()
    
    logging.basicConfig(level=logging.INFO)
    convex_decomposition(args.mjcf_path)


if __name__ == "__main__":
    main()