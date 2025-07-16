"""Defines a post-processing function that updates the mesh of the Mujoco model.

This script updates the mesh of the MJCF file.
"""

import os
import argparse
import logging
import xml.etree.ElementTree as ET
from pathlib import Path

from urdf2mjcf.utils import save_xml

logger = logging.getLogger(__name__)

def update_mesh_assets(mjcf_path: str | Path, root: ET.Element, max_vertices: int) -> None:
    """Update the mesh assets of the MJCF file.

    Args:
        root: The root element of the MJCF file.
    """
    import pymeshlab
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    compiler.attrib["meshdir"] = "."

    dir_path = mjcf_path.parent / compiler.attrib["meshdir"]

    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")

    for mesh in asset.findall("mesh"):
        mesh_file = dir_path / mesh.attrib["file"]
        if not mesh_file.exists():
            logger.warning(f"Mesh file {mesh.attrib['file']} does not exist.")
            continue
        
        # 检查文件扩展名
        if not mesh_file.suffix.lower() in ['.obj', '.stl']:
            logger.warning(f"Unsupported mesh format: {mesh_file.suffix}")
            continue
        
        if mesh.attrib["file"].startswith('./'):
            mesh.attrib["file"] = str(mesh_file.relative_to(dir_path))

        try:
            # 使用PyMeshLab加载网格获取顶点数
            ms = pymeshlab.MeshSet()
            ms.load_new_mesh(str(mesh_file))
            
            max_simple_times = 5
            vertices = ms.current_mesh().vertex_matrix()
            if len(vertices) >= max_vertices:
                while len(vertices) >= max_vertices:
                    max_simple_times -= 1
                    if max_simple_times <= 0:
                        break
                    nm = min(max_vertices, len(vertices))
                    percentage = nm / max_vertices * 0.999
                    ms.meshing_decimation_clustering(threshold=pymeshlab.PercentageValue(percentage))
                    vertices = ms.current_mesh().vertex_matrix()
                ms.save_current_mesh(str(mesh_file))
                if os.path.exists(mesh_file.with_suffix('.mtl')):
                    os.remove(mesh_file.with_suffix('.mtl'))
                elif os.path.exists(str(mesh_file)+'.mtl'):
                    os.remove(str(mesh_file)+'.mtl')

        except Exception as e:
            logger.error(f"处理网格文件 {mesh_file} 时出错: {e}")
            continue

    save_xml(mjcf_path, root)

def collision_to_stl(mjcf_path: str | Path) -> None:
    """Convert collision meshes to STL format in the MJCF file.

    Args:
        mjcf_path: The path to the MJCF file to process.
    """
    tree = ET.parse(mjcf_path)
    root = tree.getroot()

    compiler = root.find("compiler")
    mesh_dir_path = mjcf_path.parent / compiler.attrib["meshdir"]
    asset = root.find("asset")
    
    for geom in root.iter("geom"):
        if geom.attrib.get("type") == "mesh":
            mesh_name = geom.attrib.get("mesh")
            class_name = geom.attrib.get("class")
            if class_name == "collision" and Path(mesh_name).suffix.lower() != ".stl":
                logger.info(f"Converting collision mesh {mesh_name} to STL format.")
                for mesh in asset.findall("mesh"):
                    if mesh.attrib.get("name") == mesh_name:
                        mesh_file = mesh_dir_path / mesh.attrib["file"]
                        if not mesh_file.exists():
                            logger.error(f"Mesh file {mesh_file} does not exist.")
                            raise FileNotFoundError(f"Mesh file {mesh_file} does not exist.")
                        
                        # 转换为STL格式
                        stl_file = mesh_file.with_suffix('.stl')
                        if not stl_file.exists():
                            try:
                                import pymeshlab
                                ms = pymeshlab.MeshSet()
                                ms.load_new_mesh(str(mesh_file))
                                ms.save_current_mesh(str(stl_file))
                                logger.info(f"Converted {mesh_file} to {stl_file}")
                            except Exception as e:
                                logger.error(f"Error converting {mesh_file} to STL: {e}")
                        # 更新geom的mesh属性
                        geom.attrib["mesh"] = Path(mesh_name).with_suffix('.stl').name
                        logger.info(f"Updated geom {geom.attrib.get('name')} to use mesh {stl_file.name}")
                        # 更新mesh的file属性
                        mesh.attrib["name"] = Path(mesh_name).with_suffix('.stl').name
                        mesh.attrib["file"] = str(stl_file.relative_to(mesh_dir_path))
                        logger.info(f"Updated mesh {mesh.attrib.get('name')} to file {mesh.attrib.get('file')}")
                        break
    
    save_xml(mjcf_path, tree)

def remove_unused_mesh(mjcf_path: str | Path) -> None:
    """Remove the unused mesh of the MJCF file.

    Args:
        mjcf_path: The path to the MJCF file to process.
    """
    mjcf_path = Path(mjcf_path)
    tree = ET.parse(mjcf_path)
    root = tree.getroot()
    
    # 获取meshdir路径
    compiler = root.find("compiler")
    if compiler is None:
        logger.warning("No compiler element found in MJCF file")
        return
    
    meshdir = compiler.attrib.get("meshdir", ".")
    mesh_dir_path = mjcf_path.parent / meshdir
    
    # 查找所有使用的mesh和material
    used_meshes = set()
    used_materials = set()
    
    # 1. 遍历所有geom元素，收集使用的mesh
    for geom in root.iter("geom"):
        mesh_name = geom.attrib.get("mesh")
        if mesh_name:
            used_meshes.add(mesh_name)
        
        material_name = geom.attrib.get("material")
        if material_name:
            used_materials.add(material_name)
    
    # 2. 遍历所有site元素，收集使用的material
    for site in root.iter("site"):
        material_name = site.attrib.get("material")
        if material_name:
            used_materials.add(material_name)
    
    # 3. 遍历所有body元素，收集使用的material
    for body in root.iter("body"):
        material_name = body.attrib.get("material")
        if material_name:
            used_materials.add(material_name)
    
    logger.info(f"使用中的mesh: {used_meshes}")
    logger.info(f"使用中的material: {used_materials}")
    
    # 获取asset元素
    asset = root.find("asset")
    if asset is None:
        logger.warning("No asset element found in MJCF file")
        return
    
    # 收集要删除的mesh和对应文件
    meshes_to_remove = []
    mesh_files_to_remove = []
    
    for mesh in asset.findall("mesh"):
        mesh_name = mesh.attrib.get("name")
        mesh_file = mesh.attrib.get("file")
        
        if mesh_name not in used_meshes:
            logger.info(f"发现未使用的mesh: {mesh_name} (文件: {mesh_file})")
            meshes_to_remove.append(mesh)
            
            if mesh_file:
                # 添加mesh文件到删除列表
                full_mesh_path = mesh_dir_path / mesh_file
                if full_mesh_path.exists():
                    mesh_files_to_remove.append(full_mesh_path)
                
                # 如果是obj文件，检查对应的mtl文件
                if mesh_file.lower().endswith('.obj'):
                    mtl_file = full_mesh_path.with_suffix('.mtl')
                    if mtl_file.exists():
                        mesh_files_to_remove.append(mtl_file)
    
    # 收集要删除的material
    materials_to_remove = []
    
    for material in asset.findall("material"):
        material_name = material.attrib.get("name")
        
        if material_name not in used_materials:
            logger.info(f"发现未使用的material: {material_name}")
            materials_to_remove.append(material)
    
    # 删除未使用的mesh元素
    for mesh in meshes_to_remove:
        asset.remove(mesh)
        logger.info(f"已删除mesh元素: {mesh.attrib.get('name')}")
    
    # 删除未使用的material元素
    for material in materials_to_remove:
        asset.remove(material)
        logger.info(f"已删除material元素: {material.attrib.get('name')}")
    
    # 检查mesh目录中的所有mesh文件，删除未被引用的文件
    if mesh_dir_path.exists():
        # 收集所有被引用的文件名
        referenced_files = set()
        for mesh in asset.findall("mesh"):
            mesh_file = mesh.attrib.get("file")
            if mesh_file:
                referenced_files.add(mesh_file)
                # 如果是obj文件，也包含对应的mtl文件
                # if mesh_file.lower().endswith('.obj'):
                #     mtl_file = Path(mesh_file).with_suffix('.mtl').name
                #     referenced_files.add(str(Path(mesh_file).parent / mtl_file))
        
        # 查找所有mesh文件
        mesh_extensions = ['.obj', '.stl', '.dae', '.mtl']
        for mesh_file_path in mesh_dir_path.rglob('*'):
            if mesh_file_path.is_file() and mesh_file_path.suffix.lower() in mesh_extensions:
                # 计算相对于mesh目录的路径
                relative_path = mesh_file_path.relative_to(mesh_dir_path)
                relative_path_str = str(relative_path).replace('\\', '/')  # 统一使用正斜杠
                
                if relative_path_str not in referenced_files:
                    logger.info(f"发现未被引用的文件: {relative_path_str}")
                    mesh_files_to_remove.append(mesh_file_path)

    # 删除文件
    deleted_files = []
    for file_path in mesh_files_to_remove:
        try:
            if file_path.exists():
                file_path.unlink()
                deleted_files.append(str(file_path))
                logger.info(f"已删除文件: {file_path}")
        except Exception as e:
            logger.error(f"删除文件 {file_path} 时出错: {e}")
    
    # 保存修改后的MJCF文件
    if meshes_to_remove or materials_to_remove:
        save_xml(mjcf_path, tree)
        logger.info(f"已更新MJCF文件: {mjcf_path}")
    
    # 总结
    logger.info(f"清理完成:")
    logger.info(f"  删除的mesh元素: {len(meshes_to_remove)}")
    logger.info(f"  删除的material元素: {len(materials_to_remove)}")
    logger.info(f"  删除的文件: {len(deleted_files)}")
    
    if deleted_files:
        logger.info("删除的文件列表:")
        for file_path in deleted_files:
            logger.info(f"  - {file_path}")

def update_mesh(mjcf_path: str | Path, max_vertices: int = 1000000) -> None:
    """Update the mesh of the MJCF file.

    Args:
        mjcf_path: The path to the MJCF file to process.
    """
    tree = ET.parse(mjcf_path)
    root = tree.getroot()
    update_mesh_assets(mjcf_path, root, max_vertices)
    collision_to_stl(mjcf_path)
    remove_unused_mesh(mjcf_path)

def main() -> None:
    parser = argparse.ArgumentParser(description="Updates the mesh of the MJCF file.")
    parser.add_argument("mjcf_path", type=Path, help="Path to the MJCF file.")
    parser.add_argument("--max-vertices", type=int, default=200000, help="Maximum number of vertices in the mesh.")
    args = parser.parse_args()
    update_mesh(args.mjcf_path, args.max_vertices)

if __name__ == "__main__":
    main()