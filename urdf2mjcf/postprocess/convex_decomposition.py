import argparse
import logging
import multiprocessing as mp
import os
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Optional, Tuple, List

import coacd
import trimesh
from urdf2mjcf.utils import save_xml

logger = logging.getLogger(__name__)


def process_single_mesh(mesh_info: Tuple[str, str, Path, Path, list]) -> Optional[Tuple[str, List[Tuple[str, str]]]]:
    """处理单个mesh的凸分解。
    
    Args:
        mesh_info: (mesh_name, mesh_file_path, dir_path, urdf_dir, workspace_search_paths)的元组
        
    Returns:
        如果成功处理，返回(mesh_name, [(part_name, part_file), ...])
        如果处理失败或无需处理，返回None
    """
    mesh_name, mesh_file_relative, dir_path, urdf_dir, workspace_search_paths = mesh_info
    
    # 正确解析mesh文件路径
    mesh_file = None
    if mesh_file_relative.startswith('package://'):
        # Handle package:// paths
        package_path = mesh_file_relative[len('package://'):]
        pkg_name = package_path.split('/')[0]
        sub_path = '/'.join(package_path.split('/')[1:])
        
        try:
            from urdf2mjcf.package_resolver import resolve_package_path
            pkg_root = resolve_package_path(pkg_name, workspace_search_paths)
            if pkg_root:
                mesh_file = pkg_root / sub_path
        except (ImportError, Exception) as e:
            logger.warning(f"Could not resolve package path {mesh_file_relative}: {e}")
    else:
        # Handle regular paths (relative to URDF file or absolute)
        if Path(mesh_file_relative).is_absolute():
            mesh_file = Path(mesh_file_relative)
        else:
            mesh_file = urdf_dir / mesh_file_relative
    
    if mesh_file is None or not mesh_file.exists():
        logger.warning(f"Mesh file {mesh_file_relative} does not exist at {mesh_file}.")
        return None
    
    logger.info(f"Processing mesh {mesh_name} for convex decomposition")
    
    # 加载 mesh
    try:
        mesh_data = trimesh.load(mesh_file, force="mesh")
    except Exception as e:
        logger.error(f"Failed to load mesh {mesh_file}: {e}")
        return None
    
    # 执行凸分解
    try:
        mesh_coacd = coacd.Mesh(mesh_data.vertices, mesh_data.faces)
        parts = coacd.run_coacd(mesh_coacd)
        logger.info(f"Mesh {mesh_name} decomposed into {len(parts)} parts")
        
        # 如果只有一个part，说明本身就是凸的，无需处理
        if len(parts) <= 1:
            logger.info(f"Mesh {mesh_name} is already convex (only {len(parts)} part), skipping")
            return None
            
    except Exception as e:
        logger.error(f"Failed to decompose mesh {mesh_name}: {e}")
        return None
    
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
            original_dir = str(Path(mesh_file_relative).parent)  # 例如: meshes/leg
            relative_part_path = f"{original_dir}/{mesh_stem}_parts/{part_filename}"
            part_name = f"{mesh_stem}_part{i+1}"
            part_info.append((part_name, relative_part_path))
            
            logger.info(f"Saved part {i+1} to {relative_part_path}")
        except Exception as e:
            logger.error(f"Failed to save part {i+1} of mesh {mesh_name}: {e}")
    
    if part_info:
        return (mesh_name, part_info)
    else:
        return None


def convex_decomposition_assets(mjcf_path: str | Path, root: ET.Element, max_processes: Optional[int] = None, urdf_dir: Optional[Path] = None) -> None:
    """对 MJCF 文件中 collision 类型的 geom 进行凸分解。

    Args:
        mjcf_path: MJCF 文件的路径
        root: MJCF 文件的根元素
        max_processes: 最大进程数，如果为None则自动计算为CPU核心数-4
        urdf_dir: URDF文件所在目录，如果为None则假设与MJCF文件在同一目录
    """
    mjcf_path = Path(mjcf_path)
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    compiler.attrib["meshdir"] = "."

    dir_path = mjcf_path.parent / compiler.attrib["meshdir"]
    
    # 获取URDF文件的目录
    if urdf_dir is None:
        urdf_dir = mjcf_path.parent  # 默认假设与MJCF文件在同一目录
    
    # 自动检测工作空间搜索路径
    from urdf2mjcf.package_resolver import find_workspace_from_path
    workspace_search_paths = []
    
    # 从MJCF文件位置查找工作空间
    workspace_from_mjcf = find_workspace_from_path(mjcf_path.parent)
    if workspace_from_mjcf:
        workspace_search_paths.append(workspace_from_mjcf)
    
    # 检查当前工作目录
    workspace_from_cwd = find_workspace_from_path(Path.cwd())
    if workspace_from_cwd and workspace_from_cwd not in workspace_search_paths:
        workspace_search_paths.append(workspace_from_cwd)
    
    # 添加一些手动路径作为备选
    cwd = Path.cwd()
    for i in range(3):  # 检查最多3层
        if cwd not in workspace_search_paths:
            workspace_search_paths.append(cwd)
        if cwd.parent == cwd:  # 已到达文件系统根目录
            break
        cwd = cwd.parent

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
        mesh_info_list.append((mesh_name, mesh_assets[mesh_name], dir_path, urdf_dir, workspace_search_paths))
    
    # 获取CPU核心数并设置进程数
    cpu_count = os.cpu_count() or 4  # 如果获取失败，默认使用4核
    if max_processes is None:
        max_processes = max(1, cpu_count - 4)  # 至少使用1个进程
    else:
        max_processes = max(1, max_processes)  # 确保至少使用1个进程
    actual_processes = min(max_processes, len(mesh_info_list))  # 不超过任务数量
    
    logger.info(f"Detected {cpu_count} CPU cores, using {actual_processes} processes for convex decomposition")
    
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
    
    logger.info(f"Successfully processed {len(new_mesh_parts)} meshes with convex decomposition")
    
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


def convex_decomposition(mjcf_path: str | Path, max_processes: Optional[int] = None, urdf_dir: Optional[Path] = None) -> None:
    """对 MJCF 文件进行凸分解处理。

    Args:
        mjcf_path: MJCF 文件的路径
        max_processes: 最大进程数，如果为None则自动计算为CPU核心数-4
        urdf_dir: URDF文件所在目录，如果为None则假设与MJCF文件在同一目录
    """
    tree = ET.parse(mjcf_path)
    root = tree.getroot()
    convex_decomposition_assets(mjcf_path, root, max_processes, urdf_dir)

    save_xml(mjcf_path, tree)


def main() -> None:
    parser = argparse.ArgumentParser(description="对 MJCF 文件中的 collision mesh 进行凸分解")
    parser.add_argument("mjcf_path", type=Path, help="MJCF 文件的路径")
    parser.add_argument("--processes", type=int, help="指定使用的进程数量，默认为CPU核心数-4")
    args = parser.parse_args()
    
    logging.basicConfig(level=logging.INFO)
    convex_decomposition(args.mjcf_path, args.processes)


if __name__ == "__main__":
    main()