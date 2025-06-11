"""检查mesh文件中的点是否全部共面，如果是则添加inertia="shell"属性。"""

import logging
import numpy as np
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Optional

import trimesh
from urdf2mjcf.utils import save_xml

logger = logging.getLogger(__name__)


def read_mesh_vertices(file_path: Path) -> Optional[np.ndarray]:
    """
    使用trimesh读取mesh文件的顶点。
    
    Args:
        file_path: mesh文件路径
        
    Returns:
        顶点数组 (N, 3) 或 None（如果读取失败）
    """
    if not file_path.exists():
        logger.warning(f"Mesh文件不存在: {file_path}")
        return None
    
    try:
        # 使用trimesh加载mesh文件
        mesh_data = trimesh.load(file_path, force="mesh")
        
        if mesh_data is None:
            logger.warning(f"无法加载mesh文件: {file_path}")
            return None
        
        # 获取顶点
        vertices = mesh_data.vertices
        
        if vertices is None or len(vertices) == 0:
            logger.warning(f"Mesh文件 {file_path} 中没有顶点")
            return None
        
        logger.debug(f"成功读取mesh文件 {file_path}，共 {len(vertices)} 个顶点")
        return vertices
        
    except Exception as e:
        logger.warning(f"读取mesh文件 {file_path} 失败: {e}")
        return None


def check_coplanar(vertices: np.ndarray, tolerance: float = 1e-6) -> bool:
    """
    检查顶点是否全部共面。
    
    Args:
        vertices: 顶点数组 (N, 3)
        tolerance: 共面判断的容差
        
    Returns:
        True如果所有点共面，否则False
    """
    if len(vertices) < 4:
        # 少于4个点总是共面的
        logger.debug(f"顶点数量 {len(vertices)} < 4，认为是共面的")
        return True
    
    # 取前三个不共线的点来定义平面
    # 找到第一个点作为参考
    p0 = vertices[0]
    
    # 找到与p0不重合的第二个点
    p1 = None
    for i in range(1, len(vertices)):
        if not np.allclose(vertices[i], p0, atol=tolerance):
            p1 = vertices[i]
            break
    
    if p1 is None:
        logger.debug("所有顶点都重合，认为是共面的")
        return True
    
    # 找到不与p0-p1共线的第三个点
    p2 = None
    v1 = p1 - p0
    for i in range(len(vertices)):
        if np.allclose(vertices[i], p0, atol=tolerance) or np.allclose(vertices[i], p1, atol=tolerance):
            continue
        
        v2 = vertices[i] - p0
        # 检查是否共线（叉积接近零）
        cross = np.cross(v1, v2)
        if np.linalg.norm(cross) > tolerance:
            p2 = vertices[i]
            break
    
    if p2 is None:
        logger.debug("所有顶点都共线，认为是共面的")
        return True
    
    # 现在有了三个不共线的点，计算平面法向量
    v1 = p1 - p0
    v2 = p2 - p0
    normal = np.cross(v1, v2)
    normal = normal / np.linalg.norm(normal)  # 单位化
    
    # 计算平面方程中的d值: ax + by + cz + d = 0
    d = -np.dot(normal, p0)
    
    # 检查所有其他点是否在这个平面上
    for vertex in vertices:
        distance = abs(np.dot(normal, vertex) + d)
        if distance > tolerance:
            logger.debug(f"发现非共面点，距离平面 {distance:.2e} > 容差 {tolerance:.2e}")
            return False
    
    logger.debug(f"所有 {len(vertices)} 个顶点都在同一平面上")
    return True


def check_shell_meshes(mjcf_path: Path) -> None:
    """
    检查MJCF文件中的所有mesh，如果顶点全部共面则添加inertia="shell"属性。
    
    Args:
        mjcf_path: MJCF文件路径
    """
    logger.info(f"检查 {mjcf_path} 中的mesh是否为shell...")
    
    try:
        tree = ET.parse(mjcf_path)
        root = tree.getroot()
        
        # 找到assets元素
        asset_elem = root.find('asset')
        if asset_elem is None:
            logger.info("未找到asset元素，跳过shell检查")
            return
        
        # 获取mjcf文件所在目录，用于解析相对路径
        mjcf_dir = mjcf_path.parent
        
        # 检查compiler设置的meshdir
        compiler_elem = root.find('compiler')
        mesh_dir = mjcf_dir
        if compiler_elem is not None:
            meshdir = compiler_elem.get('meshdir', '.')
            mesh_dir = mjcf_dir / meshdir
        
        modified = False
        
        # 检查所有mesh元素
        for mesh_elem in asset_elem.findall('mesh'):
            mesh_name = mesh_elem.get('name', '')
            mesh_file = mesh_elem.get('file', '')
            
            if not mesh_file:
                logger.debug(f"Mesh {mesh_name} 没有file属性，跳过")
                continue
            
            # 检查是否已经有inertia属性
            if mesh_elem.get('inertia') is not None:
                logger.debug(f"Mesh {mesh_name} 已经有inertia属性，跳过")
                continue
            
            # 解析mesh文件路径
            mesh_path = mesh_dir / mesh_file
            
            # 读取顶点
            vertices = read_mesh_vertices(mesh_path)
            if vertices is None:
                logger.debug(f"无法读取mesh文件 {mesh_file}，跳过")
                continue
            
            # 检查是否共面
            if check_coplanar(vertices):
                logger.info(f"Mesh {mesh_name} ({mesh_file}) 的顶点全部共面，添加 inertia='shell'")
                mesh_elem.set('inertia', 'shell')
                modified = True
            else:
                logger.debug(f"Mesh {mesh_name} ({mesh_file}) 的顶点不全共面，保持默认inertia")
        
        # 如果有修改，保存文件
        if modified:
            save_xml(mjcf_path, tree)
            logger.info(f"已更新 {mjcf_path} 中的shell mesh属性")
        else:
            logger.info("没有发现需要设置为shell的mesh")
            
    except Exception as e:
        logger.error(f"检查shell mesh时出错: {e}")


def main() -> None:
    """命令行入口函数。"""
    import argparse
    
    parser = argparse.ArgumentParser(description="检查MJCF文件中的mesh是否为shell")
    parser.add_argument("mjcf_path", type=str, help="MJCF文件路径")
    parser.add_argument("--log-level", type=int, default=logging.INFO, help="日志级别")
    
    args = parser.parse_args()
    
    logging.basicConfig(level=args.log_level)
    
    mjcf_path = Path(args.mjcf_path)
    check_shell_meshes(mjcf_path)


if __name__ == "__main__":
    main()
