"""Defines a post-processing function that updates the mesh of the Mujoco model.

This script updates the mesh of the MJCF file.
"""

import argparse
import logging
import xml.etree.ElementTree as ET
from pathlib import Path

import trimesh
from urdf2mjcf.utils import save_xml

logger = logging.getLogger(__name__)
MAX_VERTICES = 200000

def update_mesh_assets(mjcf_path: str | Path, root: ET.Element) -> None:
    """Update the mesh assets of the MJCF file.

    Args:
        root: The root element of the MJCF file.
    """
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
        
        mesh_data = trimesh.load(mesh_file, force="mesh")

        vertices = mesh_data.vertices
        if len(vertices) >= MAX_VERTICES:
            logger.info(f"Simplifying mesh {mesh.attrib['file']} with {len(vertices)} vertices")
            # 简化网格
            target_face_ratio = 1. - float(min(MAX_VERTICES, len(vertices)* 0.5) / len(vertices))
            simplified_mesh = mesh_data.simplify_quadric_decimation(target_face_ratio)
            mesh_data = simplified_mesh
            logger.info(f"Simplified mesh to {len(mesh_data.faces)} faces")
            mesh_data.export(mesh_file)

def update_mesh(mjcf_path: str | Path) -> None:
    """Update the mesh of the MJCF file.

    Args:
        mjcf_path: The path to the MJCF file to process.
    """
    tree = ET.parse(mjcf_path)
    root = tree.getroot()
    update_mesh_assets(mjcf_path, root)

    save_xml(mjcf_path, tree)


def main() -> None:
    parser = argparse.ArgumentParser(description="Updates the mesh of the MJCF file.")
    parser.add_argument("mjcf_path", type=Path, help="Path to the MJCF file.")
    args = parser.parse_args()
    update_mesh(args.mjcf_path)


if __name__ == "__main__":
    main()