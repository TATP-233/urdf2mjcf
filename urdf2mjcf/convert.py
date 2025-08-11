"""Converts URDF files to MJCF files."""

import os
import json
import shutil
import logging
import argparse
import traceback
from pathlib import Path
import xml.etree.ElementTree as ET

from urdf2mjcf.model import DefaultJointMetadata, ActuatorMetadata, ConversionMetadata
from urdf2mjcf.postprocess.add_appendix import add_appendix
from urdf2mjcf.postprocess.add_backlash import add_backlash
from urdf2mjcf.postprocess.add_floor import add_floor
from urdf2mjcf.postprocess.add_light import add_light
from urdf2mjcf.postprocess.update_mesh import update_mesh
from urdf2mjcf.postprocess.convex_decomposition import convex_decomposition
from urdf2mjcf.postprocess.split_obj_materials import split_obj_by_materials
from urdf2mjcf.postprocess.base_joint import fix_base_joint
from urdf2mjcf.postprocess.collisions import update_collisions
from urdf2mjcf.postprocess.explicit_floor_contacts import add_explicit_floor_contacts
from urdf2mjcf.postprocess.make_degrees import make_degrees
from urdf2mjcf.postprocess.remove_redundancies import remove_redundancies
from urdf2mjcf.postprocess.check_shell import check_shell_meshes
from urdf2mjcf.utils import save_xml

from urdf2mjcf.geometry import (
    ParsedJointParams, 
    GeomElement, 
    compute_min_z, 
    rpy_to_quat
)
from urdf2mjcf.mjcf_builders import (
    add_compiler,
    add_default,
    add_contact,
    add_weld_constraints,
    add_size,
    add_option,
    add_visual,
    add_assets,
    ROBOT_CLASS
)
from urdf2mjcf.package_resolver import resolve_package_path, find_workspace_from_path
from urdf2mjcf.materials import Material, parse_mtl_name, get_obj_material_info, copy_obj_with_mtl

logger = logging.getLogger(__name__)

def _get_empty_actuator_metadata(
    robot_elem: ET.Element,
) -> dict[str, ActuatorMetadata]:
    """Create placeholder metadata for joints and actuators if none are provided.

    Each joint is simply assigned a "motor" actuator type, which has no other parameters.
    """
    actuator_meta: dict[str, ActuatorMetadata] = {}
    for joint in robot_elem.findall("joint"):
        name = joint.attrib.get("name")
        if not name:
            continue
        actuator_meta[name] = ActuatorMetadata(
            actuator_type="motor",
        )

    return actuator_meta

def convert_urdf_to_mjcf(
    urdf_path: str | Path,
    mjcf_path: str | Path | None = None,
    metadata_file: str | Path | None = None,
    *,
    default_metadata: DefaultJointMetadata | None = None,
    actuator_metadata: dict[str, ActuatorMetadata] | None = None,
    appendix_files: list[Path] | None = None,
    max_vertices: int = 1000000,
    collision_only: bool = False,
    convex_decompose: bool = True
) -> None:
    """Converts a URDF file to an MJCF file.

    Args:
        urdf_path: The path to the URDF file.
        mjcf_path: The desired output MJCF file path.
        metadata_file: Optional path to metadata file.
        default_metadata: Optional default metadata.
        actuator_metadata: Optional actuator metadata.
        appendix_files: Optional list of appendix files.
        max_vertices: Maximum number of vertices in the mesh.
        collision_only: If true, use simplified collision geometry without visual appearance for visual representation.
        convex_decompose: If true, run convex decomposition on the mesh.
    """
    urdf_path = Path(urdf_path)
    mjcf_path = Path(mjcf_path) if mjcf_path is not None else urdf_path.with_suffix(".mjcf")
    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF file not found: {urdf_path}")
    mjcf_path.parent.mkdir(parents=True, exist_ok=True)

    urdf_tree = ET.parse(urdf_path)
    robot = urdf_tree.getroot()
    if robot is None:
        raise ValueError("URDF file has no root element")

    if metadata_file is not None:
        try:
            with open(metadata_file, "r") as f:
                metadata = ConversionMetadata.model_validate_json(f.read())
        except Exception as e:
            logger.warning("Failed to load metadata from %s: %s", metadata_file, e)
            metadata = ConversionMetadata()
    else:
        metadata = ConversionMetadata()

    if actuator_metadata is None:
        missing = []
        if actuator_metadata is None:
            missing.append("joint")
        logger.warning("Missing %s metadata, falling back to single empty 'motor' class.", " and ".join(missing))
        actuator_metadata = _get_empty_actuator_metadata(robot)
    assert actuator_metadata is not None

    # Parse materials from URDF - both from root level and from link visuals
    materials: dict[str, str] = {}

    # Get materials defined at the robot root level
    for material in robot.findall("material"):
        name = material.attrib.get("name")
        if name is None:
            continue
        elif name == "":
            logger.warning("Material name is empty, using default_material")
            material.attrib["name"] = "default_material"

        color = material.find("color")
        if color is not None:
            rgba = color.attrib.get("rgba")
            if rgba is not None:
                materials[name] = rgba

    if not collision_only:
        # Get materials defined in link visual elements
        for link in robot.findall("link"):
            for visual in link.findall("visual"):
                visual_material = visual.find("material")
                if visual_material is None:
                    continue
                elif visual_material.attrib.get("name") == "":
                    visual_material.attrib["name"] = "default_material"

                name = visual_material.attrib.get("name")
                if name is None:
                    continue
                color = visual_material.find("color")
                if color is not None:
                    rgba = color.attrib.get("rgba")
                    if rgba is not None:
                        materials[name] = rgba

    # Create a new MJCF tree root element.
    mjcf_root: ET.Element = ET.Element("mujoco", attrib={"model": robot.attrib.get("name", "converted_robot")})

    # Add compiler, option, visual, and assets
    add_compiler(mjcf_root)
    add_size(mjcf_root)
    add_option(mjcf_root)
    add_visual(mjcf_root)
    add_default(mjcf_root, metadata, default_metadata, collision_only)

    # Creates the worldbody element.
    worldbody = ET.SubElement(mjcf_root, "worldbody")

    # Build mappings for URDF links and joints.
    link_map: dict[str, ET.Element] = {link.attrib["name"]: link for link in robot.findall("link")}
    parent_map: dict[str, list[tuple[str, ET.Element]]] = {}
    child_joints: dict[str, ET.Element] = {}
    for joint in robot.findall("joint"):
        parent_elem = joint.find("parent")
        child_elem = joint.find("child")
        if parent_elem is None or child_elem is None:
            logger.warning("Joint missing parent or child element")
            continue
        parent_name = parent_elem.attrib.get("link", "")
        child_name = child_elem.attrib.get("link", "")
        if not parent_name or not child_name:
            logger.warning("Joint missing parent or child link name")
            continue
        parent_map.setdefault(parent_name, []).append((child_name, joint))
        child_joints[child_name] = joint

    all_links = set(link_map.keys())
    child_links = set(child_joints.keys())
    root_links: list[str] = list(all_links - child_links)
    if not root_links:
        raise ValueError("No root link found in URDF.")
    root_link_name: str = root_links[0]

    # These dictionaries are used to collect mesh assets and actuator joints.
    mesh_assets: dict[str, str] = {}
    actuator_joints: list[ParsedJointParams] = []
    mimic_constraints: list[tuple[str, str, float, float]] = []  # (mimicked_joint, mimicking_joint, multiplier, offset)
    
    # Parse mimic joints from URDF
    for joint in robot.findall("joint"):
        mimic_elem = joint.find("mimic")
        if mimic_elem is not None:
            joint_name = joint.attrib.get("name")
            mimicked_joint = mimic_elem.attrib.get("joint")
            multiplier = float(mimic_elem.attrib.get("multiplier", "1.0"))
            offset = float(mimic_elem.attrib.get("offset", "0.0"))
            
            if joint_name and mimicked_joint:
                mimic_constraints.append((mimicked_joint, joint_name, multiplier, offset))
                logger.info(f"Found mimic constraint: {joint_name} mimics {mimicked_joint} with multiplier={multiplier}, offset={offset}")
    
    # Prepare paths for mesh processing
    urdf_dir: Path = urdf_path.parent.resolve()
    target_mesh_dir: Path = (mjcf_path.parent).resolve()
    target_mesh_dir.mkdir(parents=True, exist_ok=True)
    
    # Auto-detect workspace search paths for package resolution
    workspace_search_paths = []
    
    # Find workspace from URDF file location
    workspace_from_urdf = find_workspace_from_path(urdf_path)
    if workspace_from_urdf:
        workspace_search_paths.append(workspace_from_urdf)
        logger.debug(f"Found ROS workspace from URDF location: {workspace_from_urdf}")
    
    # Also try to find the package root of the URDF file itself for local resource resolution
    from urdf2mjcf.package_resolver import _default_resolver
    package_root = _default_resolver._find_package_root_from_urdf_path(urdf_path)
    if package_root and package_root not in workspace_search_paths:
        workspace_search_paths.append(package_root)
        logger.debug(f"Found package root from URDF location: {package_root}")

    def handle_geom_element(geom_elem: ET.Element | None, default_size: str) -> GeomElement:
        """Helper to handle geometry elements safely.

        Args:
            geom_elem: The geometry element to process
            default_size: Default size to use if not specified

        Returns:
            A GeomElement instance
        """
        if geom_elem is None:
            return GeomElement(type="box", size=default_size, scale=None, mesh=None)

        box_elem = geom_elem.find("box")
        if box_elem is not None:
            size_str = box_elem.attrib.get("size", default_size)
            return GeomElement(
                type="box",
                size=" ".join(str(float(s) / 2) for s in size_str.split()),
            )

        cyl_elem = geom_elem.find("cylinder")
        if cyl_elem is not None:
            radius = cyl_elem.attrib.get("radius", "0.1")
            length = cyl_elem.attrib.get("length", "1")
            return GeomElement(
                type="cylinder",
                size=f"{radius} {float(length) / 2}",
            )

        sph_elem = geom_elem.find("sphere")
        if sph_elem is not None:
            radius = sph_elem.attrib.get("radius", "0.1")
            return GeomElement(
                type="sphere",
                size=radius,
            )

        mesh_elem = geom_elem.find("mesh")
        if mesh_elem is not None:
            filename = mesh_elem.attrib.get("filename")
            if filename is not None:
                mesh_name = Path(filename).name
                if mesh_name not in mesh_assets:
                    mesh_assets[mesh_name] = filename
                        
                scale = mesh_elem.attrib.get("scale")
                return GeomElement(
                    type="mesh",
                    size=None,
                    scale=scale,
                    mesh=mesh_name,
                )

        return GeomElement(
            type="box",
            size=default_size,
        )

    def build_body(
        link_name: str,
        joint: ET.Element | None = None,
        actuator_joints: list[ParsedJointParams] = actuator_joints,
    ) -> ET.Element | None:
        """Recursively build a MJCF body element from a URDF link."""
        link: ET.Element = link_map[link_name]

        if joint is not None:
            origin_elem: ET.Element | None = joint.find("origin")
            if origin_elem is not None:
                pos = origin_elem.attrib.get("xyz", "0 0 0")
                rpy = origin_elem.attrib.get("rpy", "0 0 0")
                quat = rpy_to_quat(rpy)
            else:
                pos = "0 0 0"
                quat = "1 0 0 0"
        else:
            pos = "0 0 0"
            quat = "1 0 0 0"

        body: ET.Element = ET.Element("body", attrib={"name": link_name, "pos": pos, "quat": quat})

        # Add joint element if this is not the root and the joint type is not fixed.
        if joint is not None:
            jtype: str = joint.attrib.get("type", "fixed")

            if jtype in ("revolute", "continuous", "prismatic"):
                j_name: str = joint.attrib.get("name", link_name + "_joint")
                j_attrib: dict[str, str] = {"name": j_name}

                if jtype in ["revolute", "continuous"]:
                    j_attrib["type"] = "hinge"
                elif jtype == "prismatic":
                    j_attrib["type"] = "slide"
                else:
                    raise ValueError(f"Unsupported joint type: {jtype}")

                if j_name in actuator_metadata:
                    if actuator_metadata[j_name].joint_class is not None:
                        joint_class_value = actuator_metadata[j_name].joint_class
                        j_attrib["class"] = str(joint_class_value)
                        logger.info("Joint %s assigned to class: %s", j_name, joint_class_value)

                limit = joint.find("limit")
                if limit is not None:
                    lower_val = limit.attrib.get("lower")
                    upper_val = limit.attrib.get("upper")
                    if lower_val is not None and upper_val is not None:
                        j_attrib["range"] = f"{lower_val} {upper_val}"
                        lower_num: float | None = float(lower_val)
                        upper_num: float | None = float(upper_val)
                    else:
                        lower_num = upper_num = None
                else:
                    lower_num = upper_num = None
                axis_elem = joint.find("axis")
                if axis_elem is not None:
                    j_attrib["axis"] = axis_elem.attrib.get("xyz", "0 0 1")
                ET.SubElement(body, "joint", attrib=j_attrib)

                actuator_joints.append(
                    ParsedJointParams(
                        name=j_name,
                        type=j_attrib["type"],
                        lower=lower_num,
                        upper=upper_num,
                    )
                )

        # Process inertial information.
        inertial = link.find("inertial")
        if inertial is not None:
            inertial_elem = ET.Element("inertial")
            origin_inertial = inertial.find("origin")
            if origin_inertial is not None:
                inertial_elem.attrib["pos"] = origin_inertial.attrib.get("xyz", "0 0 0")
                rpy = origin_inertial.attrib.get("rpy", "0 0 0")
                if rpy != "0 0 0":
                    inertial_elem.attrib["quat"] = rpy_to_quat(rpy)
            mass_elem = inertial.find("mass")
            if mass_elem is not None:
                mass = mass_elem.attrib.get("value", "0")
                inertial_elem.attrib["mass"] = str(max(float(mass), 1e-6))
            inertia_elem = inertial.find("inertia")
            if inertia_elem is not None:
                ixx = float(inertia_elem.attrib.get("ixx", "0"))
                ixy = float(inertia_elem.attrib.get("ixy", "0"))
                ixz = float(inertia_elem.attrib.get("ixz", "0"))
                iyy = float(inertia_elem.attrib.get("iyy", "0"))
                iyz = float(inertia_elem.attrib.get("iyz", "0"))
                izz = float(inertia_elem.attrib.get("izz", "0"))
                if abs(ixy) > 1e-6 or abs(ixz) > 1e-6 or abs(iyz) > 1e-6:
                    logger.info(
                        "Warning: off-diagonal inertia terms for link '%s' are nonzero and will be ignored.",
                        link_name,
                    )
                inertial_elem.attrib["diaginertia"] = f"{max(ixx, 1e-9)} {max(iyy, 1e-9)} {max(izz, 1e-9)}"
            body.append(inertial_elem)

        # Process collision geometries.
        collisions = link.findall("collision")
        for idx, collision in enumerate(collisions):
            origin_collision = collision.find("origin")
            if origin_collision is not None:
                pos_geom: str = origin_collision.attrib.get("xyz", "0 0 0")
                rpy_geom: str = origin_collision.attrib.get("rpy", "0 0 0")
                quat_geom: str = rpy_to_quat(rpy_geom)
            else:
                pos_geom = "0 0 0"
                quat_geom = "1 0 0 0"
            name = f"{link_name}_collision"
            if len(collisions) > 1:
                name = f"{name}_{idx}"
            collision_geom_attrib: dict[str, str] = {"name": name, "pos": pos_geom, "quat": quat_geom}

            # Get material from collision element
            collision_geom_elem: ET.Element | None = collision.find("geometry")
            if collision_geom_elem is not None:
                geom = handle_geom_element(collision_geom_elem, "1 1 1")
                collision_geom_attrib["type"] = geom.type
                if geom.type == "mesh":
                    if geom.mesh is not None:
                        collision_geom_attrib["mesh"] = geom.mesh
                elif geom.size is not None:
                    collision_geom_attrib["size"] = geom.size
                if geom.scale is not None:
                    collision_geom_attrib["scale"] = geom.scale
            collision_geom_attrib["class"] = "collision"
            ET.SubElement(body, "geom", attrib=collision_geom_attrib)

        # Process visual geometries.
        if not collision_only:
            visuals = link.findall("visual")
            for idx, visual in enumerate(visuals):
                origin_elem = visual.find("origin")
                if origin_elem is not None:
                    pos_geom = origin_elem.attrib.get("xyz", "0 0 0")
                    rpy_geom = origin_elem.attrib.get("rpy", "0 0 0")
                    quat_geom = rpy_to_quat(rpy_geom)
                else:
                    pos_geom = "0 0 0"
                    quat_geom = "1 0 0 0"
                
                visual_geom_elem: ET.Element | None = visual.find("geometry")
                if visual_geom_elem is not None:
                    geom = handle_geom_element(visual_geom_elem, "1 1 1")
                    
                    # Standard single geom creation
                    name = f"{link_name}_visual"
                    if len(visuals) > 1:
                        name = f"{name}_{idx}"
                    visual_geom_attrib: dict[str, str] = {"name": name, "pos": pos_geom, "quat": quat_geom}
                    
                    visual_geom_attrib["type"] = geom.type
                    if geom.type == "mesh" and geom.mesh is not None:
                        visual_geom_attrib["mesh"] = geom.mesh
                    elif geom.size is not None:
                        visual_geom_attrib["size"] = geom.size
                        
                    if geom.scale is not None:
                        visual_geom_attrib["scale"] = geom.scale
                else:
                    # No geometry element
                    name = f"{link_name}_visual"
                    if len(visuals) > 1:
                        name = f"{name}_{idx}"
                    visual_geom_attrib = {
                        "name": name,
                        "pos": pos_geom,
                        "quat": quat_geom,
                        "type": "box",
                        "size": "1 1 1"
                    }
                
                # Check URDF material first
                assigned_material = "default_material"
                material_elem = visual.find("material")
                if material_elem is not None:
                    material_name = material_elem.attrib.get("name")
                    if material_name and material_name in materials:
                        assigned_material = material_name
                
                # For mesh geoms, check if it's a single-material OBJ file
                if geom.type == "mesh" and geom.mesh is not None and assigned_material == "default_material":
                    # Try to find the actual OBJ file to check its materials
                    obj_filename = None
                    for mesh_name, filename in mesh_assets.items():
                        if mesh_name == geom.mesh:
                            obj_filename = filename
                            break
                    
                    if obj_filename and obj_filename.lower().endswith('.obj'):
                        # Determine the actual OBJ file path
                        obj_file_path = None
                        if 'package://' in obj_filename:
                            # Handle package:// paths
                            package_path = obj_filename[len('package://'):]
                            pkg_mesh_name = package_path.split('/')[0]
                            sub_path = '/'.join(package_path.split('/')[1:])
                            try:
                                pkg_root = resolve_package_path(pkg_mesh_name, workspace_search_paths)
                                if pkg_root:
                                    obj_file_path = pkg_root / sub_path
                            except:
                                obj_file_path = None
                        else:
                            # Regular path
                            if obj_filename.startswith('/'):
                                obj_file_path = Path(obj_filename)
                            else:
                                obj_file_path = urdf_dir / obj_filename
                        
                        if obj_file_path:
                            has_single_material, material_name = get_obj_material_info(obj_file_path)
                            if has_single_material and material_name:
                                # Create a material name that matches what split_obj_materials would create
                                obj_stem = obj_file_path.stem
                                single_material_name = f"{obj_stem}_{material_name}"
                                assigned_material = single_material_name
                                logger.info(f"Assigned single OBJ material {single_material_name} to geom {visual_geom_attrib['name']}")
                            
                visual_geom_attrib["material"] = assigned_material
                visual_geom_attrib["class"] = "visual"
                ET.SubElement(body, "geom", attrib=visual_geom_attrib)

        # Recurse into child links.
        if link_name in parent_map:
            for child_name, child_joint in parent_map[link_name]:
                child_body = build_body(child_name, child_joint, actuator_joints)
                if child_body is not None:
                    body.append(child_body)
        return body

    # Build the robot body hierarchy starting from the root link.
    robot_body = build_body(root_link_name, None, actuator_joints)
    if robot_body is None:
        raise ValueError("Failed to build robot body")

    # Gets the minimum z coordinate of the robot body.
    min_z: float = compute_min_z(robot_body)
    computed_offset: float = -min_z + metadata.height_offset
    logger.info("Auto-detected base offset: %s (min z = %s)", computed_offset, min_z)

    # Moves the robot body to the computed offset.
    body_pos = robot_body.attrib.get("pos", "0 0 0")
    body_pos = [float(x) for x in body_pos.split()]
    body_pos[2] += computed_offset
    robot_body.attrib["pos"] = " ".join(f"{x:.8f}" for x in body_pos)

    robot_body.attrib["childclass"] = ROBOT_CLASS
    worldbody.append(robot_body)

    # Add a site to the root link for sensors
    root_site_name = f"{root_link_name}_site"
    ET.SubElement(
        robot_body,
        "site",
        attrib={"name": root_site_name, "pos": "0 0 0", "quat": "1 0 0 0"},
    )

    # Collect materials from single-material OBJ files
    obj_materials = {}
    for mesh_name, filename in mesh_assets.items():
        if filename.lower().endswith('.obj'):
            # Determine the actual OBJ file path
            obj_file_path = None
            if 'package://' in filename:
                package_path = filename[len('package://'):]
                pkg_mesh_name = package_path.split('/')[0]
                sub_path = '/'.join(package_path.split('/')[1:])
                try:
                    pkg_root = resolve_package_path(pkg_mesh_name, workspace_search_paths)
                    if pkg_root:
                        obj_file_path = pkg_root / sub_path
                except:
                    obj_file_path = None
            else:
                if filename.startswith('/'):
                    obj_file_path = Path(filename)
                else:
                    obj_file_path = urdf_dir / filename
            
            if obj_file_path:
                has_single_material, material_name = get_obj_material_info(obj_file_path)
                if has_single_material and material_name:
                    # Parse the MTL file to get material properties
                    try:
                        mtl_name = parse_mtl_name(obj_file_path.open("r").readlines())
                        if mtl_name:
                            mtl_file = obj_file_path.parent / mtl_name
                            if mtl_file.exists():
                                with open(mtl_file, "r") as f:
                                    mtl_lines = f.readlines()
                                
                                # Find the material definition
                                material_lines = []
                                in_material = False
                                for line in mtl_lines:
                                    line = line.strip()
                                    if line.startswith("newmtl ") and line.split()[1] == material_name:
                                        in_material = True
                                        material_lines = [line]
                                    elif line.startswith("newmtl ") and in_material:
                                        break
                                    elif in_material:
                                        material_lines.append(line)
                                
                                if material_lines:
                                    material = Material.from_string(material_lines)
                                    obj_stem = obj_file_path.stem
                                    material.name = f"{obj_stem}_{material_name}"
                                    obj_materials[material.name] = material
                                    logger.info(f"Added single OBJ material: {material.name}")
                    except Exception as e:
                        logger.warning(f"Failed to parse single-material OBJ {obj_file_path}: {e}")

    # Add assets
    add_assets(mjcf_root, materials, obj_materials)

    # Replace the actuator block with one that uses positional control.
    actuator_elem = ET.SubElement(mjcf_root, "actuator")
    for actuator_joint in actuator_joints:
        # The class name is the actuator type
        attrib: dict[str, str] = {"joint": actuator_joint.name}
        actuator_type_value = "motor"
        if actuator_joint.name in actuator_metadata:
            if actuator_metadata[actuator_joint.name].actuator_type is not None:
                actuator_type_value = actuator_metadata[actuator_joint.name].actuator_type
                logger.info("Joint %s assigned to class: %s", actuator_joint.name, actuator_type_value)

            if actuator_metadata[actuator_joint.name].joint_class is not None:
                joint_class_value = actuator_metadata[actuator_joint.name].joint_class
                attrib["class"] = str(joint_class_value)
                logger.info("Joint %s assigned to class: %s", actuator_joint.name, joint_class_value)
            
            if actuator_metadata[actuator_joint.name].kp is not None:
                attrib["kp"] = str(actuator_metadata[actuator_joint.name].kp)
            if actuator_metadata[actuator_joint.name].kv is not None:
                attrib["kv"] = str(actuator_metadata[actuator_joint.name].kv)
            if actuator_metadata[actuator_joint.name].ctrlrange is not None:
                attrib["ctrlrange"] = f"{actuator_metadata[actuator_joint.name].ctrlrange[0]} {actuator_metadata[actuator_joint.name].ctrlrange[1]}"
            if actuator_metadata[actuator_joint.name].forcerange is not None:
                attrib["forcerange"] = f"{actuator_metadata[actuator_joint.name].forcerange[0]} {actuator_metadata[actuator_joint.name].forcerange[1]}"
            if actuator_metadata[actuator_joint.name].gear is not None:
                attrib["gear"] = str(actuator_metadata[actuator_joint.name].gear)

            logger.info(f"Creating actuator {actuator_joint.name} with class: {actuator_type_value}")
            ET.SubElement(actuator_elem, actuator_type_value, attrib={"name": f"{actuator_joint.name}", **attrib})

        else:
            logger.info(f"Actuator {actuator_joint.name} not found in actuator_metadata")

    # 对actuator_elem进行排序，按照在actuator_metadata出现的顺序排序
    actuator_children = []
    actuator_lst = list(actuator_elem)
    for actuator in actuator_lst:
        if actuator.attrib["joint"] in actuator_metadata.keys():
            actuator_children.append(actuator)
        else:
            logger.warning(f"Warning: Actuator {actuator.attrib['joint']} not found in actuator_metadata")

    actuator_children.sort(key=lambda x: list(actuator_metadata.keys()).index(x.attrib["joint"]))
    
    # 清空actuator_elem并重新添加排序后的子元素
    for child in actuator_children:
        actuator_elem.remove(child)
    for child in actuator_children:
        actuator_elem.append(child)

    # Add equality constraints for mimic joints
    if mimic_constraints:
        equality_elem = ET.SubElement(mjcf_root, "equality")
        for mimicked_joint, mimicking_joint, multiplier, offset in mimic_constraints:
            joint_attrib = {
                "joint1": mimicked_joint,
                "joint2": mimicking_joint
            }
            
            # Generate polycoef attribute for MuJoCo equality constraint
            # MuJoCo polycoef format: "offset multiplier 0 0 0" for linear relationship
            # This creates the constraint: joint2 = offset + multiplier * joint1
            polycoef = f"{offset} {multiplier} 0 0 0"
            joint_attrib["polycoef"] = polycoef
            
            # Use collision class defaults for solver parameters if available
            joint_attrib["solimp"] = "0.95 0.99 0.001"
            joint_attrib["solref"] = "0.005 1"
            
            ET.SubElement(equality_elem, "joint", attrib=joint_attrib)
            logger.info(f"Added equality constraint: {mimicking_joint} = {offset} + {multiplier} * {mimicked_joint}")

    # Add mesh assets to the asset section before saving
    asset_elem: ET.Element | None = mjcf_root.find("asset")
    if asset_elem is None:
        asset_elem = ET.SubElement(mjcf_root, "asset")
    for mesh_name, filename in mesh_assets.items():
        # Clean up package:// paths to relative paths
        if 'package://' in filename:
            # Extract the relative path after package name
            package_path = filename[len('package://'):]
            parts = package_path.split('/')
            if len(parts) > 1:
                # Remove the package name part, keep the rest as relative path
                relative_path = '/'.join(parts[1:])
                filename = relative_path
        elif filename.startswith('/'):
            filename = os.path.relpath(filename, urdf_dir)

        # mesh_name already should be the stem (without .obj), 
        # so it will match the geom references
        ET.SubElement(asset_elem, "mesh", attrib={"name": mesh_name, "file": filename})

    add_contact(mjcf_root, robot)

    # Add weld constraints if specified in metadata
    add_weld_constraints(mjcf_root, metadata)

    # Copy mesh files with special handling for OBJ files
    processed_files = set()
    
    for mesh_name, filename in mesh_assets.items():
        # Determine source path based on whether it's a package:// URL or regular path
        source_path: Path | None = None
        target_path: Path | None = None
        
        if 'package://' in filename:
            # Extract package name and relative path from package URL
            package_path = filename[len('package://'):]
            pkg_mesh_name = package_path.split('/')[0]
            sub_path = '/'.join(package_path.split('/')[1:])
            # Use package_resolver to find the package path
            try:
                pkg_root = resolve_package_path(pkg_mesh_name, workspace_search_paths)
                if pkg_root:
                    source_path = pkg_root / sub_path
                else:
                    source_path = None
            except:
                source_path = None
        elif filename.startswith('/'):
            sub_path = os.path.relpath(filename, urdf_dir)
            source_path = Path(filename)
        else:
            sub_path = filename
            source_path = (urdf_dir / filename).resolve()

        if source_path and not source_path.exists():
            logger.error(f"Mesh file not found: {source_path} (from URDF path: {filename})")
            raise FileNotFoundError(f"Mesh file not found: {source_path} (from URDF path: {filename})")

        target_path = target_mesh_dir / sub_path
        # Copy the file if source exists and target is valid
        if source_path and target_path and source_path.exists():
            if not target_path.parent.exists():
                target_path.parent.mkdir(parents=True, exist_ok=True)
            # Only copy if we haven't processed this file already
            if str(target_path) not in processed_files:
                if source_path != target_path:
                    try:
                        # Special handling for OBJ files - copy with MTL
                        if source_path.suffix.lower() == '.obj':
                            copy_obj_with_mtl(source_path, target_path)
                        else:
                            shutil.copy2(source_path, target_path)
                        processed_files.add(str(target_path))
                        logger.debug(f"Copied mesh file: {source_path} -> {target_path}")
                    except Exception as e:
                        logger.warning(f"Failed to copy mesh file {source_path} to {target_path}: {e}")
        elif source_path:
            logger.warning(f"Mesh file not found: {source_path}")

    # Save the initial MJCF file
    print(f"Saving initial MJCF file to {mjcf_path}")
    save_xml(mjcf_path, ET.ElementTree(mjcf_root))
    print(f"Added light...")
    add_light(mjcf_path)
    if convex_decompose:
        print(f"Convex decomposition...")
        convex_decomposition(mjcf_path)
    if not collision_only:
        print(f"Split OBJ files by materials...")
        split_obj_by_materials(mjcf_path)  # Split OBJ files by materials
        print(f"Checking shell meshes...")
        check_shell_meshes(mjcf_path)

    update_mesh(mjcf_path, max_vertices)

    # Apply post-processing steps
    if metadata.angle != "radian":
        assert metadata.angle == "degree", "Only 'radian' and 'degree' are supported."
        make_degrees(mjcf_path)
    if metadata.backlash:
        add_backlash(mjcf_path, metadata.backlash, metadata.backlash_damping)
    if metadata.freejoint:
        fix_base_joint(mjcf_path, metadata.freejoint)
    if metadata.add_floor:
        add_floor(mjcf_path)
    if metadata.remove_redundancies:
        remove_redundancies(mjcf_path)
    if (collision_geometries := metadata.collision_geometries) is not None:
        update_collisions(mjcf_path, collision_geometries)
    if (explicit_contacts := metadata.explicit_contacts) is not None:
        add_explicit_floor_contacts(
            mjcf_path,
            contact_links=explicit_contacts.contact_links,
            class_name=explicit_contacts.class_name,
            floor_name=metadata.floor_name,
        )
    
    if appendix_files is not None and len(appendix_files) > 0:
        print(f"Adding appendix...")
        for appendix_file in appendix_files:
            add_appendix(mjcf_path, appendix_file)

def main() -> None:
    """Parse command-line arguments and execute the URDF to MJCF conversion."""
    parser = argparse.ArgumentParser(description="Convert a URDF file to an MJCF file.")

    parser.add_argument(
        "urdf_path",
        type=str,
        help="The path to the URDF file.",
    )
    parser.add_argument(
        "-o", "--output",
        type=str,
        help="The path to the output MJCF file.",
    )
    parser.add_argument(
        "--collision-only",
        action="store_true", 
        help="If true, use collision geometry without visual appearance for visual representation."
    )
    parser.add_argument(
        "--no-convex-decompose",
        action="store_true", 
        help="If true, do not run convex decomposition on the mesh."
    )
    parser.add_argument(
        "-m", "--metadata",
        type=str,
        default=None,
        help="A JSON file containing conversion metadata (joint params and sensors).",
    )
    parser.add_argument(
        "-dm",
        "--default-metadata",
        nargs='*',
        default=None,
        help="JSON files containing default metadata. Multiple files will be merged, with later files overriding earlier ones.",
    )
    parser.add_argument(
        "-am",
        "--actuator-metadata",
        nargs='*',
        default=None,
        help="JSON files containing actuator metadata. Multiple files will be merged, with later files overriding earlier ones.",
    )
    parser.add_argument(
        "-a",
        "--appendix",
        nargs='*',
        default=None,
        help="XML files containing appendix. Multiple files will be applied in order.",
    )
    parser.add_argument(
        "--log-level",
        type=int,
        default=logging.INFO,
        help="The log level to use.",
    )
    parser.add_argument(
        "--max-vertices",
        type=int,
        default=1000000,
        help="Maximum number of vertices in the mesh.",
    )
    args = parser.parse_args()
    logger.setLevel(args.log_level)

    # Load default metadata
    default_metadata = {}
    if args.default_metadata is not None and len(args.default_metadata) > 0:
        for metadata_file in args.default_metadata:
            try:
                with open(metadata_file, "r") as f:
                    file_metadata = json.load(f)
                    for key, value in file_metadata.items():
                        default_metadata[key] = DefaultJointMetadata.from_dict(value)
                logger.info(f"Loaded default metadata from {metadata_file}")
            except Exception as e:
                logger.warning("Failed to load default metadata from %s: %s", metadata_file, e)
                traceback.print_exc()
                exit(1)
    
    if not default_metadata:
        default_metadata = None

    # Load actuator metadata
    actuator_metadata = {}
    if args.actuator_metadata is not None and len(args.actuator_metadata) > 0:
        for metadata_file in args.actuator_metadata:
            try:
                with open(metadata_file, "r") as f:
                    file_metadata = json.load(f)
                    for key, value in file_metadata.items():
                        actuator_metadata[key] = ActuatorMetadata.from_dict(value)
                logger.info(f"Loaded actuator metadata from {metadata_file}")
            except Exception as e:
                logger.warning("Failed to load actuator metadata from %s: %s", metadata_file, e)
                traceback.print_exc()
                exit(1)
    
    if not actuator_metadata:
        actuator_metadata = None

    convert_urdf_to_mjcf(
        urdf_path=args.urdf_path,
        mjcf_path=args.output,
        metadata_file=args.metadata,
        default_metadata=default_metadata,
        actuator_metadata=actuator_metadata,
        appendix_files=[Path(appendix_file) for appendix_file in args.appendix] if args.appendix is not None and len(args.appendix) > 0 else None,
        max_vertices=args.max_vertices,
        collision_only=args.collision_only,
        convex_decompose=not args.no_convex_decompose
    )

if __name__ == "__main__":
    main()
