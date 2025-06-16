"""Converts URDF files to MJCF files."""

import json
import shutil
import logging
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

from urdf2mjcf.model import ActuatorMetadata, ConversionMetadata, JointMetadata
from urdf2mjcf.postprocess.add_backlash import add_backlash
from urdf2mjcf.postprocess.add_floor import add_floor
from urdf2mjcf.postprocess.add_light import add_light
from urdf2mjcf.postprocess.update_mesh import update_mesh
from urdf2mjcf.postprocess.convex_decomposition import convex_decomposition
from urdf2mjcf.postprocess.base_joint import fix_base_joint
from urdf2mjcf.postprocess.collisions import update_collisions
from urdf2mjcf.postprocess.explicit_floor_contacts import add_explicit_floor_contacts
from urdf2mjcf.postprocess.make_degrees import make_degrees
from urdf2mjcf.postprocess.remove_redundancies import remove_redundancies
from urdf2mjcf.postprocess.check_shell import check_shell_meshes
from urdf2mjcf.utils import save_xml

from urdf2mjcf.materials import Material, process_obj_mtl_materials
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
    add_option,
    add_visual,
    add_assets,
    ROBOT_CLASS
)
from urdf2mjcf.package_resolver import resolve_package_path, resolve_package_resource, find_workspace_from_path

logger = logging.getLogger(__name__)


def _get_empty_joint_and_actuator_metadata(
    robot_elem: ET.Element,
) -> tuple[dict[str, JointMetadata], dict[str, ActuatorMetadata]]:
    """Create placeholder metadata for joints and actuators if none are provided.

    Each joint is simply assigned a "motor" actuator type, which has no other parameters.
    """
    joint_meta: dict[str, JointMetadata] = {}
    for idx, joint in enumerate(robot_elem.findall("joint")):
        name = joint.attrib.get("name")
        if not name:
            continue
        joint_meta[name] = JointMetadata(
            actuator_type="motor",
            id=idx,
            nn_id=idx,
            kp=1.0,
            kd=1.0,
            soft_torque_limit=1.0,
            min_angle_deg=0.0,
            max_angle_deg=0.0,
        )

    actuator_meta = {"motor": ActuatorMetadata(actuator_type="motor")}
    return joint_meta, actuator_meta


def convert_urdf_to_mjcf(
    urdf_path: str | Path,
    mjcf_path: str | Path | None = None,
    metadata: ConversionMetadata | None = None,
    metadata_file: str | Path | None = None,
    *,
    joint_metadata: dict[str, JointMetadata] | None = None,
    actuator_metadata: dict[str, ActuatorMetadata] | None = None,
) -> None:
    """Converts a URDF file to an MJCF file.

    Args:
        urdf_path: The path to the URDF file.
        mjcf_path: The desired output MJCF file path.
        metadata: Optional conversion metadata.
        metadata_file: Optional path to metadata file.
        joint_metadata: Optional joint metadata.
        actuator_metadata: Optional actuator metadata.
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

    if metadata_file is not None and metadata is not None:
        raise ValueError("Cannot specify both metadata and metadata_file")
    elif metadata_file is not None:
        with open(metadata_file, "r") as f:
            metadata = ConversionMetadata.model_validate_json(f.read())
    if metadata is None:
        metadata = ConversionMetadata()

    if joint_metadata is None or actuator_metadata is None:
        missing = []
        if joint_metadata is None:
            missing.append("joint")
        if actuator_metadata is None:
            missing.append("actuator")
        logger.warning("Missing %s metadata, falling back to single empty 'motor' class.", " and ".join(missing))
        joint_metadata, actuator_metadata = _get_empty_joint_and_actuator_metadata(robot)
    assert joint_metadata is not None and actuator_metadata is not None

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

    # Add compiler, option, visual, but NOT assets yet (we need to process MTL materials first)
    add_compiler(mjcf_root)
    add_option(mjcf_root)
    add_visual(mjcf_root)
    add_default(mjcf_root, metadata, joint_metadata, actuator_metadata)

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
    
    # Dictionary to collect MTL materials from OBJ files
    mtl_materials: dict[str, Material] = {}
    
    # Prepare paths for mesh processing
    urdf_dir: Path = urdf_path.parent.resolve()
    target_mesh_dir: Path = (mjcf_path.parent).resolve()
    target_mesh_dir.mkdir(parents=True, exist_ok=True)
    
    # Auto-detect workspace search paths for package resolution
    workspace_search_paths = []
    
    # Strategy 1: Find workspace from URDF file location
    workspace_from_urdf = find_workspace_from_path(urdf_dir)
    if workspace_from_urdf:
        workspace_search_paths.append(workspace_from_urdf)
        logger.debug(f"Found ROS workspace from URDF location: {workspace_from_urdf}")
    
    # Strategy 2: Check current working directory
    workspace_from_cwd = find_workspace_from_path(Path.cwd())
    if workspace_from_cwd and workspace_from_cwd not in workspace_search_paths:
        workspace_search_paths.append(workspace_from_cwd)
        logger.debug(f"Found ROS workspace from CWD: {workspace_from_cwd}")
    
    # Strategy 3: Add some manual paths as fallback
    cwd = Path.cwd()
    for i in range(3):  # Check up to 3 levels up
        if cwd not in workspace_search_paths:
            workspace_search_paths.append(cwd)
        if cwd.parent == cwd:  # At filesystem root
            break
        cwd = cwd.parent

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
                    
                    # Process MTL materials for OBJ files
                    if mesh_name.lower().endswith('.obj'):
                        # Handle package:// paths correctly
                        if filename.startswith('package://'):
                            # Extract package name and relative path
                            package_path = filename[len('package://'):]
                            pkg_name = package_path.split('/')[0]
                            sub_path = '/'.join(package_path.split('/')[1:])
                            
                            # Use package_resolver to find the package path
                            try:
                                pkg_root = resolve_package_path(pkg_name, workspace_search_paths)
                                if pkg_root:
                                    mesh_file_path = pkg_root / sub_path
                                else:
                                    mesh_file_path = None
                            except (ImportError, Exception) as e:
                                logger.warning(f"Could not resolve package path {filename}: {e}")
                                mesh_file_path = None
                        else:
                            mesh_file_path = urdf_dir / filename if not filename.startswith('/') else Path(filename)
                        
                        if mesh_file_path:
                            obj_mtl_materials = process_obj_mtl_materials(mesh_file_path, target_mesh_dir)
                            mtl_materials.update(obj_mtl_materials)
                            if obj_mtl_materials:
                                logger.info(f"Loaded {len(obj_mtl_materials)} MTL materials from {mesh_name}")
                        
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

                # Only for slide and hinge joints
                j_attrib["ref"] = "0.0"

                if j_name not in joint_metadata:
                    raise ValueError(f"Joint {j_name} not found in joint_metadata")
                actuator_type_value = joint_metadata[j_name].actuator_type
                j_attrib["class"] = str(actuator_type_value)
                logger.info("Joint %s assigned to class: %s", j_name, actuator_type_value)

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
                inertial_elem.attrib["quat"] = rpy_to_quat(rpy)
            mass_elem = inertial.find("mass")
            if mass_elem is not None:
                inertial_elem.attrib["mass"] = mass_elem.attrib.get("value", "0")
            inertia_elem = inertial.find("inertia")
            if inertia_elem is not None:
                ixx = float(inertia_elem.attrib.get("ixx", "0"))
                ixy = float(inertia_elem.attrib.get("ixy", "0"))
                ixz = float(inertia_elem.attrib.get("ixz", "0"))
                iyy = float(inertia_elem.attrib.get("iyy", "0"))
                iyz = float(inertia_elem.attrib.get("iyz", "0"))
                izz = float(inertia_elem.attrib.get("izz", "0"))
                if abs(ixy) > 1e-6 or abs(ixz) > 1e-6 or abs(iyz) > 1e-6:
                    logger.warning(
                        "Warning: off-diagonal inertia terms for link '%s' are nonzero and will be ignored.",
                        link_name,
                    )
                inertial_elem.attrib["diaginertia"] = f"{ixx} {iyy} {izz}"
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
                
                # Handle OBJ files with potential material splitting
                if geom.type == "mesh" and geom.mesh is not None and geom.mesh.lower().endswith('.obj'):
                    # Check if we have split meshes for this OBJ file
                    obj_name = Path(geom.mesh).stem
                    
                    # For package:// paths, find the actual file location
                    if mesh_assets.get(geom.mesh, "").startswith('package://'):
                        obj_filename = mesh_assets[geom.mesh]
                        package_path = obj_filename[len('package://'):]
                        pkg_name = package_path.split('/')[0]
                        sub_path = '/'.join(package_path.split('/')[1:])
                        try:
                            pkg_root = resolve_package_path(pkg_name, workspace_search_paths)
                            if pkg_root:
                                original_obj_file = pkg_root / sub_path
                            else:
                                original_obj_file = None
                        except:
                            original_obj_file = None
                    else:
                        # For non-package paths, resolve relative to URDF directory
                        obj_filename = mesh_assets.get(geom.mesh, geom.mesh)
                        if obj_filename.startswith('/'):
                            # Absolute path
                            original_obj_file = Path(obj_filename)
                        else:
                            # Relative path - relative to URDF file directory
                            original_obj_file = urdf_dir / obj_filename
                    
                    if original_obj_file and original_obj_file.exists():
                        obj_target_dir = original_obj_file.parent / obj_name
                        
                        if obj_target_dir.exists():
                            # Check for split submeshes
                            submeshes = list(obj_target_dir.glob(f"{obj_name}_*.obj"))
                            
                            if submeshes:
                                # Multiple submeshes found - create a geom for each
                                logger.info(f"Found {len(submeshes)} submeshes for {geom.mesh}")
                                
                                for i, submesh_path in enumerate(sorted(submeshes)):
                                    submesh_name = submesh_path.name
                                    submesh_stem = submesh_path.stem  # without .obj extension
                                    
                                    # Create geom name
                                    geom_name = f"{link_name}_visual"
                                    if len(visuals) > 1:
                                        geom_name = f"{geom_name}_{idx}"
                                    geom_name = f"{geom_name}_{i}"
                                    
                                    visual_geom_attrib = {
                                        "name": geom_name,
                                        "pos": pos_geom,
                                        "quat": quat_geom,
                                        "type": "mesh",
                                        "mesh": submesh_stem,  # Use stem (without .obj) to match asset name
                                        "class": "visual"
                                    }
                                    
                                    if geom.scale is not None:
                                        visual_geom_attrib["scale"] = geom.scale
                                    
                                    # Try to find the corresponding MTL material for this submesh
                                    assigned_material = "default_material"
                                    
                                    # Read the submesh OBJ file to find which material it uses
                                    try:
                                        with open(submesh_path, 'r') as f:
                                            submesh_lines = f.readlines()
                                        for line in submesh_lines:
                                            if line.startswith('usemtl '):
                                                mtl_name_raw = line.split()[1].strip()
                                                mtl_name = f"{original_obj_file.stem}_{mtl_name_raw}"
                                                if mtl_name in mtl_materials:
                                                    assigned_material = mtl_name
                                                    logger.info(f"Using MTL material '{mtl_name}' for submesh '{submesh_name}'")
                                                    break
                                    except Exception as e:
                                        logger.warning(f"Could not read submesh {submesh_path} to find material: {e}")

                                    # If no MTL material found, check URDF material
                                    if assigned_material == "default_material":
                                        material_elem = visual.find("material")
                                        if material_elem is not None:
                                            material_name = material_elem.attrib.get("name")
                                            if material_name and material_name in materials:
                                                assigned_material = material_name
                                    
                                    visual_geom_attrib["material"] = assigned_material
                                    ET.SubElement(body, "geom", attrib=visual_geom_attrib)
                                    
                                    # Add the submesh to mesh_assets if not already added
                                    # Use stem as key to match the geom reference
                                    if submesh_stem not in mesh_assets:
                                        # Generate the path following the original mesh structure
                                        # Example: meshes/chassis/omni_chassis_base_link.obj -> meshes/chassis/omni_chassis_base_link/omni_chassis_base_link_0.obj
                                        if mesh_assets.get(geom.mesh):
                                            original_path_str = mesh_assets[geom.mesh]
                                            # Clean package:// path if present
                                            if original_path_str.startswith('package://'):
                                                package_path = original_path_str[len('package://'):]
                                                parts = package_path.split('/')
                                                if len(parts) > 1:
                                                    # Remove the package name part, keep the rest as relative path
                                                    original_path_str = '/'.join(parts[1:])
                                            
                                            original_path = Path(original_path_str)
                                            # Create the submesh path by inserting the obj_name directory
                                            submesh_rel_path = original_path.parent / obj_name / submesh_name
                                            
                                            # Copy to target_mesh_dir maintaining the directory structure
                                            dest_file = target_mesh_dir / submesh_rel_path
                                            dest_file.parent.mkdir(parents=True, exist_ok=True)
                                            shutil.copy2(submesh_path, dest_file)
                                            mesh_assets[submesh_stem] = str(submesh_rel_path)
                                        else:
                                            # Fallback: create submesh directory structure in target_mesh_dir
                                            dest_dir = target_mesh_dir / obj_name
                                            dest_dir.mkdir(parents=True, exist_ok=True)
                                            dest_file = dest_dir / submesh_name
                                            shutil.copy2(submesh_path, dest_file)
                                            rel_path = dest_file.relative_to(target_mesh_dir)
                                            mesh_assets[submesh_stem] = str(rel_path)
                                
                                continue  # Skip the single geom creation below
                            else:
                                # No submeshes, check for single copied mesh
                                single_mesh = obj_target_dir / f"{obj_name}.obj"
                                if single_mesh.exists():
                                    # Copy to target dir maintaining the original directory structure
                                    if mesh_assets.get(geom.mesh):
                                        original_path_str = mesh_assets[geom.mesh]
                                        # Clean package:// path if present
                                        if original_path_str.startswith('package://'):
                                            package_path = original_path_str[len('package://'):]
                                            parts = package_path.split('/')
                                            if len(parts) > 1:
                                                # Remove the package name part, keep the rest as relative path
                                                original_path_str = '/'.join(parts[1:])
                                        
                                        original_path = Path(original_path_str)
                                        # Create the path following original structure: meshes/chassis/omni_chassis_base_link.obj -> meshes/chassis/omni_chassis_base_link/omni_chassis_base_link.obj
                                        single_mesh_rel_path = original_path.parent / obj_name / f"{obj_name}.obj"
                                        
                                        # Copy to target_mesh_dir maintaining the directory structure
                                        dest_file = target_mesh_dir / single_mesh_rel_path
                                        dest_file.parent.mkdir(parents=True, exist_ok=True)
                                        shutil.copy2(single_mesh, dest_file)
                                        mesh_assets[geom.mesh] = str(single_mesh_rel_path)
                                    else:
                                        # Fallback: copy to simple subdirectory
                                        dest_dir = target_mesh_dir / obj_name  
                                        dest_dir.mkdir(parents=True, exist_ok=True)
                                        dest_file = dest_dir / single_mesh.name
                                        shutil.copy2(single_mesh, dest_file)
                                        rel_path = dest_file.relative_to(target_mesh_dir)
                                        mesh_assets[geom.mesh] = str(rel_path)
                                
                                # For single mesh with material, we still need to create the geom with proper material assignment
                                # Don't skip to standard geom creation for this case
                
                # Standard single geom creation (for non-split meshes or other types)
                name = f"{link_name}_visual"
                if len(visuals) > 1:
                    name = f"{name}_{idx}"
                visual_geom_attrib: dict[str, str] = {"name": name, "pos": pos_geom, "quat": quat_geom}
                
                visual_geom_attrib["type"] = geom.type
                if geom.type == "mesh" and geom.mesh is not None:
                    visual_geom_attrib["mesh"] = geom.mesh
                    
                    # Check if this is an OBJ file with MTL materials (single mesh case)
                    if geom.mesh.lower().endswith('.obj') and mtl_materials:
                        assigned_material = "default_material"
                        
                        # Try to find the first available MTL material for this mesh
                        obj_filename = mesh_assets.get(geom.mesh, "")
                        
                        # For single mesh, try to read the original or copied file
                        obj_paths_to_try = []
                        if obj_filename:
                            if obj_filename.startswith('package://'):
                                # Resolve package path
                                package_path = obj_filename[len('package://'):]
                                pkg_name = package_path.split('/')[0]
                                sub_path = '/'.join(package_path.split('/')[1:])
                                try:
                                    pkg_root = resolve_package_path(pkg_name, workspace_search_paths)
                                    if pkg_root:
                                        obj_paths_to_try.append(pkg_root / sub_path)
                                except:
                                    pass
                            else:
                                # For non-package paths, resolve relative to URDF directory
                                if obj_filename.startswith('/'):
                                    # Absolute path
                                    obj_paths_to_try.append(Path(obj_filename))
                                else:
                                    # Relative path - relative to URDF file directory
                                    obj_paths_to_try.append(urdf_dir / obj_filename)
                        
                        # Also try the copied/processed version
                        obj_name = Path(geom.mesh).stem
                        obj_target_dir = target_mesh_dir / obj_name
                        obj_paths_to_try.append(obj_target_dir / f"{obj_name}.obj")
                        
                        # Also try the path following the original mesh structure
                        if mesh_assets.get(geom.mesh):
                            original_path = Path(mesh_assets[geom.mesh])
                            structured_path = target_mesh_dir / original_path.parent / obj_name / f"{obj_name}.obj"
                            obj_paths_to_try.append(structured_path)
                        
                        for obj_path in obj_paths_to_try:
                            if obj_path and obj_path.exists():
                                try:
                                    with open(obj_path, 'r') as f:
                                        obj_lines = f.readlines()
                                    for line in obj_lines:
                                        if line.startswith('usemtl '):
                                            mtl_name_raw = line.split()[1].strip()
                                            # MTL materials are prefixed with obj file stem
                                            obj_stem = Path(geom.mesh).stem
                                            mtl_name = f"{obj_stem}_{mtl_name_raw}"
                                            if mtl_name in mtl_materials:
                                                assigned_material = mtl_name
                                                logger.info(f"Using MTL material '{mtl_name}' for mesh '{geom.mesh}'")
                                                break
                                    if assigned_material != "default_material":
                                        break
                                except Exception as e:
                                    logger.warning(f"Could not read OBJ file {obj_path} to find material: {e}")
                        
                        # If no specific material found, use the first available MTL material for this specific OBJ file
                        if assigned_material == "default_material" and mtl_materials:
                            obj_stem = Path(geom.mesh).stem
                            # Only look for materials that belong to this specific OBJ file
                            obj_specific_materials = {k: v for k, v in mtl_materials.items() if k.startswith(f"{obj_stem}_")}
                            if obj_specific_materials:
                                first_mtl = next(iter(obj_specific_materials.values()))
                                assigned_material = first_mtl.name
                                logger.info(f"Using first MTL material '{assigned_material}' for mesh '{geom.mesh}'")
                    else:
                        assigned_material = "default_material"
                    
                elif geom.size is not None:
                    visual_geom_attrib["size"] = geom.size
                    assigned_material = "default_material"
                else:
                    assigned_material = "default_material"
                    
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
                assigned_material = "default_material"
            
            # If no MTL material was assigned, check URDF material
            if assigned_material == "default_material":
                material_elem = visual.find("material")
                if material_elem is not None:
                    material_name = material_elem.attrib.get("name")
                    if material_name and material_name in materials:
                        assigned_material = material_name
                        
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

    # Now that we've processed all meshes and collected MTL materials, add assets
    add_assets(mjcf_root, materials, mtl_materials, metadata.visualize_collision_meshes)

    # Replace the actuator block with one that uses positional control.
    actuator_elem = ET.SubElement(mjcf_root, "actuator")
    for actuator_joint in actuator_joints:
        # The class name is the actuator type
        attrib: dict[str, str] = {"joint": actuator_joint.name}
        if actuator_joint.name not in joint_metadata:
            raise ValueError(f"Actuator {actuator_joint.name} not found in joint_metadata")
        actuator_type_value = joint_metadata[actuator_joint.name].actuator_type

        attrib["class"] = str(actuator_type_value)
        logger.info(f"Creating actuator {actuator_joint.name}_ctrl with class: {actuator_type_value}")

        ET.SubElement(actuator_elem, "motor", attrib={"name": f"{actuator_joint.name}_ctrl", **attrib})

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
        # mesh_name already should be the stem (without .obj), 
        # so it will match the geom references
        ET.SubElement(asset_elem, "mesh", attrib={"name": mesh_name, "file": filename})

    add_contact(mjcf_root, robot)

    # Add weld constraints if specified in metadata
    add_weld_constraints(mjcf_root, metadata)

    # Copy mesh files - but skip the ones that have already been processed and copied

    # Track which files we've already processed/copied to avoid duplicates
    processed_files = set()
    
    for mesh_name, filename in mesh_assets.items():
        # Skip files that have been processed and are already in the correct subdirectory structure
        if not filename.startswith('package://') and ('/' in filename):
            # This file has been processed and placed in subdirectory structure, skip copying to root
            continue
            
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
            target_path = target_mesh_dir / sub_path
        else:
            # For non-package paths, try to resolve relative to URDF directory
            if filename.startswith('/'):
                # Absolute path
                source_path = Path(filename)
            else:
                # Relative path - relative to URDF file directory
                source_path = (urdf_dir / filename).resolve()
            
            # If the path doesn't exist, log a warning but continue
            if source_path and not source_path.exists():
                logger.warning(f"Mesh file not found: {source_path} (from URDF path: {filename})")
                continue
                
            target_path = target_mesh_dir / Path(filename).name
        
        # Copy the file if source exists and target is valid
        if source_path and target_path and source_path.exists():
            if not target_path.parent.exists():
                target_path.parent.mkdir(parents=True, exist_ok=True)
            # Only copy if we haven't processed this file already
            if str(target_path) not in processed_files:
                if source_path != target_path:
                    try:
                        shutil.copy2(source_path, target_path)
                        processed_files.add(str(target_path))
                        logger.debug(f"Copied mesh file: {source_path} -> {target_path}")
                    except Exception as e:
                        logger.warning(f"Failed to copy mesh file {source_path} to {target_path}: {e}")
        elif source_path:
            logger.warning(f"Mesh file not found: {source_path}")

    # Save the initial MJCF file
    save_xml(mjcf_path, ET.ElementTree(mjcf_root))
    add_floor(mjcf_path)
    add_light(mjcf_path)
    convex_decomposition(mjcf_path, urdf_dir=urdf_path.parent)
    
    # After convex decomposition, we need to copy any newly generated mesh files
    # Re-parse the MJCF file to get the updated mesh assets
    logger.info("Copying any newly generated mesh files after post-processing...")
    updated_tree = ET.parse(mjcf_path)
    updated_root = updated_tree.getroot()
    updated_asset_elem = updated_root.find("asset")
    
    if updated_asset_elem is not None:
        copied_files = set()  # Track copied files to avoid duplicates
        
        for mesh_elem in updated_asset_elem.findall("mesh"):
            mesh_name = mesh_elem.get("name", "")
            mesh_file = mesh_elem.get("file", "")
            
            if not mesh_file:
                continue
            
            target_path = target_mesh_dir / mesh_file
            
            # Skip if already copied or target already exists
            if str(target_path) in copied_files or target_path.exists():
                continue
            
            # Try to find the source file
            source_path = None
            
            if mesh_file.startswith('package://'):
                # Handle package:// paths
                package_path = mesh_file[len('package://'):]
                pkg_name = package_path.split('/')[0]
                sub_path = '/'.join(package_path.split('/')[1:])
                try:
                    pkg_root = resolve_package_path(pkg_name, workspace_search_paths)
                    if pkg_root:
                        source_path = pkg_root / sub_path
                except:
                    source_path = None
            else:
                # For regular paths, try different locations in order
                if Path(mesh_file).is_absolute():
                    source_path = Path(mesh_file)
                else:
                    # Try multiple potential source locations
                    potential_sources = [
                        urdf_dir / mesh_file,  # Relative to URDF directory
                        mjcf_path.parent / mesh_file,  # Relative to MJCF directory (might already be copied)
                    ]
                    
                    # Also try looking in subdirectories based on the mesh structure
                    # For files like "meshes/visual/right_arm/right_arm_link3/obj/right_arm_link3.obj"
                    # we might need to look in the original URDF structure
                    mesh_path = Path(mesh_file)
                    if len(mesh_path.parts) > 1:
                        # Try the original mesh structure relative to URDF
                        potential_sources.append(urdf_dir / mesh_file)
                        # Also try without the first directory component (in case "meshes" is redundant)
                        if mesh_path.parts[0] == "meshes" and len(mesh_path.parts) > 1:
                            relative_path = Path(*mesh_path.parts[1:])
                            potential_sources.append(urdf_dir / relative_path)
                    
                    for potential_source in potential_sources:
                        if potential_source.exists():
                            source_path = potential_source
                            break
            
            # Copy the file if source exists
            if source_path and source_path.exists():
                try:
                    target_path.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(source_path, target_path)
                    copied_files.add(str(target_path))
                    logger.debug(f"Copied mesh file: {source_path} -> {target_path}")
                except Exception as e:
                    logger.warning(f"Failed to copy mesh file {source_path} to {target_path}: {e}")
            else:
                logger.warning(f"Could not find source file for mesh: {mesh_file}")
        
        logger.info(f"Copied {len(copied_files)} mesh files after post-processing")
    
    # update_mesh(mjcf_path)
    check_shell_meshes(mjcf_path)

    # Apply post-processing steps
    if metadata.angle != "radian":
        assert metadata.angle == "degree", "Only 'radian' and 'degree' are supported."
        make_degrees(mjcf_path)
    if metadata.backlash:
        add_backlash(mjcf_path, metadata.backlash, metadata.backlash_damping)
    if metadata.floating_base:
        fix_base_joint(mjcf_path, metadata.freejoint)
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


def main() -> None:
    """Parse command-line arguments and execute the URDF to MJCF conversion."""
    parser = argparse.ArgumentParser(description="Convert a URDF file to an MJCF file.")

    parser.add_argument(
        "urdf_path",
        type=str,
        help="The path to the URDF file.",
    )
    parser.add_argument(
        "--output",
        type=str,
        help="The path to the output MJCF file.",
    )
    parser.add_argument(
        "--metadata",
        type=str,
        help="A JSON string containing conversion metadata (joint params and sensors).",
    )
    parser.add_argument(
        "--metadata-file",
        type=str,
        help="A JSON file containing conversion metadata (joint params and sensors).",
    )
    parser.add_argument(
        "--log-level",
        type=int,
        default=logging.INFO,
        help="The log level to use.",
    )
    args = parser.parse_args()
    logger.setLevel(args.log_level)

    # Parse the raw metadata from the command line arguments.
    raw_metadata: dict | None = None
    if args.metadata_file is not None and args.metadata is not None:
        raise ValueError("Cannot specify both --metadata and --metadata-file")
    elif args.metadata_file is not None:
        with open(args.metadata_file, "r") as f:
            raw_metadata = json.load(f)
    elif args.metadata is not None:
        raw_metadata = json.loads(args.metadata)
    elif (metadata_path := Path(args.urdf_path).parent / "metadata.json").exists():
        logger.warning("Using metadata from %s", metadata_path)
        with open(metadata_path, "r") as f:
            raw_metadata = json.load(f)

    metadata: ConversionMetadata | None = (
        None if raw_metadata is None else ConversionMetadata.model_validate(raw_metadata, strict=True)
    )

    convert_urdf_to_mjcf(
        urdf_path=args.urdf_path,
        mjcf_path=args.output,
        metadata=metadata,
    )


if __name__ == "__main__":
    main()
