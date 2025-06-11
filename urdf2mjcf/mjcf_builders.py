"""MJCF XML element builders and utilities."""

import logging
import xml.etree.ElementTree as ET
from pathlib import Path

from urdf2mjcf.model import ActuatorMetadata, ConversionMetadata, JointMetadata
from urdf2mjcf.materials import Material

logger = logging.getLogger(__name__)

ROBOT_CLASS = "robot"


def add_compiler(root: ET.Element) -> None:
    """Add a compiler element to the MJCF root.

    Args:
        root: The MJCF root element.
    """
    attrib = {
        "angle": "radian",
        "meshdir": ".",
        # "eulerseq": "zyx",
        # "autolimits": "true",
    }

    element = ET.Element("compiler", attrib=attrib)
    existing_element = root.find("compiler")
    if isinstance(existing_element, ET.Element):
        root.remove(existing_element)
    root.insert(0, element)


def add_default(
    root: ET.Element,
    metadata: ConversionMetadata,
    joint_metadata: dict[str, JointMetadata] | None = None,
    actuator_metadata: dict[str, ActuatorMetadata] | None = None,
) -> None:
    """Add default settings with hierarchical structure for robot components."""
    default = ET.Element("default")

    if joint_metadata is None:
        raise ValueError("Missing joint metadata")
    if actuator_metadata is None:
        raise ValueError("Missing actuator metadata")

    # Main robot class defaults
    robot_default = ET.SubElement(default, "default", attrib={"class": ROBOT_CLASS})

    # Get the set of actuator types to make the classes at the top of the mjcf
    actuator_types = set()
    for current_joint_name, current_joint_metadata in joint_metadata.items():
        if current_joint_metadata is None:
            raise ValueError(f"Missing metadata for joint: {current_joint_name}")
        # More flexible type checking - check if it has the required attributes
        if not hasattr(current_joint_metadata, 'actuator_type'):
            raise ValueError(f"Metadata for joint {current_joint_name} does not have actuator_type attribute")
        actuator_types.add(current_joint_metadata.actuator_type)
        logger.info("Joint %s uses actuator type: %s", current_joint_name, current_joint_metadata.actuator_type)
    logger.info("Found %d actuator types in metadata: %s", len(actuator_types), actuator_types)

    # Create default classes for each actuator type
    for actuator_type in actuator_types:
        if actuator_type is None:
            raise ValueError(f"Actuator type: {actuator_type} cannot be None")

        sub_default = ET.SubElement(robot_default, "default", attrib={"class": str(actuator_type)})

        joint_attrib = {}
        motor_attrib = {}
        if actuator_type not in actuator_metadata:
            raise ValueError(f"Missing actuator type metadata for {actuator_type}")

        actuator_data = actuator_metadata[str(actuator_type)]
        if actuator_data.armature is not None:
            joint_attrib["armature"] = str(actuator_data.armature)
        if actuator_data.frictionloss is not None:
            joint_attrib["frictionloss"] = str(actuator_data.frictionloss)
        if actuator_data.damping is not None:
            joint_attrib["damping"] = str(actuator_data.damping)
        if actuator_data.max_torque is not None:
            joint_attrib["actuatorfrcrange"] = f"-{actuator_data.max_torque} {actuator_data.max_torque}"
            motor_attrib["ctrlrange"] = f"-{actuator_data.max_torque} {actuator_data.max_torque}"

        ET.SubElement(sub_default, "joint", attrib=joint_attrib)
        ET.SubElement(sub_default, "motor", attrib=motor_attrib)
        logger.info(
            "Added actuator class for %s: with joint attrib %s and motor attrib %s",
            actuator_type,
            joint_attrib,
            motor_attrib,
        )

    # Visual geometry class
    visual_default = ET.SubElement(
        robot_default,
        "default",
        attrib={"class": "visual"},
    )
    ET.SubElement(
        visual_default,
        "geom",
        attrib={
            "material": "visualgeom",
            "contype": "0",
            "conaffinity": "0",
            "group": "2",
        },
    )

    # Collision geometry class
    collision_default = ET.SubElement(
        robot_default,
        "default",
        attrib={"class": "collision"},
    )
    ET.SubElement(
        collision_default,
        "geom",
        attrib={
            "material": "collision_material",
            "condim": str(metadata.collision_params.condim),
            "contype": str(metadata.collision_params.contype),
            "conaffinity": str(metadata.collision_params.conaffinity),
            "priority": str(metadata.collision_params.priority),
            "group": "3",
            "solref": " ".join(f"{x:.6g}" for x in metadata.collision_params.solref),
            "friction": " ".join(f"{x:.6g}" for x in metadata.collision_params.friction),
        },
    )
    ET.SubElement(
        collision_default,
        "equality",
        attrib={
            "solimp": " ".join(f"{x:.6g}" for x in metadata.collision_params.solimp),
            "solref": " ".join(f"{x:.6g}" for x in metadata.collision_params.solref),
        },
    )

    # Add maxhullvert for efficient collising handling.
    if metadata.maxhullvert is not None:
        ET.SubElement(default, "mesh", attrib={"maxhullvert": str(metadata.maxhullvert)})

    # Replace existing default element if present
    existing_element = root.find("default")
    if isinstance(existing_element, ET.Element):
        root.remove(existing_element)
    root.insert(0, default)


def add_contact(root: ET.Element, robot: ET.Element) -> None:
    """Add a contact element to the MJCF root.

    For each pair of adjacent links that each have collision elements, we need
    to add an exclude tag to the contact element to make sure the links do not
    collide with each other.

    Args:
        root: The MJCF root element.
        robot: The URDF robot element.
    """
    links_with_collision: dict[str, ET.Element] = {}
    for link in robot.findall("link"):
        if link.find("collision") is not None and (name := link.attrib.get("name")) is not None:
            links_with_collision[name] = link

    contact: ET.Element | None = None
    for joint in robot.findall("joint"):
        parent_link = joint.find("parent")
        child_link = joint.find("child")
        if (
            parent_link is None
            or child_link is None
            or (parent_name := parent_link.attrib.get("link")) is None
            or (child_name := child_link.attrib.get("link")) is None
        ):
            continue

        if parent_name in links_with_collision and child_name in links_with_collision:
            if contact is None:
                contact = ET.SubElement(root, "contact")

            ET.SubElement(
                contact,
                "exclude",
                attrib={
                    "body1": parent_name,
                    "body2": child_name,
                },
            )


def add_weld_constraints(root: ET.Element, metadata: ConversionMetadata) -> None:
    """Add weld constraints to the MJCF root.

    Args:
        root: The MJCF root element.
        metadata: The conversion metadata containing weld constraints.
    """
    if not metadata.weld_constraints:
        return

    equality = ET.SubElement(root, "equality")
    for weld in metadata.weld_constraints:
        ET.SubElement(
            equality,
            "weld",
            attrib={
                "body1": weld.body1,
                "body2": weld.body2,
                "solimp": " ".join(f"{x:.6g}" for x in weld.solimp),
                "solref": " ".join(f"{x:.6g}" for x in weld.solref),
            },
        )


def add_option(root: ET.Element) -> None:
    """Add an option element to the MJCF root.

    Args:
        root: The MJCF root element.
    """
    # ET.SubElement(
    #     root,
    #     "option",
    #     attrib={
    #         "integrator": "implicitfast",
    #         "cone": "elliptic",
    #         "impratio": "100",
    #     },
    # )


def add_visual(root: ET.Element) -> None:
    """Add a visual element to the MJCF root.

    Args:
        root: The MJCF root element.
    """
    # visual = ET.SubElement(root, "visual")
    # ET.SubElement(
    #     visual,
    #     "global",
    #     attrib={
    #         "ellipsoidinertia": "true",
    #     },
    # )


def add_assets(root: ET.Element, materials: dict[str, str], mtl_materials: dict[str, Material] = None, visualize_collision_meshes: bool = True) -> None:
    """Add texture and material assets to the MJCF root.

    Args:
        root: The MJCF root element.
        materials: Dictionary mapping material names to RGBA color strings.
        mtl_materials: Dictionary mapping material names to MTL Material objects.
        visualize_collision_meshes: If True, add a visual element for collision meshes.
    """
    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")

    # Add MTL materials first (they take priority)
    if mtl_materials:
        for material in mtl_materials.values():
            material_attrib = {
                "name": material.name,
                "specular": material.mjcf_specular(),
                "shininess": material.mjcf_shininess(),
            }
            
            if material.map_Kd is not None:
                # Create texture asset for diffuse map
                texture_name = Path(material.map_Kd).stem
                ET.SubElement(
                    asset,
                    "texture",
                    attrib={
                        "type": "2d",
                        "name": texture_name,
                        "file": material.map_Kd,
                    },
                )
                # Reference the texture in the material
                material_attrib["texture"] = texture_name
            else:
                # Use RGBA if no texture
                material_attrib["rgba"] = material.mjcf_rgba()
                
            ET.SubElement(asset, "material", attrib=material_attrib)
            logger.info(f"Added MTL material: {material.name}")

    # Add materials from URDF (skip if already added from MTL)
    for name, rgba in materials.items():
        if name == "default_material":
            continue
        if mtl_materials and name in mtl_materials:
            continue  # Skip if already added from MTL
        ET.SubElement(
            asset,
            "material",
            attrib={
                "name": name,
                "rgba": rgba,
            },
        )

    # Add default material for visual elements without materials
    ET.SubElement(
        asset,
        "material",
        attrib={
            "name": "default_material",
            "rgba": "0.7 0.7 0.7 1",
        },
    )

    # Add blue transparent material for collision geometries
    ET.SubElement(
        asset,
        "material",
        attrib={
            "name": "collision_material",
            "rgba": "1.0 0.28 0.1 0.9" if visualize_collision_meshes else "0.0 0.0 0.0 0.0",
        },
    )

