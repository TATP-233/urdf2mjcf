"""Defines a post-processing function that adds an appendix to the Mujoco model.

This script adds an appendix to the MJCF file by merging elements from an appendix XML file.
"""

import argparse
import logging
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Set

from urdf2mjcf.utils import save_xml

logger = logging.getLogger(__name__)


def find_all_joints(element: ET.Element) -> Set[str]:
    """Recursively find all joint names in the element tree.
    
    Args:
        element: The XML element to search in.
        
    Returns:
        Set of joint names found.
    """
    joints = set()
    
    # Check if current element is a joint
    if element.tag == "joint" and "name" in element.attrib:
        joints.add(element.attrib["name"])
    
    # Recursively search children
    for child in element:
        joints.update(find_all_joints(child))
    
    return joints


def find_all_bodies(element: ET.Element) -> Set[str]:
    """Recursively find all body names in the element tree.
    
    Args:
        element: The XML element to search in.
        
    Returns:
        Set of body names found.
    """
    bodies = set()
    
    # Check if current element is a body
    if element.tag == "body" and "name" in element.attrib:
        bodies.add(element.attrib["name"])
    
    # Recursively search children
    for child in element:
        bodies.update(find_all_bodies(child))
    
    return bodies


def find_all_sites(element: ET.Element) -> Set[str]:
    """Recursively find all site names in the element tree.
    
    Args:
        element: The XML element to search in.
        
    Returns:
        Set of site names found.
    """
    sites = set()
    
    # Check if current element is a site
    if element.tag == "site" and "name" in element.attrib:
        sites.add(element.attrib["name"])
    
    # Recursively search children
    for child in element:
        sites.update(find_all_sites(child))
    
    return sites


def validate_equality_constraints(equality_element: ET.Element, available_joints: Set[str]) -> bool:
    """Validate equality constraints by checking if referenced joints exist.
    
    Args:
        equality_element: The equality XML element to validate.
        available_joints: Set of available joint names.
        
    Returns:
        True if all constraints are valid, False otherwise.
    """
    valid_constraints = []
    invalid_constraints = []
    
    for joint_elem in equality_element.findall("joint"):
        joint1 = joint_elem.get("joint1")
        joint2 = joint_elem.get("joint2")
        
        if joint1 and joint1 not in available_joints:
            logger.warning(f"Joint '{joint1}' not found in worldbody")
            invalid_constraints.append((joint1, joint2))
            continue
        
        if joint2 and joint2 not in available_joints:
            logger.warning(f"Joint '{joint2}' not found in worldbody")
            invalid_constraints.append((joint1, joint2))
            continue
            
        valid_constraints.append((joint1, joint2))
    
    if invalid_constraints:
        logger.info(f"Skipping: {len(invalid_constraints)} invalid joint constraints")
        for joint1, joint2 in invalid_constraints:
            logger.debug(f"  Invalid constraint: {joint1} <-> {joint2}")
    
    if valid_constraints:
        logger.info(f"Adding {len(valid_constraints)} valid joint constraints")
        for joint1, joint2 in valid_constraints:
            logger.debug(f"  Valid constraint: {joint1} <-> {joint2}")
    
    return len(valid_constraints) > 0


def add_filtered_contact_constraints(mjcf_root: ET.Element, contact_element: ET.Element, available_bodies: Set[str]) -> None:
    """Add only valid contact constraints to the MJCF file.
    
    Args:
        mjcf_root: The root element of the MJCF file.
        contact_element: The contact element containing constraints.
        available_bodies: Set of available body names.
    """
    # Find or create contact element in MJCF
    existing_contact = mjcf_root.find("contact")
    if existing_contact is None:
        existing_contact = ET.SubElement(mjcf_root, "contact")
        logger.info("Created new contact element")
    
    # Add only valid constraints
    valid_count = 0
    total_count = 0
    for exclude_elem in contact_element.findall("exclude"):
        total_count += 1
        body1 = exclude_elem.get("body1")
        body2 = exclude_elem.get("body2")
        
        # Check if both bodies exist
        if (body1 and body1 in available_bodies and 
            body2 and body2 in available_bodies):
            existing_contact.append(exclude_elem)
            valid_count += 1
            logger.debug(f"Added contact exclusion: {body1} <-> {body2}")
        else:
            logger.info(f"Skipping <contact>: {body1} <-> {body2} (bodies not found)")
    
    logger.info(f"Added {valid_count}/{total_count} valid contact exclusions")


def add_filtered_sensor_constraints(mjcf_root: ET.Element, sensor_element: ET.Element, available_joints: Set[str], available_sites: Set[str]) -> None:
    """Add only valid sensor constraints to the MJCF file.
    
    Args:
        mjcf_root: The root element of the MJCF file.
        sensor_element: The sensor element containing constraints.
        available_joints: Set of available joint names.
        available_sites: Set of available site names.
    """
    # Find or create sensor element in MJCF
    existing_sensor = mjcf_root.find("sensor")
    if existing_sensor is None:
        existing_sensor = ET.SubElement(mjcf_root, "sensor")
        logger.info("Created new sensor element")
    
    # Add only valid constraints
    valid_count = 0
    total_count = 0
    for sensor_elem in sensor_element:
        total_count += 1
        joint = sensor_elem.get("joint")
        site = sensor_elem.get("site")
        sensor_name = sensor_elem.get("name", "unknown")
        
        # Check if referenced joint/site exists
        valid = True
        if joint and joint not in available_joints:
            logger.info(f"Skipping <sensor>: '{sensor_name}' references non-existent joint '{joint}'")
            valid = False
        
        if site and site not in available_sites:
            logger.info(f"Skipping <sensor>: '{sensor_name}' references non-existent site '{site}'")
            valid = False
        
        if valid:
            existing_sensor.append(sensor_elem)
            valid_count += 1
            logger.debug(f"Added sensor: {sensor_name}")
    
    logger.info(f"Added {valid_count}/{total_count} valid sensors")


def validate_contact_constraints(contact_element: ET.Element, available_bodies: Set[str]) -> bool:
    """Validate contact constraints by checking if referenced bodies exist.
    
    Args:
        contact_element: The contact XML element to validate.
        available_bodies: Set of available body names.
        
    Returns:
        True if any constraints are valid, False if none are valid.
    """
    valid_count = 0
    for exclude_elem in contact_element.findall("exclude"):
        body1 = exclude_elem.get("body1")
        body2 = exclude_elem.get("body2")
        
        if (body1 and body1 in available_bodies and 
            body2 and body2 in available_bodies):
            valid_count += 1
    
    return valid_count > 0


def validate_sensor_constraints(sensor_element: ET.Element, available_joints: Set[str], available_sites: Set[str]) -> bool:
    """Validate sensor constraints by checking if referenced joints and sites exist.
    
    Args:
        sensor_element: The sensor XML element to validate.
        available_joints: Set of available joint names.
        available_sites: Set of available site names.
        
    Returns:
        True if any constraints are valid, False if none are valid.
    """
    valid_count = 0
    for sensor_elem in sensor_element:
        joint = sensor_elem.get("joint")
        site = sensor_elem.get("site")
        
        valid = True
        if joint and joint not in available_joints:
            valid = False
        
        if site and site not in available_sites:
            valid = False
        
        if valid:
            valid_count += 1
    
    return valid_count > 0


def merge_elements(parent: ET.Element, new_element: ET.Element) -> None:
    """Merge new element into parent element.
    
    Args:
        parent: The parent element to merge into.
        new_element: The new element to merge.
    """
    existing_element = parent.find(new_element.tag)
    
    if existing_element is None:
        # Element doesn't exist, add it
        parent.append(new_element)
        logger.info(f"Added new element: {new_element.tag}")
    else:
        # Element exists, merge children
        for attr in new_element.attrib:
            print(attr, new_element.attrib[attr])
            existing_element.attrib[attr] = new_element.attrib[attr]
        for child in new_element:
            existing_element.append(child)
        logger.info(f"Merged children into existing element: {new_element.tag}")

def add_filtered_equality_constraints(mjcf_root: ET.Element, equality_element: ET.Element, available_joints: Set[str]) -> None:
    """Add only valid equality constraints to the MJCF file.
    
    Args:
        mjcf_root: The root element of the MJCF file.
        equality_element: The equality element containing constraints.
        available_joints: Set of available joint names.
    """
    # Find or create equality element in MJCF
    existing_equality = mjcf_root.find("equality")
    if existing_equality is None:
        existing_equality = ET.SubElement(mjcf_root, "equality")
        logger.info("Created new equality element")
    
    # Add only valid constraints
    valid_count = 0
    for joint_elem in equality_element.findall("joint"):
        joint1 = joint_elem.get("joint1")
        joint2 = joint_elem.get("joint2")
        
        # Check if both joints exist
        if (joint1 and joint1 in available_joints and 
            joint2 and joint2 in available_joints):
            existing_equality.append(joint_elem)
            valid_count += 1
            logger.debug(f"Added constraint: {joint1} <-> {joint2}")
        else:
            logger.info(f"Skipping <equality>: {joint1} <-> {joint2} (joints not found)")
    
    logger.info(f"Added {valid_count} valid equality constraints")


def add_appendix(mjcf_path: str | Path, appendix_path: str | Path) -> None:
    """Add an appendix to the MJCF file.

    Args:
        mjcf_path: The path to the MJCF file to process.
        appendix_path: The path to the appendix file.
    """
    # Load main MJCF file
    mjcf_tree = ET.parse(mjcf_path)
    mjcf_root = mjcf_tree.getroot()
    
    # Load appendix file - handle multiple root elements
    try:
        # Read the file content and wrap it with a temporary root element
        with open(appendix_path, 'r', encoding='utf-8') as f:
            content = f.read().strip()
        
        # Wrap content with a temporary root element
        wrapped_content = f"<appendix_root>{content}</appendix_root>"
        
        # Parse the wrapped content
        appendix_root = ET.fromstring(wrapped_content)
        elements_to_process = list(appendix_root)
        
    except ET.ParseError as e:
        logger.error(f"Failed to parse appendix file {appendix_path}: {e}")
        return
    except FileNotFoundError:
        logger.error(f"Appendix file not found: {appendix_path}")
        return
    except Exception as e:
        logger.error(f"Error reading appendix file {appendix_path}: {e}")
        return
    
    # Find worldbody in MJCF
    worldbody = mjcf_root.find("worldbody")
    if worldbody is None:
        logger.error("No worldbody element found in MJCF file")
        return
    
    # Collect available joints, bodies, and sites
    available_joints = find_all_joints(worldbody)
    available_bodies = find_all_bodies(worldbody)
    available_sites = find_all_sites(worldbody)
    
    logger.info(f"Found {len(available_joints)} joints, {len(available_bodies)} bodies, {len(available_sites)} sites")
    
    # Process each element in appendix
    for element in elements_to_process:
        if element.tag == "equality":
            # Handle equality constraints specially to filter invalid ones
            add_filtered_equality_constraints(mjcf_root, element, available_joints)
        elif element.tag == "contact":
            should_add = validate_contact_constraints(element, available_bodies)
            if should_add:
                add_filtered_contact_constraints(mjcf_root, element, available_bodies)
            else:
                logger.info(f"Skipping: Skipping contact element due to validation failure")
        elif element.tag == "sensor":
            should_add = validate_sensor_constraints(element, available_joints, available_sites)
            if should_add:
                add_filtered_sensor_constraints(mjcf_root, element, available_joints, available_sites)
            else:
                logger.info(f"Skipping: Skipping sensor element due to validation failure")
        else:
            # For other elements, just merge them
            merge_elements(mjcf_root, element)
    
    # Save the updated MJCF file
    save_xml(mjcf_path, mjcf_tree)
    logger.info(f"Successfully added appendix to {mjcf_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Adds an appendix to the MJCF model.")
    parser.add_argument("mjcf_path", type=Path, help="Path to the MJCF file.")
    parser.add_argument("appendix_path", type=Path, help="Path to the appendix XML file.")
    args = parser.parse_args()

    add_appendix(args.mjcf_path, args.appendix_path)


if __name__ == "__main__":
    # python -m urdf2mjcf.postprocess.add_appendix
    main()
