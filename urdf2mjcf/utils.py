"""Defines utility functions."""

from __future__ import annotations

import io
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from xml.dom import minidom

def sort_body_elements(element: ET.Element) -> None:
    """Sort child elements in body elements according to MJCF conventions.
    
    The order should be: inertial, joint, geom (all geoms), body (all bodies).

    Args:
        element: The element to process (recursively processes all body elements)
    """
    if element.tag == "body":
        # Define the desired order
        order = ["inertial", "joint", "geom", "body"]
        
        # Group children by tag
        children_by_tag = {}
        for child in element:
            tag = child.tag
            if tag not in children_by_tag:
                children_by_tag[tag] = []
            children_by_tag[tag].append(child)
        
        # Remove all child elements (but keep attributes)
        for child in list(element):
            element.remove(child)
        
        # Re-add children in the desired order
        for tag in order:
            if tag in children_by_tag:
                for child in children_by_tag[tag]:
                    element.append(child)
        
        # Add any remaining children that weren't in our order list
        for tag, children in children_by_tag.items():
            if tag not in order:
                for child in children:
                    element.append(child)
    
    # Recursively process all child elements
    for child in element:
        sort_body_elements(child)


def save_xml(path: str | Path | io.StringIO, tree: ET.ElementTree[ET.Element] | ET.Element) -> None:
    """Save XML to file with pretty formatting and sorted body elements."""
    element: ET.Element
    if isinstance(tree, ET.ElementTree):
        root = tree.getroot()
        if root is None:
            raise ValueError("ElementTree has no root element")
        element = root
    else:
        element = tree

    # Sort body elements before saving
    sort_body_elements(element)

    xmlstr = minidom.parseString(ET.tostring(element)).toprettyxml(indent="  ")
    xmlstr = re.sub(r"\n\s*\n", "\n", xmlstr)

    # Add newlines between second-level nodes
    root = ET.fromstring(xmlstr)
    for child in root[:-1]:
        child.tail = "\n\n  "
    xmlstr = ET.tostring(root, encoding="unicode")

    if isinstance(path, io.StringIO):
        path.write(xmlstr)
    else:
        with open(path, "w") as f:
            f.write(xmlstr)
