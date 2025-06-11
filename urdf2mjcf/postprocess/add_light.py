"""Defines a post-processing function that adds lights to the Mujoco model.

This script adds default lighting to the MJCF file.
"""

import argparse
import logging
import xml.etree.ElementTree as ET
from pathlib import Path

from urdf2mjcf.utils import save_xml

logger = logging.getLogger(__name__)


def add_default_lights(root: ET.Element) -> None:
    """Add default lights to the MJCF file.

    Args:
        root: The root element of the MJCF file.
    """
    # Find the worldbody element
    worldbody = root.find("worldbody")
    if worldbody is None:
        logger.warning("No worldbody element found in the MJCF file.")
        return

    # Check if lights already exist and remove them to update
    existing_spotlight = worldbody.find(".//light[@name='spotlight']")
    existing_top = worldbody.find(".//light[@name='top']")
    
    if existing_spotlight is not None:
        worldbody.remove(existing_spotlight)
        logger.info("Removed existing spotlight to update.")
    
    if existing_top is not None:
        worldbody.remove(existing_top)
        logger.info("Removed existing top light to update.")

    # Add spotlight
    spotlight = ET.Element("light")
    spotlight.attrib["name"] = "spotlight"
    spotlight.attrib["diffuse"] = ".8 .8 .8"
    spotlight.attrib["specular"] = "0.3 0.3 0.3"
    spotlight.attrib["pos"] = "0 -6 4"
    spotlight.attrib["cutoff"] = "30"
    worldbody.append(spotlight)
    logger.info("Added spotlight to the MJCF file.")

    # Add top light
    top_light = ET.Element("light")
    top_light.attrib["name"] = "top"
    top_light.attrib["pos"] = "0 0 2"
    worldbody.append(top_light)
    logger.info("Added top light to the MJCF file.")

def add_light(mjcf_path: str | Path) -> None:
    """Add default lights to the MJCF file.

    Args:
        mjcf_path: The path to the MJCF file to process.
    """
    tree = ET.parse(mjcf_path)
    root = tree.getroot()
    add_default_lights(root)
    save_xml(mjcf_path, tree)


def main() -> None:
    parser = argparse.ArgumentParser(description="Adds default lights to the MJCF model.")
    parser.add_argument("mjcf_path", type=Path, help="Path to the MJCF file.")
    args = parser.parse_args()

    add_light(args.mjcf_path)


if __name__ == "__main__":
    # python -m urdf2mjcf.postprocess.add_light
    main()
