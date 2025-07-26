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

    # <light pos="0 0 2." dir="0 0 -1" directional="true"/>
    _ = ET.SubElement(
        worldbody,
        "light",
        attrib={"pos": "0 0 2.", "dir": "0 0 -1", "directional": "true"},
    )

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
