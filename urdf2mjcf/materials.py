"""Materials and MTL processing utilities."""

import shutil
import logging
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

logger = logging.getLogger(__name__)

# MTL fields relevant to MuJoCo
MTL_FIELDS = (
    "Ka",   # Ambient color
    "Kd",   # Diffuse color
    "Ks",   # Specular color
    "d",    # Transparency (alpha)
    "Tr",   # 1 - transparency
    "Ns",   # Shininess
    "map_Kd",  # Diffuse texture map
)


@dataclass
class Material:
    """A convenience class for constructing MuJoCo materials from MTL files."""
    
    name: str
    Ka: Optional[str] = None
    Kd: Optional[str] = None
    Ks: Optional[str] = None
    d: Optional[str] = None
    Tr: Optional[str] = None
    Ns: Optional[str] = None
    map_Kd: Optional[str] = None

    @staticmethod
    def from_string(lines: Sequence[str]) -> "Material":
        """Construct a Material object from a string."""
        attrs = {"name": lines[0].split(" ")[1].strip()}
        for line in lines[1:]:
            for attr in MTL_FIELDS:
                if line.startswith(attr):
                    elems = line.split(" ")[1:]
                    elems = [elem for elem in elems if elem != ""]
                    attrs[attr] = " ".join(elems)
                    break
        return Material(**attrs)

    def mjcf_rgba(self) -> str:
        """Convert material properties to MJCF RGBA string."""
        Kd = self.Kd or "1.0 1.0 1.0"
        if self.d is not None:  # alpha
            alpha = self.d
        elif self.Tr is not None:  # 1 - alpha
            alpha = str(1.0 - float(self.Tr))
        else:
            alpha = "1.0"
        return f"{Kd} {alpha}"

    def mjcf_shininess(self) -> str:
        """Convert shininess value to MJCF format."""
        if self.Ns is not None:
            # Normalize Ns value to [0, 1]. Ns values normally range from 0 to 1000.
            Ns = float(self.Ns) / 1_000
        else:
            Ns = 0.5
        return f"{Ns}"

    def mjcf_specular(self) -> str:
        """Convert specular value to MJCF format."""
        if self.Ks is not None:
            # Take the average of the specular RGB values.
            Ks = sum(list(map(float, self.Ks.split(" ")))) / 3
        else:
            Ks = 0.5
        return f"{Ks}"


def parse_mtl_name(lines: Sequence[str]) -> Optional[str]:
    """Parse MTL file name from OBJ file lines."""
    mtl_regex = re.compile(r"^mtllib\s+(.+?\.mtl)(?:\s*#.*)?\s*\n?$")
    for line in lines:
        match = mtl_regex.match(line)
        if match is not None:
            name = match.group(1)
            return name
    return None


def get_obj_material_info(obj_file: Path) -> tuple[bool, str | None]:
    """Get material information from an OBJ file.
    
    Args:
        obj_file: Path to the OBJ file
        
    Returns:
        Tuple of (has_single_material, material_name)
        - has_single_material: True if the OBJ has exactly one material
        - material_name: The material name if single material, None otherwise
    """
    if not obj_file.exists() or not obj_file.suffix.lower() == '.obj':
        return False, None
        
    try:
        with obj_file.open("r") as f:
            lines = f.readlines()
            
        # Check for MTL file reference
        mtl_name = parse_mtl_name(lines)
        if mtl_name is None:
            return False, None
            
        # Check if MTL file exists
        mtl_file = obj_file.parent / mtl_name
        if not mtl_file.exists():
            return False, None
            
        # Parse MTL file to count materials
        with open(mtl_file, "r") as f:
            mtl_lines = f.readlines()
            
        # Count material definitions
        material_names = []
        for line in mtl_lines:
            line = line.strip()
            if line.startswith("newmtl "):
                material_name = line.split()[1]
                material_names.append(material_name)
                
        # Return info about single material
        if len(material_names) == 1:
            return True, material_names[0]
        else:
            return False, None
            
    except Exception as e:
        logger.warning(f"Failed to analyze OBJ material info for {obj_file}: {e}")
        return False, None

def copy_obj_with_mtl(obj_source: Path, obj_target: Path) -> None:
    """Copy OBJ file and its associated MTL file if it exists.
    
    Args:
        obj_source: Source OBJ file path
        obj_target: Target OBJ file path
    """
    # Copy the OBJ file
    obj_target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(obj_source, obj_target)
    
    # Check for MTL file and copy it if it exists
    try:
        with open(obj_source, 'r') as f:
            lines = f.readlines()
        
        # Look for mtllib directive
        for line in lines:
            if line.strip().startswith('mtllib '):
                mtl_filename = line.strip().split()[1]
                mtl_source = obj_source.parent / mtl_filename
                mtl_target = obj_target.parent / mtl_filename
                
                if mtl_source.exists():
                    shutil.copy2(mtl_source, mtl_target)
                    logger.info(f"Copied MTL file: {mtl_source} -> {mtl_target}")
                break
    except Exception as e:
        logger.warning(f"Failed to check/copy MTL file for {obj_source}: {e}")
