"""Materials and MTL processing utilities."""

import logging
import re
import shutil
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


def process_obj_mtl_materials(obj_file: Path, target_dir: Path) -> dict[str, Material]:
    """Process MTL materials from OBJ file and copy required files.
    
    This function also splits the OBJ file by material groups, similar to obj2mjcf,
    creating separate mesh files for each material.
    
    Args:
        obj_file: Path to the OBJ file
        target_dir: Target directory for copied assets
        
    Returns:
        Dictionary mapping material names to Material objects
    """
    materials = {}
    
    if not obj_file.exists():
        return materials
        
    # Check if the OBJ file references an MTL file
    try:
        with obj_file.open("r") as f:
            mtl_name = parse_mtl_name(f.readlines())
    except Exception as e:
        logger.warning(f"Failed to read OBJ file {obj_file}: {e}")
        return materials
        
    if mtl_name is None:
        return materials
        
    # Make sure the MTL file exists
    mtl_file = obj_file.parent / mtl_name
    if not mtl_file.exists():
        logger.warning(f"MTL file {mtl_file} referenced in {obj_file} does not exist")
        return materials
        
    logger.info(f"Found MTL file: {mtl_file}")
    
    try:
        # Parse the MTL file into separate materials
        with open(mtl_file, "r") as f:
            lines = f.readlines()
            
        # Remove comments and empty lines
        lines = [line for line in lines if not line.startswith("#")]
        lines = [line for line in lines if line.strip()]
        lines = [line.strip() for line in lines]
        
        # Split at each new material definition
        sub_mtls = []
        for line in lines:
            if line.startswith("newmtl"):
                sub_mtls.append([])
            if sub_mtls:  # Only append if we have started a material
                sub_mtls[-1].append(line)
                
        # Process each material
        for sub_mtl in sub_mtls:
            if sub_mtl:  # Make sure the material has content
                material = Material.from_string(sub_mtl)
                material.name = f"{obj_file.stem}_{material.name}" 
                materials[material.name] = material
                logger.info(f"Found material: {material.name}")
                
                # Handle texture files
                if material.map_Kd is not None:
                    texture_path = Path(material.map_Kd)
                    src_texture = obj_file.parent / texture_path
                    if src_texture.exists():
                        dst_texture = target_dir / texture_path.name
                        dst_texture.parent.mkdir(parents=True, exist_ok=True)
                        shutil.copy2(src_texture, dst_texture)
                        # Update the material to use the copied texture
                        material.map_Kd = texture_path.name
                        logger.info(f"Copied texture: {texture_path.name}")
                    else:
                        logger.warning(f"Texture file {src_texture} does not exist")
        
        # Now process the OBJ file to split by materials (like obj2mjcf does)
        try:
            import trimesh
            
            # Load the OBJ file with material grouping
            mesh = trimesh.load(
                obj_file,
                split_object=True,
                group_material=True,  # Key parameter: group by material
                process=False,
                maintain_order=False,
            )
            
            # Create target directory in the same directory as the original OBJ file
            obj_target_dir = obj_file.parent / obj_file.stem
            obj_target_dir.mkdir(parents=True, exist_ok=True)
            
            if isinstance(mesh, trimesh.base.Trimesh):
                # Single mesh, just copy it
                target_mesh = obj_target_dir / f"{obj_file.stem}.obj"
                shutil.copy2(obj_file, target_mesh)
                logger.info(f"Copied single mesh: {target_mesh.name}")
            else:
                # Multiple submeshes, save each one separately
                logger.info(f"Splitting OBJ into {len(mesh.geometry)} submeshes by material")
                for i, (material_name, geom) in enumerate(mesh.geometry.items()):
                    submesh_name = obj_target_dir / f"{obj_file.stem}_{i}.obj"
                    geom.export(submesh_name.as_posix(), include_texture=True, header=None)
                    logger.info(f"Saved submesh: {submesh_name.name} (material: {material_name})")
                    
        except ImportError:
            logger.warning("trimesh not available, cannot split OBJ by materials")
        except Exception as e:
            logger.warning(f"Failed to split OBJ file {obj_file} by materials: {e}")
                        
    except Exception as e:
        logger.error(f"Failed to process MTL file {mtl_file}: {e}")
        
    return materials 