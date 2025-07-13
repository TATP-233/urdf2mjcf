
import trimesh
import collada
import logging
from pathlib import Path
logger = logging.getLogger(__name__)

def dae2obj(dae_path: str | Path, obj_path: str | Path):
    """Convert DAE file to OBJ format with proper material handling.
    
    Args:
        dae_path: Path to input DAE file
        obj_path: Path to output OBJ file
    """
    # Convert inputs to Path objects for consistent handling
    dae_path = Path(dae_path)
    obj_path = Path(obj_path)
    
    try:
        logger.info(f"Converting DAE to OBJ: {dae_path} -> {obj_path}")
        
        # collada.Collada and trimesh.load expect string paths
        dae = collada.Collada(str(dae_path))
        name2dae_goem = {}
        for geom in dae.geometries:
            name2dae_goem[geom.name.replace(".", "_")+"-mesh"] = geom

        mesh_data = trimesh.load(str(dae_path))
        mtl_path = obj_path.with_suffix('.mtl')
        mtl_name = mtl_path.name  # Just the filename for export
        
        logger.info(f"Loaded DAE file with {len(name2dae_goem)} geometries")
        
    except Exception as e:
        logger.error(f"Failed to load DAE file {dae_path}: {e}")
        raise

    if isinstance(mesh_data, trimesh.Scene):
        # 收集所有唯一的材质
        unique_materials = {}
        geom_to_material = {}
        
        for name, geom in mesh_data.geometry.items():
            # 获取材质对象并提取其名称
            material_obj = name2dae_goem[name].primitives[0].material
            material_name = material_obj.id if hasattr(material_obj, 'id') else str(material_obj)
            
            # 记录几何体到材质的映射
            geom_to_material[name] = material_name
            
            # 收集唯一材质
            if material_name not in unique_materials:
                unique_materials[material_name] = material_obj
            
            # 设置材质名称
            geom.visual.material.name = material_name

        # 先导出，然后分析trimesh分配的材质名称
        mesh_data.export(str(obj_path), mtl_name=mtl_name)
        
        # 从导出的文件中分析trimesh的材质分配
        material_mapping = {}
        if mtl_path.exists():
            with open(str(mtl_path), 'r') as f:
                mtl_content = f.read()
            
            # 从OBJ文件中分析每个几何体使用的材质
            if obj_path.exists():
                with open(str(obj_path), 'r') as f:
                    obj_content = f.read()
                
                # 解析OBJ文件找到几何体名称和使用的材质
                lines = obj_content.split('\n')
                current_geom = None
                for line in lines:
                    if line.startswith('o '):
                        current_geom = line[2:].strip()
                    elif line.startswith('usemtl ') and current_geom:
                        trimesh_material = line[7:].strip()
                        if current_geom in geom_to_material and trimesh_material not in material_mapping:
                            actual_material = geom_to_material[current_geom]
                            material_mapping[trimesh_material] = actual_material
            
            # 修改 MTL 文件中的材质名称
            for old_name, new_name in material_mapping.items():
                mtl_content = mtl_content.replace(f"newmtl {old_name}", f"newmtl {new_name}")
            
            with open(str(mtl_path), 'w') as f:
                f.write(mtl_content)
        
        # 修改 OBJ 文件中的材质引用
        if obj_path.exists():
            with open(str(obj_path), 'r') as f:
                obj_content = f.read()
            
            for old_name, new_name in material_mapping.items():
                obj_content = obj_content.replace(f"usemtl {old_name}", f"usemtl {new_name}")
            
            with open(str(obj_path), 'w') as f:
                f.write(obj_content)
    else:
        mesh_data.export(str(obj_path), mtl_name=mtl_name)
    
    logger.info(f"Successfully converted DAE to OBJ: {obj_path}")

