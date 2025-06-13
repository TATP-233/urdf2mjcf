# urdf2mjcf

A powerful URDF to MJCF conversion tool with advanced mesh processing and robotics simulation optimization.

## ✨ Features

- 🔄 **Cross-platform**: Linux/Windows/macOS support, just like MuJoCo!
- 🔍 **Intelligent Package Resolution**: Automatic ROS workspace/package detection with nested package structure support
- 🎨 **Material Handling**: Automatic OBJ+MTL material import and processing
- 🎯 **Convex Decomposition**: Automatic collision mesh convex decomposition for improved simulation efficiency
- 🏗️ **Mesh Simplification**: Automatic high-polygon mesh simplification for performance optimization
- 📦 **Multi-format Support**: STL, OBJ and other mesh formats with automatic model file copying

## 🚀 Installation

### Requirements

- Python 3.8+
- MuJoCo 3.3.2+
- NumPy
- Trimesh
- lxml

### Installation Steps

1. Clone the repository:
```bash
git clone https://github.com/TATP-233/urdf2mjcf.git
cd urdf2mjcf
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

3. Install the package:
```bash
pip install -e .
```

## 📖 Usage

### Basic Conversion

```bash
python convert.py input.urdf --output output.xml
```

### Advanced Usage

```bash
# Using metadata file
python convert.py robot.urdf --output robot.xml --metadata-file metadata.json

# Specifying output directory
python convert.py /path/to/robot.urdf --output /path/to/output/robot.xml
```

### Python API

```python
from urdf2mjcf.convert import convert_urdf_to_mjcf
from pathlib import Path

# Basic conversion
convert_urdf_to_mjcf(
    urdf_path="path/to/robot.urdf",
    mjcf_path="path/to/robot.xml"
)

# Conversion with metadata
convert_urdf_to_mjcf(
    urdf_path="path/to/robot.urdf",
    mjcf_path="path/to/robot.xml",
    metadata_file="path/to/metadata.json"
)
```

## 🔧 Key Features

### 1. Intelligent Package Resolution

- **ROS Package Support**: Automatically locate mesh resources without adding `<mujoco>` tags in URDF or manually replacing `package://xxx`
- **Automatic Workspace Detection**: Search upward from URDF file location to find ROS workspace
- **Recursive Package Search**: Support nested structures like `ros_ws/src/xxx/package1`
- **Multi-strategy Compatibility**: Works even without rospkg installed
- **Environment Variable Support**: Automatically reads ROS_WORKSPACE and other environment variables

### 2. Convex Decomposition

```python
# Automatic collision mesh processing
# Input: models/xxx.stl
# Output: models/xxx_parts/xxx_part1.stl, xxx_part2.stl, ...
```

- High-quality convex decomposition using CoACD algorithm
- Automatic MJCF collision geom updates
- Preserves original directory structure
- Avoids duplicate processing of identical meshes

### 3. Mesh Optimization

- **Automatic Simplification**: Simplifies high-polygon meshes (>200k vertices, MuJoCo's maximum limit)
- **Intelligent Scaling**: Dynamic simplification ratios based on mesh complexity
- **Quality Preservation**: Uses quadric error metrics to maintain geometric features

### 4. Material Processing

- **OBJ+MTL Support**: Automatic processing of OBJ file MTL materials
- **Material Separation**: Support for separating mesh sub-objects by material
- **Color Mapping**: Automatic URDF material color mapping to MJCF
- **Texture Support**: Preserves original texture mapping

## 📁 Project Structure

```
urdf2mjcf/
├── urdf2mjcf/
│   ├── convert.py              # Main conversion module
│   ├── package_resolver.py     # Package path resolution
│   ├── materials.py           # Material processing
│   ├── geometry.py            # Geometric calculations
│   ├── mjcf_builders.py       # MJCF builders
│   └── postprocess/           # Post-processing modules
│       ├── convex_decomposition.py  # Convex decomposition
│       ├── check_shell.py           # Shell detection
│       ├── update_mesh.py           # Mesh updates
│       └── ...
├── models/                    # Example models
├── requirements.txt
└── README.md
```

## 🛠️ Post-processing Pipeline

The following post-processing steps are automatically executed during conversion:

1. **Ground and Lighting Addition**: Creates basic simulation environment
2. **Convex Decomposition**: Processes collision meshes
3. **Shell Detection**: Marks coplanar meshes
4. **Unit Conversion**: Supports angle unit conversion
5. **Constraint Processing**: Adds joint constraints and damping
6. **Redundancy Removal**: Cleans up duplicate elements

## 🤝 Acknowledgments

This project builds upon these excellent open-source projects:

- **[kscalelabs/urdf2mjcf](https://github.com/kscalelabs/urdf2mjcf)**: Core conversion framework
- **[kevinzakka/obj2mjcf](https://github.com/kevinzakka/obj2mjcf)**: OBJ file processing inspiration

Thanks to the original authors for their outstanding contributions!

## 📄 License

MIT License

## 🐛 Issue Reporting

For issues or suggestions, please submit an Issue on GitHub.

## 🔗 Related Links

- [MuJoCo Official Documentation](https://mujoco.readthedocs.io/)
- [URDF Specification](http://wiki.ros.org/urdf)
- [MJCF File Format](https://mujoco.readthedocs.io/en/latest/XMLreference.html)
