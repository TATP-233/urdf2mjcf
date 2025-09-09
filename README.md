# urdf2mjcf

URDF to MJCF conversion tool with support for STL, OBJ, DAE formats, automatic recognition of mimic tags in URDF, configurable joint actuators, and one-click generation of sim-ready MJCF files.

[中文文档](./README_zh.md)

<div style="display: flex; justify-content: center; gap: 20px;">
  <img src="./examples/agilex-piper/piper.png" alt="piper" style="width: 45%;" />
  <img src="./examples/realman-rm65/rm65.png" alt="rm65" style="width: 45%;" />
</div>

## 🚀 Installation

### Installation Steps

```bash
git clone https://github.com/TATP-233/urdf2mjcf.git
cd urdf2mjcf
pip install .
```

## 📖 Usage

### Basic Conversion

```bash
cd /path/to/your/robot-description/
urdf2mjcf input.urdf --output mjcf/output.xml
# Note: Do not place the generated .xml file in the same directory as the urdf, you can add mjcf/output.xml as shown above
```

### Command Line Arguments

```bash
urdf2mjcf <urdf_path> [options]
```

#### Required Arguments
- `urdf_path`: Path to the input URDF file

#### Optional Arguments
- `-o, --output`: Output MJCF file path (default: same as input file but with .mjcf extension)
- `-m, --metadata`: Path to JSON file containing conversion metadata (joint parameters and sensor configurations)
- `-dm, --default-metadata`: Default metadata JSON files, multiple files can be specified, later files override earlier settings
- `-am, --actuator-metadata`: Actuator metadata JSON files, multiple files can be specified, later files override earlier settings
- `-a, --appendix`: Appendix XML files, multiple files can be specified and applied in order
- `--collision-only`: Use collision geometry only without visual appearance for visual representation
- `--no-convex-decompose`: Disable mesh convex decomposition processing
- `--collision-type`: Collision type(mesh, convex decomposition, convex hull)
- `--log-level`: Logging level (default: INFO level)
- `--max-vertices`: Maximum number of vertices in the mesh (default: 200000)

#### Metadata Files Description
- **metadata**: Main conversion configuration file, contains height offset, angle units, whether to add floor, etc.
- **default-metadata**: Default joint parameter configuration, defines default properties for joints
- **actuator-metadata**: Actuator configuration, defines actuator types and parameters for each joint
- **appendix**: Additional XML content that will be directly added to the generated MJCF file

### Usage Examples

```bash
# agilex-piper robot
cd examples/agilex-piper
urdf2mjcf piper.urdf \
  -o mjcf/piper.xml \
  -m metadata/metadata.json \
  -am metadata/actuator.json \
  -dm metadata/default.json \
  -a metadata/appendix.xml
# View the generated model
python -m mujoco.viewer --mjcf=mjcf/piper.xml

# realman-rm65 robotic arm
cd examples/realman-rm65
urdf2mjcf rm65b_eg24c2_description.urdf \
  -o mjcf/rm65.xml \
  -m metadata/metadata.json \
  -am metadata/actuator.json \
  -dm metadata/default.json \
  -a metadata/appendix.xml
# View the generated model
python -m mujoco.viewer --mjcf=mjcf/rm65.xml
```

## 🤝 Acknowledgments

This project builds upon these excellent open-source projects:

- **[kscalelabs/urdf2mjcf](https://github.com/kscalelabs/urdf2mjcf)**: Core conversion framework
- **[kevinzakka/obj2mjcf](https://github.com/kevinzakka/obj2mjcf)**: OBJ file processing inspiration

Thanks to the original authors for their outstanding contributions!

## 📄 License

MIT License
