# urdf2mjcf

URDF到MJCF转换工具，支持stl、obj、dae格式，自动识别urdf中的mimic标签，可配置关节驱动器，一键生成sim-ready的mjcf文件。

<div style="display: flex; justify-content: center; gap: 20px;">
  <img src="./examples/agilex-piper/piper.png" alt="piper" style="width: 45%;" />
  <img src="./examples/realman-rm65/rm65.png" alt="rm65" style="width: 45%;" />
</div>

## 🚀 安装

### 安装步骤

```bash
git clone https://github.com/TATP-233/urdf2mjcf.git
cd urdf2mjcf
pip install .
```

## 📖 使用方法

### 基本转换

```bash
cd /path/to/your/robot-description/
urdf2mjcf input.urdf --output mjcf/output.xml
# 注意不要将生成的.xml放在和urdf同一目录下，可以向上面一样添加 mjcf/output.xml
```

### 命令行参数说明

```bash
urdf2mjcf <urdf_path> [options]
```

#### 必需参数
- `urdf_path`: 输入的URDF文件路径

#### 可选参数
- `-o, --output`: 输出MJCF文件路径 (默认: 与输入文件同名但扩展名为.mjcf)
- `-m, --metadata`: 包含转换元数据的JSON文件路径 (关节参数和传感器配置)
- `-dm, --default-metadata`: 默认元数据JSON文件，可指定多个文件，后面的文件会覆盖前面的设置
- `-am, --actuator-metadata`: 执行器元数据JSON文件，可指定多个文件，后面的文件会覆盖前面的设置
- `-a, --appendix`: 附加XML文件，可指定多个文件，按顺序应用
- `--collision-only`: 仅使用碰撞几何体而不显示视觉外观
- `--collision-type`: 使用碰撞类型（原样mesh，凸分解，凸包络）
- `--log-level`: 日志级别 (默认: INFO级别)
- `--max-vertices`: 网格中的最大顶点数量 (默认: 200000)

#### 元数据文件说明
- **metadata**: 主要转换配置文件，包含高度偏移、角度单位、是否添加地面等设置
- **default-metadata**: 默认关节参数配置，定义关节的默认属性
- **actuator-metadata**: 执行器配置，定义每个关节的驱动器类型和参数
- **appendix**: 附加的XML内容，会被直接添加到生成的MJCF文件中

### 使用示例

```bash
# agilex-piper机器人
cd examples/agilex-piper
urdf2mjcf piper.urdf \
  -o mjcf/piper.xml \
  -m metadata/metadata.json \
  -am metadata/actuator.json \
  -dm metadata/default.json \
  -a metadata/appendix.xml
# 查看生成的模型
python -m mujoco.viewer --mjcf=mjcf/piper.xml

# realman-rm65机械臂
cd examples/realman-rm65
urdf2mjcf rm65b_eg24c2_description.urdf \
  -o mjcf/rm65.xml \
  -m metadata/metadata.json \
  -am metadata/actuator.json \
  -dm metadata/default.json \
  -a metadata/appendix.xml
# 查看生成的模型
python -m mujoco.viewer --mjcf=mjcf/rm65.xml
```

## 🤝 致谢

本项目基于以下优秀开源项目：

- **[kscalelabs/urdf2mjcf](https://github.com/kscalelabs/urdf2mjcf)**：核心转换框架
- **[kevinzakka/obj2mjcf](https://github.com/kevinzakka/obj2mjcf)**：OBJ文件处理灵感

感谢原作者们的杰出贡献！

## 📄 许可证

MIT License