

# urdf2mjcf

一个功能强大的URDF到MJCF转换工具，支持高级网格处理和机器人仿真优化。

## ✨ 特色功能

- 🔄 **智能包路径解析**：支持ROS workspace自动检测，兼容嵌套包结构
- 🎯 **凸分解处理**：自动对collision mesh进行凸分解，提高仿真效率
- 🏗️ **网格简化**：自动简化高多边形网格，优化性能
- 🔍 **Shell检测**：自动检测共面mesh并标记为shell，提高仿真精度
- 🎨 **材质处理**：支持OBJ+MTL材质自动导入和处理
- 📦 **多格式支持**：支持STL、OBJ等多种mesh格式

## 🚀 安装

### 环境要求

- Python 3.8+
- mujoco 3.3.2+
- NumPy
- Trimesh
- lxml

### 安装步骤

1. 克隆仓库：
```bash
git clone https://github.com/your-username/urdf2mjcf.git
cd urdf2mjcf
```

2. 安装依赖：
```bash
pip install -r requirements.txt
```

3. 安装包（开发模式）：
```bash
pip install -e .
```

## 📖 使用方法

### 基本转换

```bash
python convert.py input.urdf --output output.xml
```

### 高级用法

```bash
# 使用元数据文件
python convert.py robot.urdf --output robot.xml --metadata-file metadata.json

# 指定输出目录
python convert.py /path/to/robot.urdf --output /path/to/output/robot.xml
```

### Python API

```python
from urdf2mjcf.convert import convert_urdf_to_mjcf
from pathlib import Path

# 基本转换
convert_urdf_to_mjcf(
    urdf_path="path/to/robot.urdf",
    mjcf_path="path/to/robot.xml"
)

# 带元数据的转换
convert_urdf_to_mjcf(
    urdf_path="path/to/robot.urdf",
    mjcf_path="path/to/robot.xml",
    metadata_file="path/to/metadata.json"
)
```

## 🔧 主要功能

### 1. 智能包路径解析

- **rospackage**:支持无需在urdf中添加<mujoco>标签,无需手动替换`package://xxx`即可自动找到mesh资源文件
- **自动workspace检测**：从URDF文件位置向上查找ROS workspace
- **递归包搜索**：支持`ros_ws/src/xxx/package1`嵌套结构
- **多策略兼容**：即使没有rospkg也能正常工作
- **环境变量支持**：自动读取ROS_WORKSPACE等环境变量

### 2. 凸分解处理

```python
# 自动处理collision mesh
# 输入: models/xxx.stl
# 输出: models/xxx_parts/xxx_part1.stl, xxx_part2.stl, ...
```

- 使用CoACD算法进行高质量凸分解
- 自动更新MJCF文件中的collision geom
- 保持原始目录结构
- 避免重复处理相同mesh

### 3. 网格优化

- **自动简化**：简化高多边形网格（>200k顶点 mujoco最大限制）
- **智能比例**：基于网格复杂度动态调整简化比例
- **质量保持**：使用二次误差度量算法保持几何特征

### 4. Shell检测

- **共面检测**：自动检测mesh顶点是否全部共面
- **Shell标记**：为平面mesh添加`inertia="shell"`属性，支持mujoco3.3.2+新特性
- **数学精确**：使用严格的几何算法判断共面性
- **性能优化**：Shell设置能显著提高仿真效率

### 5. 材质处理

- **OBJ+MTL支持**：自动处理OBJ文件的MTL材质
- **材质分离**：支持按材质分离mesh子对象
- **颜色映射**：URDF材质颜色自动映射到MJCF
- **纹理支持**：保持原始纹理映射

## 📁 项目结构

```
urdf2mjcf/
├── urdf2mjcf/
│   ├── convert.py              # 主转换模块
│   ├── package_resolver.py     # 包路径解析
│   ├── materials.py           # 材质处理
│   ├── geometry.py            # 几何计算
│   ├── mjcf_builders.py       # MJCF构建器
│   └── postprocess/           # 后处理模块
│       ├── convex_decomposition.py  # 凸分解
│       ├── check_shell.py           # Shell检测
│       ├── update_mesh.py           # 网格更新
│       └── ...
├── models/                    # 示例模型
├── requirements.txt
└── README.md
```

## 🛠️ 后处理流程

转换过程中自动执行以下后处理步骤：

1. **添加地面和光照**：创建基本仿真环境
2. **凸分解**：处理collision mesh
3. **Shell检测**：标记共面mesh
4. **度数转换**：支持角度单位转换
5. **约束处理**：添加关节约束和阻尼
6. **冗余移除**：清理重复元素

## 🤝 致谢

本项目基于以下优秀开源项目：

- **[kscalelabs/urdf2mjcf](https://github.com/kscalelabs/urdf2mjcf)**：核心转换框架
- **[kevinzakka/obj2mjcf](https://github.com/kevinzakka/obj2mjcf)**：OBJ文件处理灵感

感谢原作者们的杰出贡献！

## 📄 许可证

MIT License

## 🐛 问题反馈

如果遇到问题或有建议，请在GitHub上提交Issue。

## 🔗 相关链接

- [MuJoCo官方文档](https://mujoco.readthedocs.io/)
- [URDF规格说明](http://wiki.ros.org/urdf)
- [MJCF文件格式](https://mujoco.readthedocs.io/en/latest/XMLreference.html)
