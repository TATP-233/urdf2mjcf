![image](./piper.png)

```bash
cd urdf2mjcf/examples/agilex-piper

urdf2mjcf piper.urdf -o mjcf/piper.xml -m metadata/metadata.json -am metadata/actuator.json -dm metadata/default.json -a metadata/appendix.xml --no-convex-decompose

python -m mujoco.viewer --mjcf=mjcf/piper.xml
```
