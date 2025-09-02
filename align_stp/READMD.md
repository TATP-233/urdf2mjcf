1. 使用sw导出urdf+stl（白模），将urdf文件中的mesh路径改成相对路径
2. 根据urdf合并body，生成完整的机器人模型
    ```shell
    python align_stp/merge_urdf.py /path/to/your/robot.urdf
    ```
    使用`-o`可以指定生成文件的目录，默认是`/path/to/your/robot.urdf/../merged_model`
3. 导出带外观的完整机器人模型，一个obj+mlt 或者 一个dae文件 (robot_id.obj)，下面的visual.obj和mapping.json都是第2步生成的
    ```shell
    python align_stp/assign_mesh_part.py /path/to/your/robot_id.obj -g /path/to/your/robot.urdf/../merged_model/visual.obj -m /path/to/your/robot.urdf/../merged_model/mapping.json
    ```
    使用`-o`可以指定生成文件的路径，默认是`/path/to/your/robot_id.obj/../meshes_aligned`
4. 修改urdf文件`/path/to/your/robot.urdf`，将其中的visual mesh路径修改成`/path/to/your/robot_id.obj/../meshes_aligned`中的mesh路径
5. 执行urdf到mjcf转换：
    ```shell
    urdf2mjcf /path/to/your/new/robot.urdf -o /path/to/your/new/mjcf/robot.xml --no-convex-decompose
    # (不做凸分解)
    ```