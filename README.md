# Robot-Posture-Correction
这是一个基于pybuttle，检查机器人姿态的工具（This is a tool based on Pybottle for checking the posture of a robot.）
### **机器人姿态检查工具说明文档**

#### **概述**
本工具基于 **PyBullet** 物理引擎开发，用于模拟和检查机器人的姿态。通过加载机器人模型（URDF文件），用户可以实时调整机器人的关节角度，并观察机器人的姿态变化。工具还提供了机器人状态（如位置、重心等）的实时反馈，便于调试和分析。

---

#### **功能特性**
1. **物理引擎集成**：使用 PyBullet 物理引擎进行机器人姿态模拟。
2. **关节角度控制**：通过滑块实时调整机器人各关节的角度。
3. **状态反馈**：实时获取并打印机器人的位置、姿态和重心信息。
4. **可视化调试**：支持 GUI 模式，直观查看机器人姿态变化。
5. **模块化设计**：代码结构清晰，易于扩展和修改。

---

#### **环境要求**
- **Python 版本**：3.9 或更高版本
- **依赖库**：
  - `pybullet`：物理引擎核心库
  - `numpy`：用于数学计算
- 安装依赖：
  ```bash
  pip install pybullet numpy
  ```

---

#### **代码结构**
1. **RobotController 类**：
   - 负责机器人的加载、关节控制、状态获取和调试参数设置。
   - 主要方法：
     - `__init__`：初始化物理引擎、加载机器人模型、设置初始参数。
     - `create_joint_mapping`：创建关节名称到索引的映射。
     - `setup_debug_parameters`：设置调试参数滑块。
     - `read_debug_parameters`：读取滑块当前值。
     - `apply_joint_angles`：应用关节角度到机器人。
     - `get_robot_state`：获取机器人状态（位置、姿态、重心）。
     - `print_robot_state`：打印机器人状态。
     - `run`：主循环，实时更新机器人状态。

2. **main 函数**：
   - 创建 `RobotController` 实例并启动主循环。

---

#### **使用方法**
1. **运行程序**：
   - 确保已安装依赖库。
   - 运行脚本：
     ```bash
     python robot_controller.py
     ```
   - 程序启动后，会打开 PyBullet 的 GUI 窗口，显示机器人和地面。

2. **调整关节角度**：
   - 在 GUI 窗口中，右侧会显示每个关节的滑块。
   - 拖动滑块，实时调整机器人关节角度。

3. **查看机器人状态**：
   - 程序会每秒打印一次机器人的状态信息，包括：
     - 位置（`Position`）
     - 重心（`COM`）

4. **退出程序**：
   - 按 `Ctrl+C` 终止程序。

---

#### **关键参数**
- **机器人模型**：
  - 默认加载的 URDF 文件路径为 `urdf/KQL01.urdf`。
  - 可替换为其他机器人模型文件。

- **初始位置和姿态**：
  - 初始位置：`[0, 0, 0.92]`（x, y, z）。
  - 初始姿态：`[0, 0, 0]`（欧拉角）。

- **关节角度范围**：
  - 每个关节的角度范围为 `[-π, π]`。

---

#### **扩展与定制**
1. **更换机器人模型**：
   - 修改 `self.robotId = p.loadURDF("urdf/KQL01.urdf", self.startPos, self.startOrientation)` 中的文件路径。

2. **添加更多调试参数**：
   - 在 `setup_debug_parameters` 方法中，添加新的滑块。

3. **自定义控制逻辑**：
   - 在 `run` 方法中，添加自定义的控制逻辑或算法。

4. **保存数据**：
   - 在 `get_robot_state` 方法中，将状态数据保存到文件。

---

#### **示例输出**
```
Joint 0: Ll_root_joint
Joint 1: Ll_upper_joint
...
Position: (0.0, 0.0, 0.92)
COM: (0.0, 0.0, 0.92)
```

---

#### **注意事项**
1. **URDF 文件路径**：
   - 确保 URDF 文件路径正确，且文件格式无误。

2. **物理引擎设置**：
   - 重力设置为 `-9.81 m/s²`，可根据需要调整。

3. **调试滑块**：
   - 滑块范围为 `[-π, π]`，超出范围可能导致机器人姿态异常。

4. **性能优化**：
   - 如果模拟速度较慢，可尝试关闭 GUI 模式（`p.connect(p.DIRECT)`）。

---

#### **许可证**
本项目基于 MIT 许可证开源。详细信息请参阅 [LICENSE](LICENSE) 文件。

---

**感谢使用本工具！**