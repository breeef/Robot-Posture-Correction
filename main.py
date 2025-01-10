import pybullet as p #pip install pybullet
import pybullet_data
import time
import math
import numpy as np #pip install numpy

class RobotController:
    def __init__(self):
        # 连接物理引擎
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # 设置环境
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(1)
        
        # 加载地面
        self.planeId = p.loadURDF("plane.urdf")
        # 机器人初始位置和姿态
        self.startPos = [0, 0, 0.92]
        self.startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        
        # 加载机器人
        self.robotId = p.loadURDF("urdf/KQL01.urdf", self.startPos, self.startOrientation)
        
        # 初始关节角度配置
        self.default_joint_angles = {
            'Ll_root_joint': 0.0,     # 左髋偏航
            'Ll_upper_joint': 0.0,   # 左髋横滚
            'Ll_knee_joint': 0.0,     # 左髋俯仰
            'Ll_lower_joint': 0.0,   # 左膝
            'Lf_pitch_joint': 0.0,    # 左踝俯仰
            'Lf_row_joint': 0.0,      # 左踝横滚
            'Rl_root_joint': 0.0,     # 右髋偏航
            'Rl_upper_joint': 0.0,    # 右髋横滚
            'Rl_knee_joint': 0.0,     # 右髋俯仰
            'Rl_lower_joint': 0.0,   # 右膝
            'Rf_pitch_joint': 0.0,    # 右踝俯仰
            'Rf_row_joint': 0.0,      # 右踝横滚
        }
        
        # 创建关节名称到索引的映射
        self.joint_indices = {}
        self.create_joint_mapping()
        
        # 设置调试参数
        self.setup_debug_parameters()

    def create_joint_mapping(self):
        """创建关节名称到索引的映射"""
        for i in range(p.getNumJoints(self.robotId)):
            joint_info = p.getJointInfo(self.robotId, i)
            joint_name = joint_info[1].decode('utf-8')
            self.joint_indices[joint_name] = i
            print(f"Joint {i}: {joint_name}")

    def setup_debug_parameters(self):
        """设置调试参数滑块"""
        self.debug_params = {}
        param_range = [-math.pi, math.pi]  # 关节角度范围
        
        # 为每个关节创建一个滑块
        y_position = 20
        for joint_name, default_angle in self.default_joint_angles.items():
            self.debug_params[joint_name] = p.addUserDebugParameter(
                joint_name, param_range[0], param_range[1], 
                default_angle, physicsClientId=self.physicsClient
            )
            y_position += 30

    def read_debug_parameters(self):
        """读取调试参数的当前值"""
        angles = {}
        for joint_name, param_id in self.debug_params.items():
            angles[joint_name] = p.readUserDebugParameter(param_id)
        return angles

    def apply_joint_angles(self, angles):
        """应用关节角度"""
        for joint_name, angle in angles.items():
            if joint_name in self.joint_indices:
                joint_index = self.joint_indices[joint_name]
                p.setJointMotorControl2(
                    self.robotId,
                    joint_index,
                    p.POSITION_CONTROL,
                    targetPosition=angle,
                    force=500
                )

    def get_robot_state(self):
        """获取机器人状态信息"""
        # 获取基座位置和姿态
        pos, orn = p.getBasePositionAndOrientation(self.robotId)
        # 获取重心位置
        com = p.getBasePositionAndOrientation(self.robotId)[0]
        return {
            "position": pos,
            "orientation": orn,
            "com": com
        }

    def print_robot_state(self):
        """打印机器人状态"""
        state = self.get_robot_state()
        print(f"Position: {state['position']}")
        print(f"COM: {state['com']}")

    def run(self):
        """主循环"""
        try:
            while True:
                # 读取并应用调试参数
                current_angles = self.read_debug_parameters()
                self.apply_joint_angles(current_angles)
                
                # 每秒打印一次机器人状态
                if int(time.time()) % 1 == 0:
                    self.print_robot_state()
                
                time.sleep(0.01)
        except KeyboardInterrupt:
            p.disconnect()

def main():
    controller = RobotController()
    controller.run()

if __name__ == "__main__":
    main()