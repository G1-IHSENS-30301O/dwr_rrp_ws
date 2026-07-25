#!/usr/bin/env python3
# rrp_kinematics.py
import rospy
import math
from .params import RobotParams

class RRPKinematicsDynamics:
    def __init__(self, params=None):
        if params is None:
            self.params = RobotParams()
        else:
            self.params = params
        self.DEBUG = True  # 可改为全局配置

    def forward_kinematics(self, th1, th2, d3):
        """
        正运动学：给定关节角 (th1, th2, d3_actual)，返回末端位置 (x, y, z)
        其中 d3 为实际臂长（含 OFFSET_EE）
        """
        R = self.params.a2 + d3 * math.sin(th2)
        x = R * math.cos(th1)
        y = R * math.sin(th1)
        z = self.params.base_z + self.params.d1 + d3 * math.cos(th2)
        return x, y, z

    def inverse_kinematics(self, x_e, y_e, z_e):
        """
        逆运动学：给定目标点，返回 (th1, th2, d3_actual) 或 None
        """
        dz = z_e - self.params.base_z - self.params.d1
        R = math.sqrt(x_e**2 + y_e**2)
        if R < self.params.a2:
            rospy.logwarn("[IK] 水平距离 R=%.3f < a2=%.2f", R, self.params.a2)
            return None
        d3 = math.sqrt((R - self.params.a2)**2 + dz**2)
        if d3 < self.params.D3_MIN or d3 > self.params.D3_MAX:
            rospy.logwarn("[IK] d3=%.3f 超出实际臂长范围 [%.3f, %.3f]",
                          d3, self.params.D3_MIN, self.params.D3_MAX)
            return None
        theta1 = math.atan2(y_e, x_e)
        theta2 = math.atan2(R - self.params.a2, dz)
        if theta2 > 1.50:
            theta2 = 1.50
            rospy.logwarn("[IK] theta2 限幅至 1.50")
        if self.DEBUG:
            rospy.loginfo("[IK] 结果: th1=%.3f, th2=%.3f, d3=%.3f", theta1, theta2, d3)
        return theta1, theta2, d3

    def gravity_compensation(self, th1, th2, d3, mass, joint1_ctrl, joint2_ctrl, joint3_ctrl):
        """
        施加重力补偿力矩/力
        """
        tau1 = mass * self.params.G * math.cos(th1) * math.sin(th2)
        tau2 = mass * self.params.G * math.cos(th2)
        f3 = mass * self.params.G * math.cos(th2)
        joint1_ctrl.apply_effort(tau1, 0.5)
        joint2_ctrl.apply_effort(tau2, 0.5)
        joint3_ctrl.apply_effort(f3, 0.5)
        rospy.sleep(0.6)
        joint1_ctrl.apply_effort(0.0, 0.0)
        joint2_ctrl.apply_effort(0.0, 0.0)
        joint3_ctrl.apply_effort(0.0, 0.0)
