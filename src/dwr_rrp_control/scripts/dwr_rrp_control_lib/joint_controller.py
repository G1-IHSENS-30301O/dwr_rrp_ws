#!/usr/bin/env python3
# joint_controller.py 底层单轴PID控制器（硬件层，无业务逻辑）
import rospy
from gazebo_msgs.srv import ApplyJointEffort, GetJointProperties

class JointController:
    def __init__(self, joint_name, kp=15.0, ki=0.0, kd=5.0):
        self.joint_name = joint_name
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_pos = None
        self.integral = 0.0
        rospy.wait_for_service('/gazebo/apply_joint_effort')
        rospy.wait_for_service('/gazebo/get_joint_properties')
        # 修复ServiceProxy对应服务
        self.apply_srv = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        self.get_srv = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
        self.DEBUG = True

    def get_position(self):
        """读取当前关节角度/位移，失败返回None"""
        try:
            res = self.get_srv(self.joint_name)
            if res.success and len(res.position) > 0:
                return res.position[0]
        except rospy.ServiceException as e:
            if self.DEBUG:
                rospy.logwarn(f"{self.joint_name} 读取关节服务异常: {e}")
        return None

    def apply_effort(self, effort, duration=0.05):
        """短时输出力矩，无闭环"""
        try:
            self.apply_srv(self.joint_name, effort, rospy.Time(0), rospy.Duration(duration))
        except rospy.ServiceException as e:
            if self.DEBUG:
                rospy.logwarn(f"{self.joint_name} 下发力矩服务异常: {e}")

    def control_to(self, target, duration=8.0, tolerance=0.05):
        """位置闭环PID，内部短时循环，结束自动清零力矩"""
        rate = rospy.Rate(50)
        start = rospy.Time.now().to_sec()
        self.prev_pos = None
        self.integral = 0.0
        if self.DEBUG:
            rospy.loginfo(f"[CTRL-{self.joint_name}] 目标={target:.3f}, 最大时长={duration}s")
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            if now - start > duration:
                rospy.logwarn(f"[CTRL-{self.joint_name}] 运动超时退出")
                break
            pos = self.get_position()
            if pos is None:
                rospy.sleep(0.02)
                continue
            err = target - pos
            if abs(err) < tolerance:
                rospy.loginfo(f"[CTRL-{self.joint_name}] 到达误差阈值{tolerance}")
                break
            # 微分项
            vel = (pos - self.prev_pos) / 0.02 if self.prev_pos is not None else 0.0
            self.integral = max(min(self.integral + err * 0.02, 20), -20)
            effort = self.kp * err + self.ki * self.integral - self.kd * vel
            effort = max(min(effort, 100), -100)
            self.apply_effort(effort, 0.02)
            self.prev_pos = pos
            rate.sleep()
        # 运动结束清除输出力矩
        self.apply_effort(0.0)
        final_pos = self.get_position() or 0.0
        if self.DEBUG:
            rospy.loginfo(f"[CTRL-{self.joint_name}] 完成，最终位置={final_pos:.3f}")