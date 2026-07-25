#!/usr/bin/env python3
# joint_controller.py
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
        self.apply_srv = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        self.get_srv = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
        self.DEBUG = True

    def get_position(self):
        try:
            res = self.get_srv(self.joint_name)
            if res.success and len(res.position) > 0:
                return res.position[0]
        except:
            pass
        return None

    def apply_effort(self, effort, duration=0.05):
        try:
            self.apply_srv(self.joint_name, effort, rospy.Time(0,0), rospy.Duration(duration))
        except:
            pass

    def control_to(self, target, duration=8.0, tolerance=0.05):
        rate = rospy.Rate(50)
        start = rospy.Time.now().to_sec()
        self.prev_pos = None
        self.integral = 0.0
        if self.DEBUG:
            rospy.loginfo("[CTRL] %s 目标=%.3f, 持续时间=%.1f", self.joint_name, target, duration)
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            if now - start > duration:
                break
            pos = self.get_position()
            if pos is None:
                rospy.sleep(0.02)
                continue
            error = target - pos
            if abs(error) < tolerance:
                break
            if self.prev_pos is not None:
                vel = (pos - self.prev_pos) / 0.02
            else:
                vel = 0.0
            self.integral += error * 0.02
            self.integral = max(min(self.integral, 20.0), -20.0)
            effort = self.kp * error + self.ki * self.integral - self.kd * vel
            effort = max(min(effort, 100.0), -100.0)
            self.apply_effort(effort, 0.02)
            self.prev_pos = pos
            rate.sleep()
        self.apply_effort(0.0, 0.0)
        if self.DEBUG:
            final_pos = self.get_position()
            rospy.loginfo("[CTRL] %s 完成, 最终=%.3f", self.joint_name, final_pos if final_pos is not None else 0.0)
