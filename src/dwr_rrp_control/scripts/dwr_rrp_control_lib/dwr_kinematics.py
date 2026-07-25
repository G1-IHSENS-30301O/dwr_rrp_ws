#!/usr/bin/env python3
# dwr_kinematics.py
import rospy
import tf
from threading import Lock
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class DWRKinematics:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.lock = Lock()
        rospy.Subscriber('/odom', Odometry, self.odom_cb)

    def odom_cb(self, msg):
        with self.lock:
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.theta = euler[2]

    def get_pose(self):
        with self.lock:
            return self.x, self.y, self.theta

    @staticmethod
    def publish_cmd_vel(pub, v, w):
        """
        发布线速度和角速度
        """
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        pub.publish(twist)
