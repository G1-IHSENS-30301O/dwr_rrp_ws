#!/usr/bin/env python3
# controller.py
import rospy
import math
from .params import RobotParams
from .rrp_kinematics import RRPKinematicsDynamics
from .dwr_kinematics import DWRKinematics
from .joint_controller import JointController
from .trajectory_planner import TrajectoryPlanner
from geometry_msgs.msg import Twist

class DWR_RRP_Controller:
    def __init__(self, grid):
        """
        初始化控制器
        grid: 二维栅格地图 (0=可通过, 1=障碍)
        """
        self.params = RobotParams()
        self.dwr = DWRKinematics()
        self.rrp = RRPKinematicsDynamics(self.params)

        # 初始化关节控制器
        self.joint1_ctrl = JointController('joint1', kp=15.0, ki=0.0, kd=5.0)
        self.joint2_ctrl = JointController('joint2', kp=60.0, ki=0.5, kd=15.0)
        self.joint3_ctrl = JointController('joint3', kp=80.0, ki=0.5, kd=10.0)
        self.left_finger_ctrl = JointController('left_finger_joint', kp=10.0, ki=0.0, kd=2.0)
        self.right_finger_ctrl = JointController('right_finger_joint', kp=10.0, ki=0.0, kd=2.0)

        # 轨迹规划器
        self.trajectory = TrajectoryPlanner(grid, res=0.2)

        # 发布 cmd_vel
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # 行驶姿态
        self.travel_th1 = math.pi / 2
        self.travel_th2 = 0.0
        self.travel_d3 = self.params.OFFSET_EE   # 实际臂长

        rospy.sleep(1.0)

    # ---------- 辅助函数 ----------
    def set_joint_positions(self, th1, th2, d3_actual, duration=10.0):
        d3_actual = max(min(d3_actual, self.params.D3_MAX), self.params.D3_MIN)
        d3_shift = d3_actual - self.params.OFFSET_EE
        if self.trajectory.DEBUG:
            rospy.loginfo("[SET] 设置关节: th1=%.3f, th2=%.3f, d3_actual=%.3f (位移=%.3f)",
                          th1, th2, d3_actual, d3_shift)
        self.joint1_ctrl.control_to(th1, duration)
        self.joint2_ctrl.control_to(th2, duration)
        self.joint3_ctrl.control_to(d3_shift, duration)
        rospy.sleep(0.5)
        self.print_robot_state()

    def hold_travel_pose(self):
        for ctrl, target_shift in [(self.joint1_ctrl, self.travel_th1),
                                   (self.joint2_ctrl, self.travel_th2),
                                   (self.joint3_ctrl, self.travel_d3 - self.params.OFFSET_EE)]:
            pos = ctrl.get_position()
            if pos is not None:
                error = target_shift - pos
                effort = 0.02 * error
                effort = max(min(effort, 2.0), -2.0)
                ctrl.apply_effort(effort, 0.05)

    def print_robot_state(self):
        th1 = self.joint1_ctrl.get_position() or 0.0
        th2 = self.joint2_ctrl.get_position() or 0.0
        d3_shift = self.joint3_ctrl.get_position() or 0.0
        d3_actual = d3_shift + self.params.OFFSET_EE
        x, y, z = self.rrp.forward_kinematics(th1, th2, d3_actual)
        rospy.loginfo("=== Robot State ===")
        rospy.loginfo("  th1=%.3f, th2=%.3f, d3_actual=%.3f (位移=%.3f)", th1, th2, d3_actual, d3_shift)
        rospy.loginfo("  End-effector: (%.3f, %.3f, %.3f)", x, y, z)
        cx, cy, ctheta = self.dwr.get_pose()
        rospy.loginfo("  Odometry: x=%.3f, y=%.3f, theta=%.3f", cx, cy, ctheta)

    def sequential_grasp(self, ik_solution):
        th1, th2, d3_actual = ik_solution
        rospy.loginfo("[抓取] 顺序执行: d3_actual=%.2f → θ2=%.2f → θ1=%.2f", d3_actual, th2, th1)
        d3_shift = d3_actual - self.params.OFFSET_EE
        self.joint3_ctrl.control_to(d3_shift, duration=6.0)
        rospy.sleep(0.5)
        self.print_robot_state()
        self.joint2_ctrl.control_to(th2, duration=12.0)
        rospy.sleep(0.5)
        self.print_robot_state()
        self.joint1_ctrl.control_to(th1, duration=10.0)
        rospy.sleep(0.5)
        self.print_robot_state()
        # 精调
        self.joint1_ctrl.control_to(th1, duration=4.0, tolerance=0.03)
        self.joint2_ctrl.control_to(th2, duration=4.0, tolerance=0.03)
        self.joint3_ctrl.control_to(d3_shift, duration=4.0, tolerance=0.03)
        self.print_robot_state()

    def open_gripper(self):
        self.left_finger_ctrl.control_to(0.0, 1.5)
        self.right_finger_ctrl.control_to(0.0, 1.5)

    def close_gripper(self):
        self.left_finger_ctrl.control_to(0.05, 1.5)
        self.right_finger_ctrl.control_to(0.05, 1.5)

    def move_to_point(self, x_d, y_d, timeout=30.0):
        rate = rospy.Rate(20)
        start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if rospy.Time.now().to_sec() - start > timeout:
                break
            cx, cy, ctheta = self.dwr.get_pose()
            dx = x_d - cx
            dy = y_d - cy
            if math.hypot(dx, dy) < 0.3:
                break
            xc_dot = -0.8 * dx
            yc_dot = -0.8 * dy
            theta_d = math.atan2(yc_dot, xc_dot)
            v = math.sqrt(xc_dot**2 + yc_dot**2)
            v = max(min(v, self.params.MAX_V * 0.5), -self.params.MAX_V * 0.5)
            angle_diff = theta_d - ctheta
            w = -1.0 * math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            twist = Twist()
            twist.linear.x = v
            twist.angular.z = w
            self.cmd_pub.publish(twist)
            rate.sleep()
        self.cmd_pub.publish(Twist())

    def execute_task(self, task):
        obj = task['object']
        pick = task['from']
        place = task['to']
        mass_map = {'medicine': self.params.MASS_MEDICINE,
                    'water': self.params.MASS_WATER,
                    'phone': self.params.MASS_PHONE,
                    'walker': self.params.MASS_WALKER}
        mass = mass_map[obj]
        rospy.loginfo("[任务] %s 开始", obj)

        # 1. 移动到取物点附近
        cx, cy, _ = self.dwr.get_pose()
        dx = pick[0] - cx
        dy = pick[1] - cy
        app_theta = math.atan2(dy, dx)
        app_x = pick[0] - 1.0 * math.cos(app_theta)
        app_y = pick[1] - 1.0 * math.sin(app_theta)
        path = self.trajectory.dijkstra((cx, cy), (app_x, app_y), use_expanded=True)
        if path:
            self.trajectory.follow_path(path, self.dwr.get_pose, self.cmd_pub,
                                        self.params, self.hold_travel_pose)
        else:
            self.move_to_point(app_x, app_y)

        # 2. 底盘对准
        self.trajectory.align_to_workspace(pick, self.dwr.get_pose, self.cmd_pub,
                                           self.params, self.hold_travel_pose)

        # 3. 逆运动学求解抓取姿态
        cx, cy, ctheta = self.dwr.get_pose()
        dx_obj = pick[0] - cx
        dy_obj = pick[1] - cy
        x_rel = dx_obj * math.cos(ctheta) + dy_obj * math.sin(ctheta)
        y_rel = -dx_obj * math.sin(ctheta) + dy_obj * math.cos(ctheta)
        ik = self.rrp.inverse_kinematics(x_rel, y_rel, pick[2])
        if not ik:
            rospy.logerr("[任务] IK无解，跳过")
            return False

        # 4. 抓取
        self.open_gripper()
        self.sequential_grasp(ik)
        th1, th2, d3_actual = ik
        self.set_joint_positions(th1, th2, d3_actual, duration=6.0)
        self.close_gripper()
        self.set_joint_positions(th1, th2, d3_actual, duration=6.0)

        # 5. 重力补偿
        self.rrp.gravity_compensation(th1, th2, d3_actual, mass,
                                      self.joint1_ctrl, self.joint2_ctrl, self.joint3_ctrl)
        self.set_joint_positions(th1, th2, d3_actual, duration=4.0)

        # 6. 举升
        lift_d3 = max(d3_actual - 0.1, self.params.D3_MIN)
        self.joint3_ctrl.control_to(lift_d3 - self.params.OFFSET_EE, duration=3.0)
        rospy.sleep(1.0)

        # 7. 移动到放置点
        cx, cy, _ = self.dwr.get_pose()
        dx_p = place[0] - cx
        dy_p = place[1] - cy
        app_p_theta = math.atan2(dy_p, dx_p)
        app_p_x = place[0] - 1.0 * math.cos(app_p_theta)
        app_p_y = place[1] - 1.0 * math.sin(app_p_theta)
        path_p = self.trajectory.dijkstra((cx, cy), (app_p_x, app_p_y), use_expanded=True)
        if path_p:
            self.trajectory.follow_path(path_p, self.dwr.get_pose, self.cmd_pub,
                                        self.params, self.hold_travel_pose)
        else:
            self.move_to_point(app_p_x, app_p_y)

        # 8. 放置
        self.trajectory.align_to_workspace(place, self.dwr.get_pose, self.cmd_pub,
                                           self.params, self.hold_travel_pose)
        cx, cy, ctheta = self.dwr.get_pose()
        dx_pl = place[0] - cx
        dy_pl = place[1] - cy
        xr = dx_pl * math.cos(ctheta) + dy_pl * math.sin(ctheta)
        yr = -dx_pl * math.sin(ctheta) + dy_pl * math.cos(ctheta)
        ik_pl = self.rrp.inverse_kinematics(xr, yr, place[2])
        if ik_pl:
            rospy.loginfo("[放置] 移动到放置点")
            self.sequential_grasp(ik_pl)
        self.open_gripper()
        rospy.loginfo("[任务] %s 完成", obj)
        return True

    def run(self):
        prio = {'high': 0, 'medium': 1, 'low': 2}
        for t in sorted(self.params.TASKS, key=lambda x: prio[x['priority']]):
            self.execute_task(t)
