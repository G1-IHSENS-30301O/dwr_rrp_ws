#!/usr/bin/env python3
# controller.py 重构版 分层无阻塞 状态机任务
import rospy
import math
from enum import Enum
from .params import RobotParams
from .rrp_kinematics import RRPKinematicsDynamics
from .dwr_kinematics import DWRKinematics
from .joint_controller import JointController
from .trajectory_planner import TrajectoryPlanner
from geometry_msgs.msg import Twist

# ========== 任务状态枚举 有限状态机FSM ==========
class TaskState(Enum):
    IDLE = 0                # 空闲
    APPROACH_PICK = 1       # 前往取物前置点
    ALIGN_PICK = 2          # 对准抓取工位
    SOLVE_IK_PICK = 3       # 求解抓取IK
    GRASP_ACTION = 4        # 抓取动作+重力补偿
    LIFT_OBJECT = 5         # 举升负载
    APPROACH_PLACE = 6      # 前往放置点
    ALIGN_PLACE = 7         # 对准放置工位
    DROP_OBJECT = 8         # 卸货完成
    FAULT = 9               # 故障熔断

class DWR_RRP_Controller:
    def __init__(self, grid):
        rospy.loginfo("[CTRL] 控制器初始化开始")
        self.params = RobotParams()
        self.dwr = DWRKinematics()
        self.rrp = RRPKinematicsDynamics(self.params)

        # 关节控制器
        self.joint1_ctrl = JointController('joint1', kp=30.0, ki=0.0, kd=8.0)
        self.joint2_ctrl = JointController('joint2', kp=60.0, ki=0.5, kd=15.0)
        self.joint3_ctrl = JointController('joint3', kp=80.0, ki=0.5, kd=10.0)
        self.left_finger_ctrl = JointController('left_finger_joint', kp=20.0, ki=0.0, kd=2.0)
        self.right_finger_ctrl = JointController('right_finger_joint', kp=20.0, ki=0.0, kd=2.0)

        self.trajectory = TrajectoryPlanner(grid, res=0.2)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # 行驶收起姿态参数
        self.travel_th1 = math.pi / 2
        self.travel_th2 = 0.0
        self.travel_d3 = self.params.OFFSET_EE
        rospy.sleep(1.0)
        rospy.loginfo("[CTRL] 控制器初始化完成")

    # ===================== 底层单次力输出函数【无循环，永不阻塞】 =====================
    def hold_travel_pose(self, mass=None, th1=0.0, th2=0.0, d3_actual=0.0):
        """单次输出行驶维持力，无while死循环，导航每帧调用一次"""
        target_map = [
            (self.joint1_ctrl, self.travel_th1),
            (self.joint2_ctrl, self.travel_th2),
            (self.joint3_ctrl, self.travel_d3 - self.params.OFFSET_EE)
        ]
        for ctrl, target_shift in target_map:
            pos = ctrl.get_position()
            if pos is not None:
                error = target_shift - pos
                effort = max(min(0.005 * error, 2.0), -2.0)
                ctrl.apply_effort(effort, 0.05)
        # 带负载叠加重力前馈
        if mass is not None and mass > 0:
            tau1, tau2, f3 = self.rrp.gravity_compensation(th1, th2, d3_actual, mass)
            self.joint1_ctrl.apply_effort(tau1, 0.02)
            self.joint2_ctrl.apply_effort(tau2, 0.02)
            self.joint3_ctrl.apply_effort(f3, 0.02)

    def print_robot_state(self):
        """全机器人状态打印，调试专用"""
        th1 = self.joint1_ctrl.get_position() or 0.0
        th2 = self.joint2_ctrl.get_position() or 0.0
        d3_shift = self.joint3_ctrl.get_position() or 0.0
        d3_actual = d3_shift + self.params.OFFSET_EE
        x, y, z = self.rrp.forward_kinematics(th1, th2, d3_actual)
        cx, cy, ctheta = self.dwr.get_pose()
        rospy.loginfo("===== 机器人实时状态 =====")
        rospy.loginfo(f"机械臂: th1={th1:.3f}, th2={th2:.3f}, d3_act={d3_actual:.3f}(shift={d3_shift:.3f})")
        rospy.loginfo(f"末端XYZ: X={x:.2f}, Y={y:.2f}, Z={z:.2f}")
        rospy.loginfo(f"底盘里程: X={cx:.2f}, Y={cy:.2f}, θ={ctheta:.2f}\n")

    # ===================== 机械臂动作接口 =====================
    def set_joint_pos(self, th1, th2, d3_actual, duration=10.0):
        """限幅后同步运动三轴"""
        d3_actual = max(min(d3_actual, self.params.D3_MAX), self.params.D3_MIN)
        d3_shift = d3_actual - self.params.OFFSET_EE
        rospy.loginfo(f"[ARM] 目标关节 th1={th1:.3f}, th2={th2:.3f}, d3_act={d3_actual:.3f}, shift={d3_shift:.3f}")
        self.joint1_ctrl.control_to(th1, duration)
        self.joint2_ctrl.control_to(th2, duration)
        self.joint3_ctrl.control_to(d3_shift, duration)
        self.print_robot_state()

    def sequential_grasp(self, ik_sol, move_tol=0.03):
        """三轴同步运动抓取，消除分步偏移"""
        th1, th2, d3_actual = ik_sol
        d3_shift = d3_actual - self.params.OFFSET_EE
        tx, ty, tz = self.rrp.forward_kinematics(th1, th2, d3_actual)
        rospy.loginfo(f"[GRASP] 同步三轴运动，目标末端 X={tx:.3f}, Y={ty:.3f}, Z={tz:.3f}")
        self.joint1_ctrl.control_to(th1, duration=8.0, tolerance=move_tol)
        self.joint2_ctrl.control_to(th2, duration=8.0, tolerance=move_tol)
        self.joint3_ctrl.control_to(d3_shift, duration=8.0, tolerance=move_tol)
        self.print_robot_state()

    def open_gripper(self):
        rospy.loginfo("[GRIPPER] 打开夹爪")
        self.left_finger_ctrl.control_to(0.0, 2.5)
        self.right_finger_ctrl.control_to(0.0, 2.5)

    def close_gripper(self):
        rospy.loginfo("[GRIPPER] 闭合夹爪夹持")
        self.left_finger_ctrl.control_to(0.05, 2.5)
        self.right_finger_ctrl.control_to(0.05, 2.5)

    def gravity_hold(self, th1, th2, d3_act, mass, hold_sec=8.0):
        """短时重力维持（局部循环，仅抓取阶段使用，不阻塞主线任务）"""
        rospy.loginfo(f"DYN 开启{hold_sec}秒重力补偿")
        tau1, tau2, f3 = self.rrp.gravity_compensation(th1, th2, d3_act, mass)
        rate = rospy.Rate(100)
        end_t = rospy.Time.now() + rospy.Duration(hold_sec)
        while rospy.Time.now() < end_t and not rospy.is_shutdown():
            self.joint1_ctrl.apply_effort(tau1, 0.02)
            self.joint2_ctrl.apply_effort(tau2, 0.02)
            self.joint3_ctrl.apply_effort(f3, 0.02)
            rate.sleep()
        self.joint1_ctrl.apply_effort(0.0)
        self.joint2_ctrl.apply_effort(0.0)
        self.joint3_ctrl.apply_effort(0.0)
        rospy.loginfo("[DYN] 重力补偿结束，力矩清零")

    # ===================== 地图边界与分段导航工具 =====================
    def clamp_map_xy(self, x, y):
        """根据栅格限制裁剪XY，防止超出地图导致路径为空"""
        MAP_X_MIN = 0.0
        MAP_X_MAX = 20.0
        MAP_Y_MIN = 0.0
        MAP_Y_MAX = 15.0
        x_clamp = max(MAP_X_MIN, min(x, MAP_X_MAX))
        y_clamp = max(MAP_Y_MIN, min(y, MAP_Y_MAX))
        if x != x_clamp or y != y_clamp:
            rospy.logwarn(f"坐标({x:.2f},{y:.2f})超出栅格，自动裁剪为({x_clamp:.2f},{y_clamp:.2f})")
        return x_clamp, y_clamp

    def segment_move(self, start_x, start_y, target_x, target_y, seg_len=3.0):
        """长距离拆分3m一段逐段行驶，避免单次超长move_to_target阻塞"""
        cx, cy = start_x, start_y
        total_dx = target_x - cx
        total_dy = target_y - cy
        total_dist = math.hypot(total_dx, total_dy)
        if total_dist < seg_len:
            self.move_to_target(target_x, target_y, timeout=60)
            return
        seg_num = math.ceil(total_dist / seg_len)
        for i in range(seg_num):
            ratio = (i + 1) / seg_num
            seg_x = cx + total_dx * ratio
            seg_y = cy + total_dy * ratio
            rospy.loginfo(f"分段行驶 {i+1}/{seg_num} 中间点({seg_x:.2f},{seg_y:.2f})")
            self.move_to_target(seg_x, seg_y, timeout=40)

    # ===================== 底盘运动接口 =====================
    def move_to_target(self, x_d, y_d, timeout=30.0, arrive_thresh=0.3):
        """定点比例导航，优化航向防晃动"""
        rospy.loginfo(f"[BASE] 前往目标({x_d:.2f},{y_d:.2f})，超时{timeout}s")
        rate = rospy.Rate(20)
        start_t = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            t_now = rospy.Time.now().to_sec()
            if t_now - start_t > timeout:
                rospy.logwarn("[BASE] 导航超时，强制停车")
                break
            cx, cy, cth = self.dwr.get_pose()
            dx = x_d - cx
            dy = y_d - cy
            dist = math.hypot(dx, dy)
            if dist < arrive_thresh:
                rospy.loginfo(f"[BASE] 到达目标，距离{dist:.2f}m")
                break

            theta_t = math.atan2(dy, dx)
            delta_ang = math.atan2(math.sin(theta_t - cth), math.cos(theta_t - cth))

            k_dist = 0.8
            raw_v = k_dist * dist
            max_v = self.params.MAX_V * 0.5
            v = max(min(raw_v, max_v), -max_v)

            k_ang = 0.4
            w = k_ang * delta_ang

            twist = Twist()
            twist.linear.x = v
            twist.angular.z = w
            self.cmd_pub.publish(twist)
            rate.sleep()
        self.cmd_pub.publish(Twist())
        rospy.loginfo("[BASE] 底盘停止")

    # ===================== 单任务完整状态机执行 =====================
    def execute_task(self, task):
        obj = task['object']
        pick_pt = task['pick_pos']
        place_pt = task['place_pos']
        mass_map = {
            'medicine': self.params.MASS_MEDICINE,
            'water': self.params.MASS_WATER,
            'phone': self.params.MASS_PHONE,
            'walker': self.params.MASS_WALKER
        }
        mass = mass_map[obj]
        rospy.loginfo("="*50)
        rospy.loginfo(f"【新任务启动】物品:{obj}, 负载质量={mass}kg")

        state = TaskState.APPROACH_PICK
        ik_pick = None
        lift_d3 = 0.0
        fsm_rate = rospy.Rate(1)

        while state != TaskState.IDLE and not rospy.is_shutdown():
            if state == TaskState.APPROACH_PICK:
                rospy.loginfo(f"[STEP{state.value}] 前往取物前置停靠点")
                cx, cy, _ = self.dwr.get_pose()
                dx = pick_pt[0] - cx
                dy = pick_pt[1] - cy
                app_ang = math.atan2(dy, dx)
                app_x = pick_pt[0] - math.cos(app_ang)
                app_y = pick_pt[1] - math.sin(app_ang)
                # 限制右侧最大X，远离右墙桌子，预留通行通道
                MAX_RIGHT_X = 18.4
                if app_x > MAX_RIGHT_X:
                    app_x = MAX_RIGHT_X
                    rospy.logwarn(f"限制停靠X坐标≤{MAX_RIGHT_X}，避开右侧狭窄通道")
                # 坐标边界裁剪，防止超出地图
                app_x, app_y = self.clamp_map_xy(app_x, app_y)
                # 目标自动偏移：若落在障碍物上则搜索最近可行点
                free_goal = self.trajectory.find_nearest_free_point(app_x, app_y, max_search_radius=2.0)
                if free_goal is None:
                    rospy.logerr("[PLAN] 无法找到可行停靠位置，任务故障")
                    state = TaskState.FAULT
                    break
                app_x, app_y = free_goal
                cx, cy, _ = self.dwr.get_pose()
                path = self.trajectory.dijkstra((cx, cy), (app_x, app_y), use_expanded=False)
                if path:
                    def nav_cb():
                        self.hold_travel_pose()
                    self.trajectory.follow_path(path, self.dwr.get_pose, self.cmd_pub, self.params, nav_cb)
                else:
                    rospy.logwarn("[PLAN] Dijkstra无全局路径，启用分段定点兜底")
                    self.move_to_target(app_x, app_y, timeout=120)
                rospy.loginfo(f"[STEP{state.value}] 行驶完成，切换对准工位")
                state = TaskState.ALIGN_PICK

            elif state == TaskState.ALIGN_PICK:
                rospy.loginfo(f"[STEP{state.value}] 底盘对准抓取工位")
                def nav_cb():
                    self.hold_travel_pose()
                self.trajectory.align_to_workspace(pick_pt, self.dwr.get_pose, self.cmd_pub, self.params, nav_cb)
                rospy.loginfo(f"[STEP{state.value}] 对准完成，求解IK")
                state = TaskState.SOLVE_IK_PICK

            elif state == TaskState.SOLVE_IK_PICK:
                rospy.loginfo(f"[STEP{state.value}] 坐标转换+逆运动学求解")
                cx, cy, cth = self.dwr.get_pose()
                dx_obj = pick_pt[0] - cx
                dy_obj = pick_pt[1] - cy
                x_rel = dx_obj * math.cos(cth) + dy_obj * math.sin(cth)
                y_rel = -dx_obj * math.sin(cth) + dy_obj * math.cos(cth)
                ik_pick = None
                retry = 0
                max_r = 2
                while ik_pick is None and retry <= max_r:
                    ik_pick = self.rrp.inverse_kinematics(x_rel, y_rel, pick_pt[2])
                    if ik_pick is None:
                        retry += 1
                        rospy.logwarn(f"[STEP3] IK无解，第{retry}次重新对准")
                        def nav_cb():
                            self.hold_travel_pose()
                        self.trajectory.align_to_workspace(pick_pt, self.dwr.get_pose, self.cmd_pub, self.params, nav_cb)
                        cx, cy, cth = self.dwr.get_pose()
                        dx_obj = pick_pt[0] - cx
                        dy_obj = pick_pt[1] - cy
                        x_rel = dx_obj * math.cos(cth) + dy_obj * math.sin(cth)
                        y_rel = -dx_obj * math.sin(cth) + dy_obj * math.cos(cth)
                if ik_pick is None:
                    rospy.logerr("[STEP3] IK多次求解失败，任务故障退出")
                    state = TaskState.FAULT
                    break
                th1_p, th2_p, d3_p = ik_pick
                rospy.loginfo(f"[STEP3] IK解 th1={th1_p:.3f}, th2={th2_p:.3f}, d3={d3_p:.3f}")
                state = TaskState.GRASP_ACTION

            elif state == TaskState.GRASP_ACTION:
                rospy.loginfo(f"[STEP{state.value}] 执行抓取流程")
                self.open_gripper()
                self.sequential_grasp(ik_pick)
                self.close_gripper()
                th1_p, th2_p, d3_p = ik_pick
                self.gravity_hold(th1_p, th2_p, d3_p, mass, hold_sec=8.0)
                rospy.loginfo(f"[STEP{state.value}] 抓取+重力补偿完成，准备举升")
                state = TaskState.LIFT_OBJECT

            elif state == TaskState.LIFT_OBJECT:
                rospy.loginfo(f"[STEP{state.value}] 举升负载抬高0.1m")
                th1_p, th2_p, d3_p = ik_pick
                lift_d3 = max(d3_p - 0.1, self.params.D3_MIN)
                shift_lift = lift_d3 - self.params.OFFSET_EE
                self.joint3_ctrl.control_to(shift_lift, duration=3.0)
                rospy.sleep(1.0)
                self.print_robot_state()
                rospy.loginfo(f"[STEP{state.value}] 举升完成，前往放置点")
                state = TaskState.APPROACH_PLACE

            elif state == TaskState.APPROACH_PLACE:
                rospy.loginfo(f"[STEP{state.value}] 前往放置前置点")
                cx, cy, _ = self.dwr.get_pose()
                dx = place_pt[0] - cx
                dy = place_pt[1] - cy
                app_ang = math.atan2(dy, dx)
                app_x = place_pt[0] - math.cos(app_ang)
                app_y = place_pt[1] - math.sin(app_ang)
                # 限制右侧最大X，远离右墙桌子，预留通行通道
                MAX_RIGHT_X = 18.4
                if app_x > MAX_RIGHT_X:
                    app_x = MAX_RIGHT_X
                    rospy.logwarn(f"限制停靠X坐标≤{MAX_RIGHT_X}，避开右侧狭窄通道")
                # 坐标边界裁剪，防止超出地图
                app_x, app_y = self.clamp_map_xy(app_x, app_y)
                # 目标自动偏移：若落在障碍物上则搜索最近可行点
                free_goal = self.trajectory.find_nearest_free_point(app_x, app_y, max_search_radius=2.0)
                if free_goal is None:
                    rospy.logerr("[PLAN] 无法找到可行停靠位置，任务故障")
                    state = TaskState.FAULT
                    break
                app_x, app_y = free_goal
                cx, cy, _ = self.dwr.get_pose()
                path_p = self.trajectory.dijkstra((cx, cy), (app_x, app_y), use_expanded=False)
                if not path_p:
                    rospy.logwarn("[PLAN] Dijkstra无全局路径，启用分段定点兜底")
                def nav_load_cb():
                    self.hold_travel_pose()
                if path_p:
                    self.trajectory.follow_path(path_p, self.dwr.get_pose, self.cmd_pub, self.params, nav_load_cb)
                else:
                    cx_now, cy_now, _ = self.dwr.get_pose()
                    self.segment_move(cx_now, cy_now, app_x, app_y, seg_len=3.0)
                rospy.loginfo(f"[STEP{state.value}] 到达放置区域，对准工位")
                state = TaskState.ALIGN_PLACE

            elif state == TaskState.ALIGN_PLACE:
                rospy.loginfo(f"[STEP{state.value}] 对准放置工位")
                def nav_cb():
                    self.hold_travel_pose(mass=mass, d3_actual=lift_d3)
                self.trajectory.align_to_workspace(place_pt, self.dwr.get_pose, self.cmd_pub, self.params, nav_cb)
                rospy.loginfo(f"[STEP{state.value}] 对准完成，准备卸货")
                state = TaskState.DROP_OBJECT

            elif state == TaskState.DROP_OBJECT:
                rospy.loginfo(f"[STEP{state.value}] 卸货流程")
                cx, cy, cth = self.dwr.get_pose()
                dx_pl = place_pt[0] - cx
                dy_pl = place_pt[1] - cy
                xr = dx_pl * math.cos(cth) + dy_pl * math.sin(cth)
                yr = -dx_pl * math.sin(cth) + dy_pl * math.cos(cth)
                ik_drop = self.rrp.inverse_kinematics(xr, yr, place_pt[2])
                if ik_drop is not None:
                    self.sequential_grasp(ik_drop)
                self.open_gripper()
                rospy.loginfo(f"【任务完成】{obj} 搬运结束")
                state = TaskState.IDLE

            elif state == TaskState.FAULT:
                rospy.logerr(f"【任务故障】收回机械臂行驶姿态")
                self.set_joint_pos(self.travel_th1, self.travel_th2, self.travel_d3, duration=3.0)
                self.cmd_pub.publish(Twist())
                return False
            fsm_rate.sleep()
        return True

    def run(self):
        """顶层调度：按任务优先级串行执行"""
        rospy.loginfo("[MAIN] 启动任务调度器")
        prio_weight = {'high':0, 'medium':1, 'low':2}
        sorted_tasks = sorted(self.params.TASKS, key=lambda x: prio_weight[x['priority']])
        for task in sorted_tasks:
            success = self.execute_task(task)
            if not success:
                rospy.logwarn("[MAIN] 当前任务失败，继续下一项")
        rospy.loginfo("[MAIN] 全部任务执行完毕，进入空闲")
