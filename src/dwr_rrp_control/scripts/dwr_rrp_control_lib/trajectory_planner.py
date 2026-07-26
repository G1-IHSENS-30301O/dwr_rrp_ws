#!/usr/bin/env python3
# trajectory_planner.py
import rospy
import math
import heapq
from geometry_msgs.msg import Twist

class TrajectoryPlanner:
    # 车体安全膨胀半径，由URDF底盘尺寸计算
    INFLATION_R = 0.55

    def __init__(self, grid, res=0.2):
        self.grid = grid
        self.res = res
        self.width = len(grid[0])
        self.height = len(grid)
        self.DEBUG = True

    def world_to_grid(self, wx, wy):
        gx = int(round(wx / self.res))
        gy = int(round(wy / self.res))
        return gx, gy

    def grid_to_world(self, gx, gy):
        return (gx + 0.5) * self.res, (gy + 0.5) * self.res

    def _is_grid_valid(self, gx, gy):
        if 0 <= gx < self.width and 0 <= gy < self.height:
            return True
        return False

    def _world_pt_collide(self, wx, wy):
        """判断世界坐标点【膨胀后】是否撞上障碍物"""
        check_step = self.res
        sample_r = self.INFLATION_R
        num_sample = int(sample_r / check_step) + 2
        for dx in range(-num_sample, num_sample + 1):
            for dy in range(-num_sample, num_sample + 1):
                sx = wx + dx * check_step
                sy = wy + dy * check_step
                gx, gy = self.world_to_grid(sx, sy)
                if self._is_grid_valid(gx, gy):
                    if self.grid[gy][gx] == 1:
                        return True
        return False

    def find_nearest_free_point(self, goal_wx, goal_wy, max_search_radius=2.0, step=0.15):
        """
        核心函数：目标落在障碍物上时，向外环形搜索最近可行停靠点
        优先沿Y方向上下搜索（适配右侧狭长通道场景，X方向空间紧张）
        返回 (nx, ny) / None
        """
        if not self._world_pt_collide(goal_wx, goal_wy):
            return (goal_wx, goal_wy)

        rospy.logwarn(f"[PLAN] 原始目标({goal_wx:.2f},{goal_wy:.2f})位于障碍区域，开始偏移搜索")
        r = step
        while r <= max_search_radius:
            # 优先垂直方向（上下）加大采样密度，适配右侧狭长通道
            angle_list = []
            # 重点优先上下 90° 270°
            angle_list.extend([math.pi / 2, 3 * math.pi / 2])
            # 再补充一圈环形采样
            angle = 0.0
            while angle < 2 * math.pi:
                angle_list.append(angle)
                angle += 0.3
            # 去重遍历
            for angle in list(set(angle_list)):
                nx = goal_wx + r * math.cos(angle)
                ny = goal_wy + r * math.sin(angle)
                if not self._world_pt_collide(nx, ny):
                    rospy.loginfo(f"[PLAN] 偏移可行点位({nx:.2f},{ny:.2f})")
                    return (nx, ny)
            r += step
        rospy.logerr(f"[PLAN] {max_search_radius}m范围内找不到可行停靠点！")
        return None

    def a_star(self, start_world, goal_world):
        """
        A*全局路径规划 8邻域，欧几里得启发
        start_world:(sx,sy)  goal_world:(gx,gy)
        返回世界坐标路径list / []
        """
        sx, sy = start_world
        gx, gy = goal_world
        start_g = self.world_to_grid(sx, sy)
        goal_g = self.world_to_grid(gx, gy)

        neighbors = [(-1,0),(1,0),(0,-1),(0,1),
                     (-1,-1),(-1,1),(1,-1),(1,1)]

        def heuristic(pt_g):
            x, y = self.grid_to_world(*pt_g)
            return math.hypot(x - gx, y - gy)

        open_heap = []
        heapq.heappush(open_heap, (heuristic(start_g), start_g))
        g_cost = {start_g: 0.0}
        came_from = {}

        found = False
        while open_heap:
            f_curr, curr_g = heapq.heappop(open_heap)
            if curr_g == goal_g:
                found = True
                break
            cxg, cyg = curr_g
            for dx, dy in neighbors:
                nxg = cxg + dx
                nyg = cyg + dy
                if not self._is_grid_valid(nxg, nyg):
                    continue
                wxn, wyn = self.grid_to_world(nxg, nyg)
                if self._world_pt_collide(wxn, wyn):
                    continue
                move_cost = math.hypot(dx*self.res, dy*self.res)
                new_g = g_cost[curr_g] + move_cost
                if (nxg, nyg) not in g_cost or new_g < g_cost[(nxg, nyg)]:
                    g_cost[(nxg, nyg)] = new_g
                    f_new = new_g + heuristic((nxg, nyg))
                    came_from[(nxg, nyg)] = curr_g
                    heapq.heappush(open_heap, (f_new, (nxg, nyg)))

        if not found:
            return []
        path_grid = []
        curr = goal_g
        while curr in came_from:
            path_grid.append(curr)
            curr = came_from[curr]
        path_grid.append(start_g)
        path_grid.reverse()
        path_world = [self.grid_to_world(gx, gy) for gx, gy in path_grid]
        return path_world

    def dijkstra(self, start_world, goal_world, use_expanded=True):
        """
        Dijkstra全局路径规划 8邻域，无启发函数，保证完备性
        start_world:(sx,sy)  goal_world:(gx,gy)
        use_expanded参数保留兼容，实际碰撞检测由_world_pt_collide动态完成
        返回世界坐标路径list / []
        """
        sx, sy = start_world
        gx, gy = goal_world
        start_g = self.world_to_grid(sx, sy)
        goal_g = self.world_to_grid(gx, gy)

        neighbors = [(-1,0),(1,0),(0,-1),(0,1),
                     (-1,-1),(-1,1),(1,-1),(1,1)]

        open_heap = []
        heapq.heappush(open_heap, (0.0, start_g))
        dist_cost = {start_g: 0.0}
        came_from = {}
        found = False

        while open_heap:
            cost, curr_g = heapq.heappop(open_heap)
            if curr_g == goal_g:
                found = True
                break
            cxg, cyg = curr_g
            for dx, dy in neighbors:
                nxg = cxg + dx
                nyg = cyg + dy
                if not self._is_grid_valid(nxg, nyg):
                    continue
                wxn, wyn = self.grid_to_world(nxg, nyg)
                # 动态车体碰撞检测
                if self._world_pt_collide(wxn, wyn):
                    continue
                step_cost = math.hypot(dx * self.res, dy * self.res)
                new_cost = cost + step_cost
                if (nxg, nyg) not in dist_cost or new_cost < dist_cost[(nxg, nyg)]:
                    dist_cost[(nxg, nyg)] = new_cost
                    came_from[(nxg, nyg)] = curr_g
                    heapq.heappush(open_heap, (new_cost, (nxg, nyg)))

        if not found:
            return []
        # 回溯路径
        path_grid = []
        curr = goal_g
        while curr in came_from:
            path_grid.append(curr)
            curr = came_from[curr]
        path_grid.append(start_g)
        path_grid.reverse()
        path_world = [self.grid_to_world(gx, gy) for gx, gy in path_grid]
        return path_world

    def follow_path(self, path, robot_pose_func, cmd_pub, params, hold_func, timeout=300.0):
        """路径跟踪，保持原有逻辑不变"""
        pass

    # ---------- 安全可达判定（带 safe_gap，防止临界边界停车） ----------
    def is_point_safely_reachable(self, x_rel, y_rel, z_world, params, safe_gap=0.08):
        """
        判断物体是否【安全】进入工作空间（带安全间隙），防止临界边界停车导致IK求解失败。
        safe_gap 默认 0.08m，在球壳内外边界各缩进保护。
        """
        a2 = params.a2
        dmin = params.D3_MIN
        dmax = params.D3_MAX
        zb = params.base_z
        d1_val = params.d1

        rho = math.hypot(x_rel, y_rel - a2)
        dz = z_world - zb - d1_val
        r_sphere = math.hypot(rho - a2, dz)

        if (r_sphere > dmin + safe_gap) and (r_sphere < dmax - safe_gap):
            if abs(dz) < (dmax - safe_gap):
                return True
        return False

    # ---------- 你已经写好的函数，直接保留 ----------
    def is_point_reachable(self, x_rel, y_rel, z_world, params):
        """
        判断物体在基座坐标系下的点 (x_rel, y_rel, z_world) 是否在工作空间（空心球壳）内。
        params 需包含: a2, D3_MIN, D3_MAX, base_z, d1
        """
        a2 = params.a2
        dmin = params.D3_MIN
        dmax = params.D3_MAX
        zb = params.base_z
        d1_val = params.d1

        rho = math.hypot(x_rel, y_rel - a2)
        dz = z_world - zb - d1_val
        r_sphere = math.hypot(rho - a2, dz)

        if r_sphere < dmin - 1e-6 or r_sphere > dmax + 1e-6:
            return False
        if abs(dz) > dmax + 1e-6:
            return False
        return True

    def align_to_workspace(self, target_pose, robot_pose_func, cmd_pub, params, hold_func, timeout=90.0):
        """
        底盘对准：使物体进入机械臂工作空间（空心球壳）才停止。
        target_pose : (wx, wy, wz) 世界坐标
        robot_pose_func : 返回 (x, y, theta) 的函数
        hold_func : 每控制周期调用一次，保持关节姿态

        修复：连续 N 帧稳定满足安全条件才退出，过滤瞬时抖动；
              退出后额外等待底盘完全静止，消除滑移。
        """
        wx, wy, wz = target_pose
        kp_dist = 0.8
        kp_ang = 0.3
        max_linear = 0.15
        max_angular = 0.8
        rate = rospy.Rate(20)
        start_t = rospy.Time.now().to_sec()

        stable_count = 0
        REQUIRED_STABLE = 10

        rospy.loginfo("[对准] 使用球壳约束进行精细调整")
        while not rospy.is_shutdown():
            if rospy.Time.now().to_sec() - start_t > timeout:
                rospy.logwarn("[对准] 超时")
                break

            hold_func()

            cx, cy, ctheta = robot_pose_func()

            dx = wx - cx
            dy = wy - cy
            x_rel = dx * math.cos(ctheta) + dy * math.sin(ctheta)
            y_rel = -dx * math.sin(ctheta) + dy * math.cos(ctheta)

            if self.is_point_safely_reachable(x_rel, y_rel, wz, params, safe_gap=0.08):
                stable_count += 1
                if stable_count >= REQUIRED_STABLE:
                    rospy.loginfo(f"[对准] 物体持续稳定处于安全工作空间，停止。连续稳定帧数:{stable_count}")
                    break
            else:
                stable_count = 0

            dist = math.hypot(dx, dy)
            ang_to_obj = math.atan2(dy, dx)
            delta_ang = math.atan2(math.sin(ang_to_obj - ctheta), math.cos(ang_to_obj - ctheta))
            rel_angle = delta_ang

            e_dist = dist - 0.85

            v = kp_dist * e_dist
            v = max(min(v, max_linear), -max_linear)
            w = kp_ang * delta_ang
            w = max(min(w, max_angular), -max_angular)

            twist = Twist()
            twist.linear.x = v
            twist.angular.z = w
            cmd_pub.publish(twist)

            rospy.loginfo_throttle(1.0, f"[对准] dist={dist:.3f} ang={math.degrees(rel_angle):.1f}° v={v:.2f} w={w:.2f} stable={stable_count}/{REQUIRED_STABLE}")
            rate.sleep()

        cmd_pub.publish(Twist())
        rospy.sleep(1.2)
        rospy.loginfo("[对准] 底盘完全停止，退出对准")