#!/usr/bin/env python3
# trajectory_planner.py
import rospy
import math
import heapq
from geometry_msgs.msg import Twist

class TrajectoryPlanner:
    def __init__(self, grid, res=0.2):
        self.grid = grid          # 二维列表，0 可行，1 障碍
        self.res = res
        self.width = len(grid[0])
        self.height = len(grid)
        self.DEBUG = True

    def world_to_grid(self, wx, wy):
        return int(wx / self.res), int(wy / self.res)

    def grid_to_world(self, gx, gy):
        return (gx + 0.5) * self.res, (gy + 0.5) * self.res

    def dijkstra(self, start_world, goal_world, use_expanded=True):
        """
        Dijkstra 全局路径规划
        """
        g = self.grid if use_expanded else [[0]*self.width for _ in range(self.height)]
        start = self.world_to_grid(*start_world)
        goal = self.world_to_grid(*goal_world)
        if start == goal:
            return [start_world]
        directions = [(1,0), (-1,0), (0,1), (0,-1), (1,1), (1,-1), (-1,1), (-1,-1)]
        dist = [[float('inf')]*self.width for _ in range(self.height)]
        parent = [[None]*self.width for _ in range(self.height)]
        dist[start[1]][start[0]] = 0.0
        pq = [(0.0, start)]
        while pq:
            d, (x, y) = heapq.heappop(pq)
            if d > dist[y][x]:
                continue
            if (x, y) == goal:
                break
            for dx, dy in directions:
                nx, ny = x+dx, y+dy
                if 0 <= nx < self.width and 0 <= ny < self.height and g[ny][nx] == 0:
                    cost = 1.0 if dx == 0 or dy == 0 else 1.414
                    nd = d + cost
                    if nd < dist[ny][nx]:
                        dist[ny][nx] = nd
                        parent[ny][nx] = (x, y)
                        heapq.heappush(pq, (nd, (nx, ny)))
        if dist[goal[1]][goal[0]] == float('inf'):
            rospy.logwarn("[规划] 路径未找到")
            return []
        path = []
        cur = goal
        while cur is not None:
            path.append(self.grid_to_world(cur[0], cur[1]))
            cur = parent[cur[1]][cur[0]]
        path.reverse()
        rospy.loginfo("[规划] 路径点数: %d", len(path))
        return path

    def follow_path(self, path, robot_pose_func, cmd_pub, params, hold_func, timeout=300.0):
        """
        路径跟踪：纯跟踪控制器
        robot_pose_func: 返回 (x, y, theta) 的函数
        hold_func: 保持关节姿态的函数
        """
        if not path:
            return
        rate = rospy.Rate(20)
        start_t = rospy.Time.now().to_sec()
        last_idx = 0
        while not rospy.is_shutdown():
            if rospy.Time.now().to_sec() - start_t > timeout:
                break
            cx, cy, ctheta = robot_pose_func()
            # 查找前瞻点
            target_idx = last_idx
            while target_idx < len(path)-1:
                dx = path[target_idx][0] - cx
                dy = path[target_idx][1] - cy
                if math.hypot(dx, dy) < params.LOOKAHEAD:
                    target_idx += 1
                else:
                    break
            last_idx = target_idx
            if target_idx >= len(path)-1:
                gx, gy = path[-1]
                if math.hypot(gx-cx, gy-cy) < 0.3:
                    cmd_pub.publish(Twist())
                    rospy.sleep(2.0)
                    break
                # 终点控制
                xc_dot = -0.8*(gx-cx)
                yc_dot = -0.8*(gy-cy)
                theta_d = math.atan2(yc_dot, xc_dot)
                v = math.sqrt(xc_dot**2 + yc_dot**2)
                v = max(min(v, params.MAX_V*0.5), -params.MAX_V*0.5)
                w = -1.0 * math.atan2(math.sin(theta_d-ctheta), math.cos(theta_d-ctheta))
                twist = Twist()
                twist.linear.x = v
                twist.angular.z = w
                cmd_pub.publish(twist)
                rate.sleep()
                continue
            # 前瞻点控制
            gx, gy = path[target_idx]
            dx_w = gx - cx
            dy_w = gy - cy
            dx_b = dx_w*math.cos(ctheta) + dy_w*math.sin(ctheta)
            dy_b = -dx_w*math.sin(ctheta) + dy_w*math.cos(ctheta)
            target_angle = math.atan2(dy_b, dx_b)
            v = params.MAX_V
            if abs(target_angle) > 0.5:
                v *= 0.6
            w = 1.5 * target_angle
            twist = Twist()
            twist.linear.x = v
            twist.angular.z = w
            cmd_pub.publish(twist)
            rate.sleep()
        cmd_pub.publish(Twist())

    def align_to_workspace(self, target_pose, robot_pose_func, cmd_pub, params, hold_func, timeout=90.0):
        """
        底盘对准：使机器人停在距离目标点 1.25~1.35m 处
        """
        obj_x, obj_y, _ = target_pose
        kp_ang = 0.8
        max_linear = 0.15
        max_angular = 0.8
        rate = rospy.Rate(20)
        start_t = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if rospy.Time.now().to_sec() - start_t > timeout:
                rospy.logwarn("[对准] 超时")
                break
            hold_func()   # 保持关节姿态
            cx, cy, ctheta = robot_pose_func()
            dx = obj_x - cx
            dy = obj_y - cy
            dist = math.hypot(dx, dy)
            rel_angle = math.atan2(dy, dx) - ctheta
            rel_angle = math.atan2(math.sin(rel_angle), math.cos(rel_angle))
            if 1.25 < dist < 1.35 and abs(rel_angle) < 0.1:
                rospy.loginfo("[对准] 到达目标窗口, dist=%.3f, ang=%.1f°", dist, math.degrees(rel_angle))
                break
            v = 0.4 * (dist - 1.30)
            v = max(min(v, max_linear), -max_linear)
            w = kp_ang * rel_angle
            w = max(min(w, max_angular), -max_angular)
            twist = Twist()
            twist.linear.x = v
            twist.angular.z = w
            cmd_pub.publish(twist)
            rospy.loginfo_throttle(1.0, "[对准] dist=%.3f ang=%.1f° v=%.2f w=%.2f",
                                   dist, math.degrees(rel_angle), v, w)
            rate.sleep()
        cmd_pub.publish(Twist())
        rospy.sleep(1.0)
