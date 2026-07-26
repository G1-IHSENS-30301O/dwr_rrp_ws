#!/usr/bin/env python3
# main.py
import rospy
from dwr_rrp_control_lib.controller import DWR_RRP_Controller

# ========== 栅格地图生成（同原代码） ==========
MAP_RES = 0.2
MAP_SIZE_X = int(20.0 / MAP_RES) + 1
MAP_SIZE_Y = int(15.0 / MAP_RES) + 1
grid = [[0]*MAP_SIZE_X for _ in range(MAP_SIZE_Y)]

def set_block(wx_min, wx_max, wy_min, wy_max):
    ix_min = int(wx_min / MAP_RES); ix_max = int(wx_max / MAP_RES) + 1
    iy_min = int(wy_min / MAP_RES); iy_max = int(wy_max / MAP_RES) + 1
    for i in range(max(0, ix_min), min(MAP_SIZE_X, ix_max)):
        for j in range(max(0, iy_min), min(MAP_SIZE_Y, iy_max)):
            grid[j][i] = 1

walls = [
    (0.0, 0.2, 0.0, 15.0), (19.8, 20.0, 0.0, 15.0),
    (0.0, 20.0, 0.0, 0.2), (0.0, 20.0, 14.8, 15.0),
    (0.35, 0.65, 3.7, 4.3), (19.35, 19.65, 10.7, 11.3),
    (0.35, 0.65, 10.7, 11.3), (19.35, 19.65, 6.7, 7.3),
    (9.6, 10.4, 7.6, 8.4), (19.2, 19.8, 3.0, 6.0),
    (4.85, 5.15, 12.85, 13.15), (14.85, 15.15, 1.85, 2.15),
]
for w in walls:
    set_block(*w)

# 全局静态膨胀已移除。
# 车体安全半径 0.55m 保护移至 trajectory_planner.py 的 _world_pt_collide，
# A* 遍历节点时实时判断碰撞，避免狭窄通道被一次性堵死。

# 调试工具，运行一次查看各目标点可行停靠位置
from dwr_rrp_control_lib.trajectory_planner import TrajectoryPlanner
tp = TrajectoryPlanner(grid, MAP_RES)
test_points = [
    (19.5, 7.3),   # 放置原始目标
    (0.5, 4.0),
    (19.5, 11.0)
]
for x, y in test_points:
    free = tp.find_nearest_free_point(x, y)
    print(f"目标({x},{y}) → 可行停靠点 {free}")

if __name__ == '__main__':
    try:
        rospy.init_node('dwr_rrp_controller')
        controller = DWR_RRP_Controller(grid)
        controller.run()
    except rospy.ROSInterruptException:
        pass
