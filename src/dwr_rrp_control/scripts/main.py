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

# 膨胀一格
CELLS = 1
new_grid = [row[:] for row in grid]
for j in range(MAP_SIZE_Y):
    for i in range(MAP_SIZE_X):
        if grid[j][i] == 1:
            for dx in range(-CELLS, CELLS+1):
                for dy in range(-CELLS, CELLS+1):
                    ni, nj = i+dx, j+dy
                    if 0 <= ni < MAP_SIZE_X and 0 <= nj < MAP_SIZE_Y:
                        new_grid[nj][ni] = 1
grid = new_grid

if __name__ == '__main__':
    try:
        rospy.init_node('dwr_rrp_controller')
        controller = DWR_RRP_Controller(grid)
        controller.run()
    except rospy.ROSInterruptException:
        pass
