#!/usr/bin/env python3
# params.py
class RobotParams:
    # RRP 臂参数
    d1 = 1.15                  # 基座到关节2高度 (m)
    a2 = 0.8                   # 关节2到伸缩杆基座水平连杆 (m)
    OFFSET_EE = 0.51           # 固定偏置 (位移零位到末端) (m)
    D3_MIN = 0.11              # 实际臂长下限 (m)
    D3_MAX = 0.61              # 实际臂长上限 (m)
    base_z = 0.25              # 底盘高度 (m)
    GRAB_Z = 1.60              # 统一抓取放置Z高度

    # 负载质量 (kg)
    MASS_MEDICINE = 0.2
    MASS_WATER = 0.3
    MASS_PHONE = 0.15
    MASS_WALKER = 2.0
    G = 9.81                   # 重力加速度 (m/s^2)

    # 移动平台参数
    MAX_V = 0.6                # 最大线速度 (m/s)
    MAX_W = 1.5                # 最大角速度 (rad/s)
    LOOKAHEAD = 1.0            # 前瞻距离 (m)

    # 任务定义（修复关键字冲突+统一高度）
    TASKS = [
         {'object': 'medicine', 'priority': 'high',
          'pick_pos': (0.5, 4.0, GRAB_Z), 'place_pos': (19.5, 7.25, GRAB_Z)},
        {'object': 'water',    'priority': 'medium',
          'pick_pos': (19.5, 11.0, GRAB_Z), 'place_pos': (19.5, 7.25, GRAB_Z)},
        {'object': 'phone',    'priority': 'low',
          'pick_pos': (0.5, 11.0, GRAB_Z), 'place_pos': (19.5, 7.25, GRAB_Z)},
        {'object': 'walker',   'priority': 'low',
          'pick_pos': (3.0, 2.0, GRAB_Z), 'place_pos': (19.5, 7.25, GRAB_Z)},
    ]