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

    # 任务定义（可选，也可放在主程序）
    TASKS = [
        {'object': 'medicine', 'priority': 'high',
         'from': (0.5, 4.0, 1.60), 'to': (19.5, 7.3, 1.60)},
        {'object': 'water',    'priority': 'medium',
         'from': (19.5, 11.0, 1.60), 'to': (19.5, 7.3, 1.60)},
        {'object': 'phone',    'priority': 'low',
         'from': (0.5, 11.0, 1.60), 'to': (19.5, 7.3, 1.60)},
        {'object': 'walker',   'priority': 'low',
         'from': (3.0, 2.0, 1.60), 'to': (19.5, 7.3, 1.60)},
    ]
