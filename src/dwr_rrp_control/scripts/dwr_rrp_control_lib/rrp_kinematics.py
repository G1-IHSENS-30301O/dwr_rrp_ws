#!/usr/bin/env python3
# rrp_kinematics.py
import math
import numpy as np

def wrap_rad(angle):
    two_pi = 2 * math.pi
    rem = angle % two_pi
    if rem > math.pi:
        rem -= two_pi
    return rem

class RRPKinematicsDynamics:
    def __init__(self, params):
        self.params = params
        # 迭代参数
        self.max_iter = 20
        self.eps = 1e-4
        self.step = 0.5
    
    # Forward Kinematics Analysis
    def forward_kinematics(self, th1, th2, d3):
        """新版DH FK(固定)"""
        s1 = math.sin(th1)
        c1 = math.cos(th1)
        s2 = math.sin(th2)
        c2 = math.cos(th2)
        a2 = self.params.a2
        zb = self.params.base_z
        d1 = self.params.d1
        x = -a2 * s1 + d3 * c1 * s2
        y =  a2 * c1 + d3 * s1 * s2
        z = zb + d1 + d3 * c2
        return x,y,z

    # Inverse Kinematics Analysis
    def inverse_kinematics(self, x_t, y_t, z_t, q0=None):
        """
        q0:初始猜测 [th1,th2,d3]
        返回 (th1,th2,d3) or None
        """
        import numpy as np
        # 初始猜测
        if q0 is None:
            th1, th2, d3 = 0.0, 0.8, 0.3
        else:
            th1, th2, d3 = q0

        for _ in range(self.max_iter):
            x,y,z = self.forward_kinematics(th1,th2,d3)
            ex = x - x_t
            ey = y - y_t
            ez = z - z_t
            err = np.array([ex,ey,ez])
            if np.linalg.norm(err) < self.eps:
                break
            J = np.array(self.jacobian(th1,th2,d3))
            dq = -self.step * self.mat_pinv(J) @ err
            th1 += dq[0]
            th2 += dq[1]
            d3 += dq[2]

            # 关节限位
            d3 = max(self.params.D3_MIN, min(d3, self.params.D3_MAX))
        xf,yf,zf = self.forward_kinematics(th1,th2,d3)
        if math.hypot(xf-x_t,yf-y_t,zf-z_t) > 5e-3:
            return None
        # 归一化角度
        th1 = wrap_rad(th1)
        # 俯仰硬件限位 ±90°
        th2 = max(min(wrap_rad(th2), math.pi / 2), -math.pi / 2)
        return th1, th2, d3

    # Differential Kinematics
    def jacobian(self, th1, th2, d3):
        """
        J = [[dx/dth1, dx/dth2, dx/dd3],
             [dy/dth1, dy/dth2, dy/dd3],
             [dz/dth1, dz/dth2, dz/dd3]]
        """
        s1 = math.sin(th1)
        c1 = math.cos(th1)
        s2 = math.sin(th2)
        c2 = math.cos(th2)
        a2 = self.params.a2

        dxdt1 = -a2*c1 - d3*s1*s2
        dxdt2 = d3*c1*c2
        dxdd3 = c1*s2

        dydt1 = -a2*s1 + d3*c1*s2
        dydt2 = d3*s1*c2
        dydd3 = s1*s2

        dzdt1 = 0.0
        dzdt2 = -d3*s2
        dzdd3 = c2
        return [
            [dxdt1, dxdt2, dxdd3],
            [dydt1, dydt2, dydd3],
            [dzdt1, dzdt2, dzdd3]
        ]

    def mat_pinv(self, J):
        Jnp = np.array(J)
        return np.linalg.pinv(Jnp)

    # Gravity Compensation
    def gravity_compensation(self, th1, th2, d3, mass):
        g = self.params.G
        # 严格遵循PPT τ=J^T F 推导结果
        tau1 = 0.0
        tau2 = -mass * g * d3 * math.sin(th2)
        f3 = mass * g * math.cos(th2)
        
        # 力矩限幅保护驱动器（PPT工程实现要点）
        MAX_TORQUE = 15.0
        MAX_FORCE = 90.0
        tau2 = max(min(tau2, MAX_TORQUE), -MAX_TORQUE)
        f3 = max(min(f3, MAX_FORCE), -MAX_FORCE)
        return tau1, tau2, f3
