import math
import sys
from collections import deque


class Robot:
    def __init__(self, model=None, dt=0.01, x_0=[0.0, 0.0, 0.0], deadtime=0.0):
        if model is None:
            sys.exit("Error: A robot model must be provided for 'model'.")

        self.model = model
        self.dt = dt
        self.model.set_dt(dt)

        self.dead_order = 0
        self.u_stack = None
        if deadtime != 0.0:
            if deadtime < dt:
                sys.exit("Error: Deadtime ({deadtime}) must be greater than or equal to the time step ({dt}).")
            else:
                self.dead_order = int(deadtime//self.dt)
                self.u_stack = deque([[0.0, 0.0]]*self.dead_order, maxlen=self.dead_order)

        self.x_t = x_0
        self.v_ = [0.0, 0.0, 0.0]

        self.time = 0.0

    def step(self, u):
        if self.u_stack is not None:
            tmp = u
            u = self.u_stack[0]
            self.u_stack.append(tmp)
        self.v_ = self.model.model(u)

        v_x = self.v_[0]
        v_y = self.v_[1]
        w = self.v_[2]

        # x = self.x_t[0] + v * self.dt * math.cos(self.x_t[2])
        # y = self.x_t[1] + v * self.dt * math.sin(self.x_t[2])
        # theta = self.x_t[2] + w * self.dt

        # 旋回運動を考慮した更新
        if abs(w) < 1e-6:
            # 直線運動 (w ≈ 0)
            x = self.x_t[0] + (v_x * math.cos(self.x_t[2]) - v_y * math.sin(self.x_t[2])) * self.dt
            y = self.x_t[1] + (v_x * math.sin(self.x_t[2]) + v_y * math.cos(self.x_t[2])) * self.dt
            theta = self.x_t[2]
        else:
            # 旋回運動 (w ≠ 0)
            theta_new = self.x_t[2] + w * self.dt
            R_x = (v_x * math.sin(w * self.dt) + v_y * (1 - math.cos(w * self.dt))) / w
            R_y = (v_y * math.sin(w * self.dt) - v_x * (1 - math.cos(w * self.dt))) / w

            x = self.x_t[0] + R_x * math.cos(self.x_t[2]) - R_y * math.sin(self.x_t[2])
            y = self.x_t[1] + R_x * math.sin(self.x_t[2]) + R_y * math.cos(self.x_t[2])
            theta = (theta_new + math.pi) % (2 * math.pi) - math.pi  # -π から π に正規化

        self.x_t = [x, y, theta]
        return self.x_t

    def poseInit(self, x_=[0.0, 0.0, 0.0]):
        self.x_t = x_

    def timeInit(self, time=0.0):
        self.time = time

    def reset(self, x_, time):
        self.poseInit(x_)
        self.timeInit(time)