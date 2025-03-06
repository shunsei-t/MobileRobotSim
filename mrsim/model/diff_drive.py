import sys
from mrsim.model.motor import Motor

class DiffDrive:
    def __init__(self,
                 radius = 0.1,
                 tread = 0.5,
                 slip_rate_L=1.0,
                 slip_rate_R=1.0,
                 deadzone_width_L = 0.0,
                 deadzone_width_R = 0.0,
                 max_vel_L=50.0,
                 min_vel_L=-50.0,
                 max_vel_R=50.0,
                 min_vel_R=-50.0,
                 T_l=0.0,
                 T_r=0.0):
        """対向二輪モデル

        Args:
            radius (float, optional): 車輪半径[m]. Defaults to 0.01.
            tread (float, optional): 二輪幅[m]. Defaults to 0.5.
            slip_rate_L (float, optional): 左滑り割合（制御入力への掛け算）. Defaults to 1.0.
            slip_rate_R (float, optional): 右滑り割合（制御入力への掛け算）. Defaults to 1.0.
            deadzone_width_L (float, optional): 左不感帯幅. Defaults to 0.0.
            deadzone_width_R (float, optional): 右不感帯幅. Defaults to 0.0.
            max_vel_L (float, optional): _description_. Defaults to 50.0.
            min_vel_L (float, optional): _description_. Defaults to -50.0.
            max_vel_R (float, optional): _description_. Defaults to 50.0.
            min_vel_R (float, optional): _description_. Defaults to -50.0.
        """
        self.radius = radius
        self.tread = tread
        self.slip_rate_L = slip_rate_L
        self.slip_rate_R = slip_rate_R
        self.deadzone_width_L = deadzone_width_L
        self.deadzone_width_R = deadzone_width_R
        self.max_vel_L = max_vel_L
        self.min_vel_L = min_vel_L
        self.max_vel_R = max_vel_R
        self.min_vel_R = min_vel_R

        self.motor_l = Motor(T_l)
        self.motor_r = Motor(T_r)

        self.u_size = 2


    def set_dt(self, dt):
        self.motor_l.set_dt(dt)
        self.motor_r.set_dt(dt)


    def model(self, u):
        """車輪への制御入力を受け取り、並進、旋回速度を返す関数

        Args:
            u (list): [左タイヤ制御量[rad/s], 右タイヤ制御量[rad/s]]

        Returns:
            v (float): 並進速度[m/s], w (float): 旋回速度[rad/s]
        """

        if len(u) != self.u_size:
            sys.exit(f"Invalid size for argument 'u': expectedx size 2, but got {len(u)}")

        vl, vr = u[0], u[1]
        vl = self.motor_l.step(vl)
        vr = self.motor_r.step(vr)

        vl = self.constrain(vl, self.min_vel_L, self.max_vel_L)
        vr = self.constrain(vr, self.min_vel_R, self.max_vel_R)
        vl = self.deadzone(vl, 0.0, self.deadzone_width_L)
        vr = self.deadzone(vr, 0.0, self.deadzone_width_R)
        vl = vl*self.slip_rate_L
        vr = vr*self.slip_rate_R

        vlw = vl*self.radius
        vrw = vr*self.radius

        v_x = (vlw + vrw)/2.0
        v_y = 0.0
        w = (vrw - vlw)/self.tread

        return [v_x, v_y, w]

    def inv_model(self, v_):
        vl = v_[0] - v_[2]*self.tread/2.0
        vr = v_[0] + v_[2]*self.tread/2.0
        vlw = vl/self.radius
        vrw = vr/self.radius

        return [vlw, vrw]

    def constrain(self, x, min_val, max_val):
        return max(min_val, min(x, max_val))

    def deadzone(self, x, target, width):
        if (x > -width/2.0) and (x < width/2.0):
            return target
        else:
            return x


if __name__ == "__main__":
    dd = DiffDrive()
    dd.model([2, 2, 2])