import numpy as np

from mrsim import DiffDrive, Robot, Env

from pure_pursuit import PurePursuit

if __name__ == "__main__":
    dd = DiffDrive(T_l=0.1, T_r=0.1)
    robot = Robot(dd, dt=0.01, deadtime=0.02)
    env = Env(robot, [0.0, 0.05], [0.0, 0.05], [0.0, 0.05])
    path = np.loadtxt("./path/path_l.csv")
    env.set_path(path)
    ctrl = PurePursuit(lookahead_distance=1.0, default_linear_speed=2.0, max_angular_speed=4.0)

    steps = 500
    u_ = [0.0, 1.0]

    for i in range(steps):
        state_ = env.step(u_)
        v_ = ctrl.control(state_, path)
        u_ = dd.inv_model(v_)

    env.plot_animation()