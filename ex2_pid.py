import numpy as np

from mrsim import DiffDrive, Robot, Env

from pid_follower import PIDFollower

if __name__ == "__main__":
    dd = DiffDrive(T_l=0.1, T_r=0.1)
    robot = Robot(dd, dt=0.01, deadtime=0.02)
    env = Env(robot, [0.0, 0.05], [0.0, 0.05], [0.0, 0.05])
    path = np.loadtxt("./path/path_l.csv")
    env.set_path(path)
    ctrl = PIDFollower(kp=5.0, ki=0.1, kd=0.05, lookahead_distance=0.5, max_linear_speed=3.0)

    steps = 500
    u_ = [0.0, 1.0]

    for i in range(steps):
        state_ = env.step(u_)
        v_ = ctrl.control(state_, path, robot.dt)
        u_ = dd.inv_model(v_)

    env.plot_animation()