import numpy as np

from mrsim import DiffDrive, Robot, Env

if __name__ == "__main__":
    dd = DiffDrive()
    robot = Robot(dd)
    env = Env(robot, [0.0, 0.1], [0.0, 0.1], [0.0, 0.1])

    env.viewer.plot_robot([0.0, 0.0, 0.1])