from mrsim import DiffDrive, Robot, Env

if __name__ == "__main__":
    dd = DiffDrive()
    robot = Robot(dd)
    env = Env()

    env.viewer.plot_path("./path_sin.csv")