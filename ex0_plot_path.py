# 経路をプロットするサンプルコード

from mrsim import DiffDrive, Robot, Env

if __name__ == "__main__":
    dd = DiffDrive()
    robot = Robot(dd)
    env = Env(robot)

    # csvファイルから経路を読み込み、可視化
    env.viewer.plot_path("./path/path_sin.csv")