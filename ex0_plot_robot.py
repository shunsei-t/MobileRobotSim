# ロボットのモデルを可視化するサンプルコード

import numpy as np
from mrsim import DiffDrive, Robot, Env

if __name__ == "__main__":
    # DiffDrive（対向二輪車）のインスタンスを作成
    # radiusは車輪の半径、treadは車輪間距離
    dd = DiffDrive(radius=0.1, tread=0.5)
    # Robotのインスタンスを作成
    robot = Robot(dd)
    # Envのインスタンスを作成
    # 引数で環境の外乱を設定
    env = Env(robot)

    # 初期姿勢(0.0, 0.0, 0.1)でロボットを可視化
    env.viewer.plot_robot([0.0, 0.0, 0.1])