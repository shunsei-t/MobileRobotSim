import numpy as np
from mrsim import Env, Robot, DiffDrive

if __name__ == "__main__":
    dd = DiffDrive()
    robot = Robot(dd)
    # Env インスタンスに計測ノイズを追加
    env = Env(robot, [0.0, 0.1], [0.0, 0.1], [0.0, 0.1])

    # 左右輪へ100ステップ分の指令を作成
    steps = 100
    u_ = np.ones([steps, 2])
    u_[:, 0] = u_[:, 0]*10.0
    u_[:, 1] = u_[:, 1]*5.0

    # シミュレーションの実効
    for i in range(steps):
        state_ = env.step(u_[i])

    # シミュレーションの可視化
    # render_noize=Trueで計測ノイズを可視化
    env.plot_animation(render_noize=True)