import numpy as np

from mrsim import DiffDrive, Robot, Env

from pid_follower import PIDFollower

if __name__ == "__main__":
    # T_l, T_rはそれぞれ左車輪、右車輪のモータの時間定数（一次遅れ系）
    dd = DiffDrive(T_l=0.1, T_r=0.1)
    # dtはロボットの制御周期（秒）
    # deadtimeはロボットの制御遅れ（秒）
    robot = Robot(dd, dt=0.01, deadtime=0.02)
    env = Env(robot, [0.0, 0.05], [0.0, 0.05], [0.0, 0.05])
    # csvファイルから経路を読み込み、envに設定
    path = np.loadtxt("./path/path_l.csv")
    env.set_path(path)
    # PIDFollowerのインスタンスを作成
    # kp, ki, kdはPID制御のゲイン
    # lookahead_distanceは経路追従のための先読み距離
    # max_linear_speedは最大直進速度
    ctrl = PIDFollower(kp=5.0, ki=0.1, kd=0.05, lookahead_distance=0.5, max_linear_speed=3.0)

    steps = 500
    # 初期制御入力（左車輪、右車輪の速度）を設定
    u_ = [0.0, 1.0]

    for i in range(steps):
        # u_の入力を受けっとった時にロボットの状態をシミュレーション
        state_ = env.step(u_)
        # PID制御を実行
        # state_はロボットの状態（位置、姿勢）、pathは経路、robot.dtは制御周期
        v_ = ctrl.control(state_, path, robot.dt)
        # PID制御の出力（[前進速度, 旋回速度]）をロボットの制御入力（[左車輪速度, 右車輪速度]）に変換
        u_ = dd.inv_model(v_)

    env.plot_animation()