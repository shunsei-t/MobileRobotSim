import numpy as np

class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(-np.inf, np.inf)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        
        self.previous_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        """
        PID制御の計算を行い、制御出力を返す。
        error: 現在の誤差
        dt: 前回の更新からの時間間隔
        """
        if dt <= 0.0:
            return 0.0

        # 各成分の計算
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = np.clip(output, *self.output_limits)
        
        self.previous_error = error
        return output

class PIDFollower:
    def __init__(self, kp, ki, kd, lookahead_distance=0.2, default_linear_speed=0.5, max_linear_speed=5.0, max_angular_speed=6.0):
        self.pid = PIDController(kp, ki, kd, output_limits=(-max_angular_speed, max_angular_speed))
        self.lookahead_distance = lookahead_distance
        self.default_linear_speed = default_linear_speed
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed

    def control(self, state_, path, dt):
        """
        現在のロボット状態と経路をもとに、速度指令を計算する。
        state: [x, y, theta] (ロボットの現在の位置と向き)
        path: [[x1, y1], [x2, y2], ...] (経路のリスト)
        dt: 時間間隔
        """
        dist_ = np.hypot(path[:, 0]-state_[0], path[:, 1]-state_[1])
        min_id = np.argmin(dist_)
        min_dist = dist_[min_id]

        if min_dist > self.lookahead_distance:
            target_id = min_id
        else:
            target_id = min_id+1

        if target_id >= path.shape[0] - 1:
            return [0.0, 0.0, 0.0]


        azimuth = np.arctan2(path[target_id, 1] - state_[1],
                             path[target_id, 0] - state_[0])
        alpha = azimuth - state_[2]
        alpha = (alpha + np.pi) % (2 * np.pi) - np.pi

        angular_velocity = self.pid.compute(alpha, dt)
        angular_velocity = np.clip(angular_velocity, -self.max_angular_speed, self.max_angular_speed)

        linear_velocity = self.max_linear_speed
        return [linear_velocity, 0.0, angular_velocity]