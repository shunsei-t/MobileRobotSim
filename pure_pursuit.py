import numpy as np

class PurePursuit:
    def __init__(self, lookahead_distance=0.2, default_linear_speed=0.5, max_linear_speed=5.0, max_angular_speed=6.0):
        self.lookahead_distance = lookahead_distance
        self.default_linear_speed = default_linear_speed
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed

    def control(self, state_, path):
        """
        計算された前進速度 (linear_velocity) と旋回速度 (angular_velocity) を返す
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

        # 速度計算
        linear_velocity = min(self.max_linear_speed, self.default_linear_speed)
        L = dist_[target_id]
        angular_velocity = 2.0*linear_velocity*np.sin(alpha)/L
        angular_velocity = np.clip(angular_velocity, -self.max_angular_speed, self.max_angular_speed)

        return [linear_velocity, 0.0, angular_velocity]
