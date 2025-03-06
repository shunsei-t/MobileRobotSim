class Motor:
    def __init__(self, T=0.0):
        self.T = T

        self.y_0 = 0.0

    def set_dt(self, dt):
        self.dt = dt

    def step(self, u):
        if self.T == 0.0:
            return u
        y_1 = self.y_0 + (self.dt / self.T) * (u - self.y_0)
        self.y_0 = y_1

        return y_1


if __name__ == "__main__":
    m = Motor(1.0)
    m.set_dt(0.1)

    K = 5.0
    for i in range(10):
        y = m.step(K)
        print(y)
