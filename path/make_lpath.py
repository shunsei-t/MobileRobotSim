import numpy as np

steps = 30
xyt_ = np.zeros([steps, 3])

l = np.linspace(0.0, 5.0, 15)
xyt_[:15, 1] = l
xyt_[:15, 0] = 0.0

xyt_[15:, 1] = 5.0
xyt_[15:, 0] = l


for i in range(steps-1):
    xyt_[i, 2] = np.arctan2(xyt_[i+1, 0] - xyt_[i, 0], xyt_[i+1, 1] - xyt_[i, 1])

xyt_[-1, 2] = xyt_[-2, 2]

np.savetxt("path_l.csv", xyt_)