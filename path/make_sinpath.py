import numpy as np

steps = 30
xyt_ = np.zeros([steps, 3])

l = np.linspace(0.0, 2.0*np.pi, steps)
xyt_ [:,1] = 2.5*np.sin(l)
xyt_ [:,0] = l/(2.0*np.pi)*5.0

for i in range(steps-1):
    xyt_[i, 2] = np.arctan2(xyt_[i+1, 0] - xyt_[i, 0], xyt_[i+1, 1] - xyt_[i, 1])

xyt_[-1, 2] = xyt_[-2, 2]

np.savetxt("path_sin.csv", xyt_)