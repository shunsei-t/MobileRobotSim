import random
import sys
from mrsim.viewer import Viewer
import copy

class Env:
    def __init__(self, robot=None,
                 obs_x_noize=[0.0, 0.0],
                 obs_y_noize=[0.0, 0.0],
                 obs_theta_noize=[0.0, 0.0]):

        if robot is None:
            sys.exit("Error: A robot must be provided for 'robot'.")

        self.robot = robot
        self.obs_x_noize = obs_x_noize
        self.obs_y_noize = obs_y_noize
        self.obs_theta_noize = obs_theta_noize

        self.logger_init = {'time':[], 'state': [], 'state_noized': [], 'u':[], 'v':[]}
        self.logger = self.logger_init

        self.viewer = Viewer(robot)

        self.path = None
        self.time = 0.0

    def step(self, u):
        state = self.robot.step(u)

        state_noized = [0, 0, 0]
        state_noized[0] = state[0] + random.gauss(mu=self.obs_x_noize[0], sigma=self.obs_x_noize[1])
        state_noized[1] = state[1] + random.gauss(mu=self.obs_y_noize[0], sigma=self.obs_y_noize[1])
        state_noized[2] = state[2] + random.gauss(mu=self.obs_theta_noize[0], sigma=self.obs_theta_noize[1])

        self.logger['time'].append(self.time)
        self.logger['state'].append(state)
        self.logger['state_noized'].append(state_noized)
        self.logger['u'].append(u)
        self.logger['v'].append(copy.copy(self.robot.v_))

        self.time += self.robot.dt

        return state_noized

    def reset(self, x_=[0.0, 0.0, 0.0], time=0.0):
        self.logger = self.logger_init

        self.robot.reset(x_, time)

    def set_path(self, path):
        self.path = path

    def plot_animation(self, fps=20.0, render_noize = False, plot_v_=True):

        if self.path is None:
            self.viewer.plot_animation(logger=self.logger, fps=fps, render_noize=render_noize, plot_v_=plot_v_)
        else:
            self.viewer.plot_animation(logger=self.logger, fps=fps, path=self.path, render_noize=render_noize, plot_v_=plot_v_)

    def plot_animation_with(self, render_noize = False):
        interval_ms = int(self.robot.dt*1000.0)

        if self.path is None:
            self.viewer.plot_animation_with(self.logger, interval_ms, render_noize=render_noize)
        else:
            self.viewer.plot_animation_with(self.logger, interval_ms, self.path, render_noize=render_noize)


