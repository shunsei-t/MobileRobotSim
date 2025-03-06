import matplotlib.pyplot as plt
import numpy as np
import sys
import matplotlib.animation as animation
import matplotlib.patches as patches
from mrsim.model.diff_drive import DiffDrive

class Viewer:
    def __init__(self, robot):
        self.robot = robot
        self.robot_patches = None
        if isinstance(robot.model, DiffDrive):
            self.robot_patches = diffDrivePathes(robot)
        else:
            sys.exit("Error: A robot model must be DiffDrive class.")

    def plot_animation(self, logger, fps=20.0, path=None, length=0.2, render_noize=False, plot_v_=False):
        self.logger = logger
        self.render_noize = render_noize
        self.plot_v_ = plot_v_

        self.fps = fps
        interval = int(1.0/fps*1000.0)

        # self.fig, self.ax = plt.subplots()
        if plot_v_:
            self.fig, (self.ax, self.ax2, self.ax3) = plt.subplots(3, 1, gridspec_kw={'height_ratios': [3, 1, 1]})
        else:
            self.fig, self.ax = plt.subplots()


        self.ax.set_xlim(-1.0, 1.0)
        self.ax.set_ylim(-1.0, 1.0)
        self.ax.set_aspect('equal')

        if path is not None:
            self.ax.set_xlim(np.min(path[:, 0]) - 2.0, np.max(path[:, 0]) + 2.0)
            self.ax.set_ylim(np.min(path[:, 1]) - 2.0, np.max(path[:, 1]) + 2.0)

            for p in path:
                patch = patches.Arrow(x=p[0],
                                    y=p[1],
                                    dx=length*np.sin(p[2]),
                                    dy=length*np.cos(p[2]),
                                    width=0.1,
                                    edgecolor="#555555")
                self.ax.add_patch(patch)

        self.frame_text = self.ax.text(0.02, 0.95, "time: 0.000", transform=self.ax.transAxes, fontsize=8, verticalalignment='top')

        # 速度プロットの設定
        self.v_ = np.array(self.logger['v'])
        self.ax2.set_xlim(0, len(self.v_))
        self.ax2.set_ylim(np.min(self.v_[:, 0]) - 0.1, np.max(self.v_[:, 0]) + 0.1)
        self.ax2.set_ylabel("Linear")
        self.ax2.set_xlabel("Frame")

        self.ax3.set_xlim(0, len(self.v_))
        self.ax3.set_ylim(np.min(self.v_[:, 2]) - 0.1, np.max(self.v_[:, 2]) + 0.1)
        self.ax3.set_ylabel("Angular")
        # self.ax2.set_xlabel("Frame")

        # 速度プロットのラインオブジェクトを作成
        self.line_v, = self.ax2.plot([], [], 'r-', lw=2)
        self.line_w, = self.ax3.plot([], [], 'r-', lw=2)

        et = 0.0
        fdt = 1.0/fps
        self.skiped_id = []
        for i, t in enumerate(self.logger['time']):
            if t >= et:
                et += fdt
                self.skiped_id.append(i)

        self.ani = animation.FuncAnimation(
            self.fig,
            self.update,
            frames=len(self.skiped_id),
            interval=interval,
            blit=True  # 高速化
        )

        plt.show()



    def update(self, frame):
        """フレームごとにプロットを更新"""

        state = self.logger['state'][self.skiped_id[frame]]
        state_noized = self.logger['state_noized'][self.skiped_id[frame]]

        # ロボットの形状を更新
        self.patches = self.robot_patches.patchesParts(state, "#000000")
        if self.render_noize:
            self.patches += self.robot_patches.patchesParts(state_noized, "#ff0000")

        # すべてのパッチを再描画
        patches_list = []
        for patch in self.patches:
            self.ax.add_patch(patch)
            patches_list.append(patch)

        self.frame_text.set_text(f"time: {self.logger['time'][self.skiped_id[frame]]:.3f}")

        if self.plot_v_:
            # 速度データの更新
            x_data = np.arange(self.skiped_id[frame] + 1)
            y_data = self.v_[:self.skiped_id[frame] + 1, 0]
            self.line_v.set_data(x_data, y_data)

            x_data = np.arange(self.skiped_id[frame] + 1)
            y_data = self.v_[:self.skiped_id[frame] + 1, 2]
            self.line_w.set_data(x_data, y_data)

            return patches_list + [self.line_v, self.line_w, self.frame_text]
        else:
            return patches_list + [self.frame_text]


    def plot_robot(self, x_):
        fig, ax = plt.subplots()

        ax.set_xlim(-1.0, 1.0)
        ax.set_ylim(-1.0, 1.0)
        ax.set_aspect('equal')
        ax.grid()

        patches = self.robot_patches.patchesParts(x_, "#000000")

        for patch in patches:
            ax.add_patch(patch)

        plt.show()


    def plot_path(self, file, length=0.2):
        fig, ax = plt.subplots()

        path = np.loadtxt(file)

        ax.set_aspect('equal')
        # ax.set_adjustable("datalim")

        ax.set_xlim(np.min(path[:, 0]) - 0.5, np.max(path[:, 0]) + 0.5)
        ax.set_ylim(np.min(path[:, 1]) - 0.5, np.max(path[:, 1]) + 0.5)
        ax.grid()

        for p in path:
            patch = patches.Arrow(x=p[0],
                                  y=p[1],
                                  dx=length*np.sin(p[2]),
                                  dy=length*np.cos(p[2]),
                                  width=0.1,
                                  edgecolor="#555555")
            ax.add_patch(patch)

        plt.show()

class diffDrivePathes:
    def __init__(self, robot):
        self.robot = robot

    def patchesParts(self, x_, color):
        """ロボットの形状を更新"""
        tread = self.robot.model.tread
        radius = self.robot.model.radius

        rotate_matrix = np.array([
            [np.cos(x_[2]), -np.sin(x_[2])],
            [np.sin(x_[2]), np.cos(x_[2])]
        ])

        center_ = np.array([x_[0], x_[1]])

        body_orig_ = np.array([-tread/2.0, -tread/2.0])
        body_center_ = center_ + rotate_matrix@body_orig_

        body = patches.Rectangle(
            xy=body_center_,
            angle=np.rad2deg(x_[2]),  # radian を degree に変換
            width=tread,
            height=tread,
            ec=color,
            fill=False
        )

        wheel_thichness = radius/2.0
        l_wheel_orig_x = -radius
        l_wheel_orig_y = -tread/2.0 - wheel_thichness/2.0

        l_wheel_orig_ = np.array([l_wheel_orig_x, l_wheel_orig_y])
        l_wheel_shift_ = center_ + rotate_matrix@l_wheel_orig_.T

        l_wheel = patches.Rectangle(
            xy=l_wheel_shift_,
            angle=np.rad2deg(x_[2]),  # radian を degree に変換
            width=radius*2.0,
            height=wheel_thichness,
            ec=color,
            fill=False
        )

        r_wheel_orig_x = -radius
        r_wheel_orig_y = tread/2.0 - wheel_thichness/2.0
        r_wheel_orig_ = np.array([r_wheel_orig_x, r_wheel_orig_y])
        r_wheel_shift_ = center_ + rotate_matrix@r_wheel_orig_.T

        r_wheel = patches.Rectangle(
            xy=r_wheel_shift_,
            angle=np.rad2deg(x_[2]),  # radian を degree に変換
            width=radius*2.0,
            height=wheel_thichness,
            ec=color,
            fill=False
        )

        edge_top_ = np.array([tread/2.0, 0.0])
        edge_shift_ = center_ + rotate_matrix@edge_top_

        line = patches.Polygon([center_, edge_shift_],
                                    closed=False,
                                    edgecolor=color,
                                    linewidth=1)

        return [body, l_wheel, r_wheel, line]
