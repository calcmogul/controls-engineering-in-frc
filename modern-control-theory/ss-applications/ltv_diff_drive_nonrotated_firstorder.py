#!/usr/bin/env python3

# Runs LTV differential drive simulation

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
    import bookutil.latex as latex

import matplotlib.pyplot as plt
import numpy as np

from bookutil.drivetrain import get_diff_vels, differential_drive
from bookutil.systems import LTVDifferentialDrive


class DifferentialDrive(LTVDifferentialDrive):
    def __init__(self, dt, states):
        """Differential drive subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        states -- state vector around which to linearize model
        """
        LTVDifferentialDrive.__init__(self, dt, states)

    def update_plant(self):
        self.sysc = self.create_model(self.x, self.u)
        self.sysd = self.sysc.sample(self.dt)

        self.x = self.sysd.A @ self.x + self.sysd.B @ self.u
        self.y = self.sysd.C @ self.x + self.sysd.D @ self.u

    def update_controller(self, next_r):
        self.design_controller_observer()

        u = self.K @ (self.r - self.x_hat)
        rdot = (next_r - self.r) / self.dt
        uff = self.Kff @ (rdot - self.f(self.x_hat, np.zeros(self.u.shape)))
        self.r = next_r
        self.u = u + uff

        u_cap = np.max(np.abs(self.u))
        if u_cap > 12.0:
            self.u = self.u / u_cap * 12.0


def main():
    t = []
    refs = []

    # Radius of robot in meters
    rb = 0.59055 / 2.0

    with open("ramsete_traj.csv", "r") as trajectory_file:
        import csv

        current_t = 0

        reader = csv.reader(trajectory_file, delimiter=",")
        trajectory_file.readline()
        for row in reader:
            t.append(float(row[0]))
            x = float(row[1])
            y = float(row[2])
            theta = float(row[3])
            vl, vr = get_diff_vels(float(row[4]), float(row[5]), rb * 2.0)
            ref = np.array([[x], [y], [theta], [vl], [vr]])
            refs.append(ref)

    dt = 0.02
    x = np.array([[refs[0][0, 0] + 0.5], [refs[0][1, 0] + 0.5], [np.pi / 2], [0], [0]])
    diff_drive = DifferentialDrive(dt, x)

    state_rec, ref_rec, u_rec, y_rec = diff_drive.generate_time_responses(t, refs)

    plt.figure(1)
    x_rec = np.squeeze(state_rec[0:1, :])
    y_rec = np.squeeze(state_rec[1:2, :])
    plt.plot(x_rec, y_rec, label="LTV controller")
    plt.plot(ref_rec[0, :], ref_rec[1, :], label="Reference trajectory")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.legend()

    # Equalize aspect ratio
    xlim = plt.xlim()
    width = abs(xlim[0]) + abs(xlim[1])
    ylim = plt.ylim()
    height = abs(ylim[0]) + abs(ylim[1])
    if width > height:
        plt.ylim([-width / 2, width / 2])
    else:
        plt.xlim([-height / 2, height / 2])

    if "--noninteractive" in sys.argv:
        latex.savefig("ltv_diff_drive_nonrotated_firstorder_xy")

    diff_drive.plot_time_responses(t, state_rec, ref_rec, u_rec)

    if "--noninteractive" in sys.argv:
        latex.savefig("ltv_diff_drive_nonrotated_firstorder_response")
    else:
        plt.show()


if __name__ == "__main__":
    main()
