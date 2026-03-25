from __future__ import annotations
import matplotlib.pyplot as plt
import numpy as np


def plot_trajectory(x_hist, xd_hist, outpath):
    plt.figure(figsize=(6, 5))
    plt.plot(x_hist[:, 0], x_hist[:, 1], label="actual")
    plt.plot(xd_hist[:, 0], xd_hist[:, 1], '--', label="desired")
    plt.axvline(0.82, linestyle=':', label='wall')
    plt.axhline(0.30, linestyle=':')
    plt.axhline(0.54, linestyle=':')
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("End-effector trajectory")
    plt.legend()
    plt.tight_layout()
    plt.savefig(outpath, dpi=180)
    plt.close()


def plot_timeseries(time, err, contact, outpath):
    plt.figure(figsize=(7, 4))
    plt.plot(time, err, label="position error norm")
    plt.plot(time, contact, label="contact force norm")
    plt.xlabel("time [s]")
    plt.ylabel("m / N")
    plt.title("Tracking and contact metrics")
    plt.legend()
    plt.tight_layout()
    plt.savefig(outpath, dpi=180)
    plt.close()
