from __future__ import annotations
import json
import os
import sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from spacer_manip.dynamics import fk, jacobian, step
from spacer_manip.tasks import ConstrainedApproachTask
from spacer_manip.controllers import CartesianImpedanceController, JointPDController
from spacer_manip.metrics import compute_metrics
from spacer_manip.visualize import plot_trajectory, plot_timeseries


def inverse_kinematics(x, lengths=(0.7, 0.5)):
    l1, l2 = lengths
    x0, y0 = x
    c2 = (x0**2 + y0**2 - l1**2 - l2**2) / (2 * l1 * l2)
    c2 = np.clip(c2, -1.0, 1.0)
    q2 = np.arccos(c2)
    q1 = np.arctan2(y0, x0) - np.arctan2(l2 * np.sin(q2), l1 + l2 * np.cos(q2))
    return np.array([q1, q2])


def simulate(controller_name: str, dt=0.002, horizon=4.0):
    task = ConstrainedApproachTask()
    q = np.array([0.0, 1.8])
    dq = np.zeros(2)
    x0 = fk(q)
    if controller_name == "impedance":
        controller = CartesianImpedanceController(kp=130.0, kd=30.0, torque_limit=18.0)
    elif controller_name == "joint_pd":
        q_goal = inverse_kinematics(task.target)
        controller = JointPDController(q_goal=q_goal, kp=14.0, kd=4.2, torque_limit=18.0)
    else:
        raise ValueError(controller_name)

    time = np.arange(0.0, horizon + dt, dt)
    q_hist, x_hist, xd_hist, tau_hist, f_contact_hist = [], [], [], [], []

    for t in time:
        x = fk(q)
        J = jacobian(q)
        dx = J @ dq
        xd, vd = task.desired_trajectory(t, x0)
        tau_cmd = controller(q, dq, xd, vd)
        f_contact = task.contact_force(x, dx)
        tau_env = J.T @ f_contact
        tau_total = tau_cmd + tau_env
        q, dq, _ = step(q, dq, tau_total, dt)
        q_hist.append(q.copy())
        x_hist.append(x.copy())
        xd_hist.append(xd.copy())
        tau_hist.append(tau_cmd.copy())
        f_contact_hist.append(f_contact.copy())

    return {
        "time": time,
        "q_hist": np.array(q_hist),
        "x_hist": np.array(x_hist),
        "xd_hist": np.array(xd_hist),
        "tau_hist": np.array(tau_hist),
        "f_contact_hist": np.array(f_contact_hist),
        "metrics": compute_metrics(time, np.array(x_hist), np.array(xd_hist), np.array(tau_hist), np.array(f_contact_hist)),
    }


def main():
    outdir = ROOT / "results"
    os.makedirs(outdir, exist_ok=True)

    all_results = {}
    for name in ["impedance", "joint_pd"]:
        res = simulate(name)
        all_results[name] = res["metrics"]
        np.savez(
            outdir / f"{name}_run.npz",
            time=res["time"], x_hist=res["x_hist"], xd_hist=res["xd_hist"],
            tau_hist=res["tau_hist"], f_contact_hist=res["f_contact_hist"]
        )
        err = np.linalg.norm(res["xd_hist"] - res["x_hist"], axis=1)
        contact = np.linalg.norm(res["f_contact_hist"], axis=1)
        plot_trajectory(res["x_hist"], res["xd_hist"], outdir / f"{name}_trajectory.png")
        plot_timeseries(res["time"], err, contact, outdir / f"{name}_timeseries.png")

    with open(outdir / "metrics.json", "w", encoding="utf-8") as f:
        json.dump(all_results, f, indent=2)

    # markdown table for README
    with open(outdir / "metrics_table.md", "w", encoding="utf-8") as f:
        f.write("| Controller | RMS pos. error [m] | Final error [m] | Peak contact [N] | Settling [s] | Constraint violations |\n")
        f.write("|---|---:|---:|---:|---:|---:|\n")
        for ctrl, m in all_results.items():
            f.write(
                f"| {ctrl} | {m['rms_position_error_m']:.4f} | {m['final_position_error_m']:.4f} | {m['peak_contact_force_N']:.2f} | {m['settling_time_s']:.2f} | {m['constraint_violation_samples']} |\n"
            )

if __name__ == "__main__":
    main()
