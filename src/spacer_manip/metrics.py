from __future__ import annotations
import numpy as np


def compute_metrics(time, x_hist, xd_hist, tau_hist, f_contact_hist):
    e = xd_hist - x_hist
    err_norm = np.linalg.norm(e, axis=1)
    rms_err = float(np.sqrt(np.mean(err_norm**2)))
    final_err = float(err_norm[-1])
    peak_contact = float(np.max(np.linalg.norm(f_contact_hist, axis=1)))
    control_energy = float(np.trapz(np.sum(tau_hist**2, axis=1), x=time))
    valid = np.where(time > 0.5)[0]
    settling_idx = valid[err_norm[valid] < 0.02] if len(valid) else np.array([], dtype=int)
    settling_time = float(time[settling_idx[0]]) if len(settling_idx) else float(time[-1])
    corridor_viol = int(np.sum((x_hist[:, 1] < 0.30) | (x_hist[:, 1] > 0.54) | (x_hist[:, 0] > 0.82)))
    return {
        "rms_position_error_m": rms_err,
        "final_position_error_m": final_err,
        "peak_contact_force_N": peak_contact,
        "control_energy": control_energy,
        "settling_time_s": settling_time,
        "constraint_violation_samples": corridor_viol,
    }
