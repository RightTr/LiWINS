from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
LOG_DIR = ROOT_DIR / "Log/calib/20260415_001248"
PLOT_DIR = LOG_DIR / "plots"


def load_txt(path):
    return np.loadtxt(path, comments="#")


def normalize_time(*arrays):
    t0 = min(arr[0, 0] for arr in arrays if arr.size > 0)
    for arr in arrays:
        arr[:, 0] -= t0
    return t0


def plot_imu_wheel_position(imu_state, wheel_integration, out_path):
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    axes[0].plot(imu_state[:, 0], imu_state[:, 1], label="imu_x", linewidth=1.5)
    axes[0].plot(wheel_integration[:, 0], wheel_integration[:, 1], label="wheel_x", linewidth=1.5)
    axes[0].set_ylabel("x (m)")
    axes[0].set_title("IMU / Wheel X vs Time")
    axes[0].grid(True, linestyle="--", alpha=0.4)
    axes[0].legend()

    axes[1].plot(imu_state[:, 0], imu_state[:, 2], label="imu_y", linewidth=1.5)
    axes[1].plot(wheel_integration[:, 0], wheel_integration[:, 2], label="wheel_y", linewidth=1.5)
    axes[1].set_xlabel("time (s)")
    axes[1].set_ylabel("y (m)")
    axes[1].set_title("IMU / Wheel Y vs Time")
    axes[1].grid(True, linestyle="--", alpha=0.4)
    axes[1].legend()

    fig.tight_layout()
    fig.savefig(out_path, dpi=200)
    plt.close(fig)


def plot_wheel_state(wheel_state, out_path):
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    labels = [("theta (rad)", 3, "Wheel Theta vs Time"),
              ("sr", 4, "Wheel sr vs Time"),
              ("sl", 5, "Wheel sl vs Time")]

    for ax, (ylabel, col, title) in zip(axes, labels):
        ax.plot(wheel_state[:, 0], wheel_state[:, col], linewidth=1.5)
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(True, linestyle="--", alpha=0.4)

    axes[-1].set_xlabel("time (s)")
    fig.tight_layout()
    fig.savefig(out_path, dpi=200)
    plt.close(fig)


def plot_2d_pose_trajectory(imu_state, wheel_plot, ate, out_path):
    fig, ax = plt.subplots(figsize=(12, 8))

    imu_x = imu_state[:, 1]
    imu_y = imu_state[:, 2]
    wheel_x = wheel_plot[:, 1]
    wheel_y = wheel_plot[:, 2]

    ax.plot(
        imu_x,
        imu_y,
        label="IMU trajectory",
        linewidth=1.8,
        color="0.75",
        linestyle="--",
    )
    ax.plot(
        wheel_x,
        wheel_y,
        label="Wheel trajectory",
        linewidth=2.0,
        color="tab:green",
        linestyle="--",
    )

    ax.scatter(imu_x[0], imu_y[0], color="0.75", marker="o", s=50, label="IMU start")
    ax.scatter(imu_x[-1], imu_y[-1], color="0.75", marker="x", s=60, label="IMU end")
    ax.scatter(
        wheel_x[0], wheel_y[0], color="tab:green", marker="o", s=50, label="Wheel start"
    )
    ax.scatter(
        wheel_x[-1], wheel_y[-1], color="tab:green", marker="x", s=60, label="Wheel end"
    )

    ax.set_title("2D Translation Trajectory")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    all_x = np.concatenate((imu_x, wheel_x))
    all_y = np.concatenate((imu_y, wheel_y))
    x_span = np.ptp(all_x)
    y_span = np.ptp(all_y)
    x_margin = max(x_span * 0.05, 0.05)
    y_margin = max(y_span * 0.1, 0.05)
    ax.set_xlim(all_x.min() - x_margin, all_x.max() + x_margin)
    ax.set_ylim(all_y.min() - y_margin, all_y.max() + y_margin)
    ax.set_aspect("auto")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend()
    ax.text(
        0.02,
        0.98,
        f"ATE RMSE: {ate['rmse']:.3f} m\nATE Mean: {ate['mean']:.3f} m\nATE Max: {ate['max']:.3f} m",
        transform=ax.transAxes,
        ha="left",
        va="top",
        bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "0.8"},
    )

    fig.tight_layout()
    fig.savefig(out_path, dpi=200)
    plt.close(fig)


def align_wheel_translation(imu_state, wheel_integration):
    wheel_mask = (
        (wheel_integration[:, 0] >= imu_state[0, 0])
        & (wheel_integration[:, 0] <= imu_state[-1, 0])
    )
    wheel_aligned = wheel_integration[wheel_mask]
    imu_x = np.interp(wheel_aligned[:, 0], imu_state[:, 0], imu_state[:, 1])
    imu_y = np.interp(wheel_aligned[:, 0], imu_state[:, 0], imu_state[:, 2])
    offset = np.array([imu_x[0] - wheel_aligned[0, 1], imu_y[0] - wheel_aligned[0, 2]])
    return wheel_aligned, imu_x, imu_y, offset


def compute_ate(imu_state, wheel_integration):
    wheel_aligned, imu_x, imu_y, offset = align_wheel_translation(imu_state, wheel_integration)
    wheel_x = wheel_aligned[:, 1] + offset[0]
    wheel_y = wheel_aligned[:, 2] + offset[1]
    err = np.sqrt((imu_x - wheel_x) ** 2 + (imu_y - wheel_y) ** 2)
    return {
        "count": len(err),
        "rmse": np.sqrt(np.mean(err ** 2)),
        "mean": np.mean(err),
        "max": np.max(err),
        "offset_x": offset[0],
        "offset_y": offset[1],
    }


def main():
    imu_state_path = LOG_DIR / "liw_calib_imu_state.txt"
    wheel_state_path = LOG_DIR / "liw_calib_wheel_state.txt"
    wheel_integration_path = LOG_DIR / "liw_calib_wheel_integration.txt"
    PLOT_DIR.mkdir(parents=True, exist_ok=True)

    imu_state = load_txt(imu_state_path)
    wheel_state = load_txt(wheel_state_path)
    wheel_integration = load_txt(wheel_integration_path)

    normalize_time(imu_state, wheel_state, wheel_integration)

    saved_paths = [
        PLOT_DIR / "imu_wheel_position_vs_time.png",
        PLOT_DIR / "wheel_theta_sr_sl_vs_time.png",
        PLOT_DIR / "pose_trajectory_2d.png",
    ]

    ate = compute_ate(imu_state, wheel_integration)
    wheel_plot = wheel_integration.copy()
    wheel_plot[:, 1] += ate["offset_x"]
    wheel_plot[:, 2] += ate["offset_y"]
    plot_imu_wheel_position(imu_state, wheel_integration, saved_paths[0])
    plot_wheel_state(wheel_state, saved_paths[1])
    plot_2d_pose_trajectory(imu_state, wheel_plot, ate, saved_paths[2])

    for path in saved_paths:
        print(f"saved: {path}")
    print(
        f"ate_2d count={ate['count']} rmse={ate['rmse']:.6f} "
        f"mean={ate['mean']:.6f} max={ate['max']:.6f}"
    )

if __name__ == "__main__":
    main()
