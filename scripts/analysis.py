from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
LOG_DIR = ROOT_DIR / "Log/20260414_002507"
PLOT_DIR = LOG_DIR / "plots"


def load_txt(path: Path) -> np.ndarray:
    return np.loadtxt(path, comments="#")


def normalize_time(*arrays: np.ndarray) -> float:
    t0 = min(arr[0, 0] for arr in arrays if arr.size > 0)
    for arr in arrays:
        if arr.size > 0:
            arr[:, 0] -= t0
    return t0


def plot_imu_wheel_position(imu_state: np.ndarray, wheel_integration: np.ndarray, out_path: Path) -> None:
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


def plot_wheel_state(wheel_state: np.ndarray, out_path: Path) -> None:
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


def plot_2d_pose_trajectory(
    imu_state: np.ndarray,
    wheel_state: np.ndarray,
    wheel_integration: np.ndarray,
    out_path: Path,
) -> None:
    fig, ax = plt.subplots(figsize=(10, 10))

    imu_x = imu_state[:, 1]
    imu_y = imu_state[:, 2]
    wheel_x = wheel_integration[:, 1]
    wheel_y = wheel_integration[:, 2]

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
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend()

    fig.tight_layout()
    fig.savefig(out_path, dpi=200)
    plt.close(fig)


def main() -> None:
    imu_state_path = LOG_DIR / "liw_calib_imu_state.txt"
    wheel_state_path = LOG_DIR / "liw_calib_wheel_state.txt"
    wheel_integration_path = LOG_DIR / "liw_calib_wheel_integration.txt"
    PLOT_DIR.mkdir(parents=True, exist_ok=True)

    imu_state = load_txt(imu_state_path)
    wheel_state = load_txt(wheel_state_path)
    wheel_integration = load_txt(wheel_integration_path)

    normalize_time(imu_state, wheel_state, wheel_integration)

    plot_imu_wheel_position(
        imu_state,
        wheel_integration,
        PLOT_DIR / "imu_wheel_position_vs_time.png",
    )
    plot_wheel_state(
        wheel_state,
        PLOT_DIR / "wheel_theta_sr_sl_vs_time.png",
    )
    plot_2d_pose_trajectory(
        imu_state,
        wheel_state,
        wheel_integration,
        PLOT_DIR / "pose_trajectory_2d.png",
    )

    print(f"saved: {PLOT_DIR / 'imu_wheel_position_vs_time.png'}")
    print(f"saved: {PLOT_DIR / 'wheel_theta_sr_sl_vs_time.png'}")
    print(f"saved: {PLOT_DIR / 'pose_trajectory_2d.png'}")

if __name__ == "__main__":
    main()
