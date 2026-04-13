from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
LOG_DIR = ROOT_DIR / "Log/20260413_110702_099"
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

    print(f"saved: {PLOT_DIR / 'imu_wheel_position_vs_time.png'}")
    print(f"saved: {PLOT_DIR / 'wheel_theta_sr_sl_vs_time.png'}")

if __name__ == "__main__":
    main()
