import argparse
from pathlib import Path

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_DIR = str(ROOT_DIR / "Log/preprocess")
DEFAULT_WINDOW_SIZE = 21


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-dir", default=DEFAULT_DIR)
    parser.add_argument("--window-size", type=int, default=DEFAULT_WINDOW_SIZE)
    return parser.parse_args()


def trim_wheel_prefix(wheel_data):
    if len(wheel_data) < 3:
        return wheel_data

    dt = np.diff(wheel_data[:, 0])
    median_dt = np.median(dt)
    jump_idx = np.where(dt > 100 * median_dt)[0]
    if len(jump_idx) == 0:
        return wheel_data
    return wheel_data[jump_idx[0] + 1 :]


def moving_average(values, window_size):
    if window_size <= 1:
        return values.copy()
    if window_size % 2 == 0:
        raise ValueError(f"window_size must be odd, got {window_size}")

    radius = window_size // 2
    prefix = np.zeros(len(values) + 1, dtype=float)
    prefix[1:] = np.cumsum(values)

    filtered = np.empty_like(values, dtype=float)
    for idx in range(len(values)):
        left = max(0, idx - radius)
        right = min(len(values), idx + radius + 1)
        filtered[idx] = (prefix[right] - prefix[left]) / (right - left)
    return filtered


def lowpass_wheel(wheel_data, window_size):
    output = wheel_data.copy()
    output[:, 1] = moving_average(wheel_data[:, 1], window_size)
    output[:, 2] = moving_average(wheel_data[:, 2], window_size)
    return output


def interpolate_imu_to_wheel(imu_data, wheel_data):
    imu_times = imu_data[:, 0]
    wheel_times = wheel_data[:, 0]
    overlap_mask = (wheel_times >= imu_times[0]) & (wheel_times <= imu_times[-1])

    if not np.any(overlap_mask):
        raise ValueError("No overlapping time range between IMU and wheel data.")

    synced_wheel = wheel_data[overlap_mask].copy()
    synced_times = synced_wheel[:, 0]
    synced_imu = np.column_stack(
        [synced_times]
        + [np.interp(synced_times, imu_times, imu_data[:, col]) for col in range(1, imu_data.shape[1])]
    )
    return synced_imu, synced_wheel


def preprocess(input_dir, imu_raw, wheel_raw, window_size):
    wheel_raw = trim_wheel_prefix(wheel_raw)

    imu_aligned = imu_raw.copy()
    imu_aligned[:, 0] = imu_raw[:, 0] * 1_000_000.0
    imu_aligned[:, 0] -= imu_aligned[0, 0]

    wheel_aligned = wheel_raw.copy()
    wheel_aligned[:, 0] -= wheel_aligned[0, 0]
    wheel_lowpass = lowpass_wheel(wheel_aligned, window_size)
    imu_sync, wheel_sync = interpolate_imu_to_wheel(imu_aligned, wheel_lowpass)

    outputs = {
        "imu_sync_output": input_dir / "imu_zero_us_on_wheel.txt",
        "wheel_sync_output": input_dir / "wheel_zero_us.txt",
    }

    for key, data in (
        ("imu_sync_output", imu_sync),
        ("wheel_sync_output", wheel_sync),
    ):
        path = outputs[key]
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8") as file:
            for row in data:
                file.write(" ".join(f"{value:.6f}" for value in row) + "\n")

    return outputs


def main():
    args = parse_args()
    input_dir = Path(args.input_dir).expanduser().resolve()
    imu_raw = np.atleast_2d(np.loadtxt(input_dir / "imu_raw.txt"))
    wheel_raw = np.atleast_2d(np.loadtxt(input_dir / "wheel_raw.txt"))

    outputs = preprocess(
        input_dir=input_dir,
        imu_raw=imu_raw,
        wheel_raw=wheel_raw,
        window_size=args.window_size,
    )

    for name, path in outputs.items():
        print(f"{name}={path}")


if __name__ == "__main__":
    main()
