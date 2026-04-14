import argparse
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt


ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_DIR = str(ROOT_DIR / "Log/preprocess")
DEFAULT_MAX_LAG_US = 500000


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-dir", default=DEFAULT_DIR)
    parser.add_argument("--start-row", type=int, default=0)
    parser.add_argument("--end-row", type=int, default=-1)
    parser.add_argument("--max-lag-us", type=float, default=DEFAULT_MAX_LAG_US)
    parser.add_argument("--segment-count", type=int, default=4)
    return parser.parse_args()


def search_tau(gyro_z, enc1, enc2, max_lag_us, dt_us):
    # Positive lag means encoders are shifted forward.
    max_lag = int(round(max_lag_us / dt_us))
    lags = np.arange(-max_lag, max_lag + 1)
    residuals = np.empty(len(lags))

    for i, lag in enumerate(lags):
        if lag > 0:
            g = gyro_z[:-lag]
            e1 = enc1[lag:]
            e2 = enc2[lag:]
        elif lag < 0:
            g = gyro_z[-lag:]
            e1 = enc1[:lag]
            e2 = enc2[:lag]
        else:
            g = gyro_z
            e1 = enc1
            e2 = enc2

        H = np.column_stack([e1, e2])
        result = np.linalg.lstsq(H, g, rcond=None)
        coeffs = result[0]
        res_list = result[1]
        if len(res_list) > 0:
            residuals[i] = res_list[0]
        else:
            pred = H @ coeffs
            residuals[i] = float(np.dot(g - pred, g - pred))

    best_idx = int(np.argmin(residuals))
    best_lag = int(lags[best_idx])
    best_lag_us = best_lag * dt_us
    return lags, residuals, best_idx, best_lag, best_lag_us


def plot_result(
    time_us, gyro_z, enc1, enc2, lags, residuals,
    best_lag, best_lag_us, g_aligned, pred_aligned, output_path, segment_count
):
    t0 = time_us / 1e6
    norm_residuals = residuals / residuals.max()

    H0 = np.column_stack([enc1, enc2])
    coeffs0, _, _, _ = np.linalg.lstsq(H0, gyro_z, rcond=None)
    pred0 = H0 @ coeffs0

    if best_lag > 0:
        t_aligned = time_us[:-best_lag] / 1e6
    elif best_lag < 0:
        t_aligned = time_us[-best_lag:] / 1e6
    else:
        t_aligned = time_us / 1e6

    plt.figure(figsize=(14, 14))

    plt.subplot(5, 1, 1)
    plt.plot(t0, gyro_z, label="gyro_z", linewidth=1.0)
    plt.plot(t0, pred0, label=f"a·enc1+b·enc2 (τ=0)", linewidth=1.0, alpha=0.8)
    plt.title("Before Alignment  (τ = 0)")
    plt.grid(True, alpha=0.3)
    plt.legend()

    overview_segments = 3
    segment_len = max(1, len(t_aligned) // overview_segments)
    for i in range(overview_segments):
        start = i * segment_len
        end = len(t_aligned) if i == overview_segments - 1 else (i + 1) * segment_len
        plt.subplot(5, 1, 2 + i)
        plt.plot(t_aligned[start:end], g_aligned[start:end], label="gyro_z", linewidth=1.0)
        plt.plot(
            t_aligned[start:end], pred_aligned[start:end],
            label=f"a·enc1+b·enc2 (τ={best_lag_us:.0f} µs)",
            linewidth=1.0, alpha=0.8,
        )
        plt.title(f"Aligned Segment {i + 1}")
        plt.grid(True, alpha=0.3)
        plt.legend()

    plt.subplot(5, 1, 5)
    tau_us_axis = lags * np.median(np.diff(time_us))
    plt.plot(tau_us_axis, norm_residuals, linewidth=1.0)
    plt.axvline(best_lag * np.median(np.diff(time_us)), color="red", linestyle="--", linewidth=1)
    plt.title("Normalised residual J(τ)  [lower = better]")
    plt.xlabel("τ (µs)")
    plt.ylabel("J / J_max")
    plt.grid(True, alpha=0.3)

    plt.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_path, dpi=200)
    plt.close()

    segment_len_full = max(1, len(time_us) // segment_count)
    for i in range(segment_count):
        s = i * segment_len_full
        e = len(time_us) if i == segment_count - 1 else min(len(time_us), (i + 1) * segment_len_full)
        if s >= e:
            break

        seg_t = time_us[s:e] / 1e6
        seg_g = gyro_z[s:e]
        H_seg = np.column_stack([enc1[s:e], enc2[s:e]])
        c_seg, _, _, _ = np.linalg.lstsq(H_seg, seg_g, rcond=None)
        pred_seg0 = H_seg @ c_seg

        if best_lag > 0:
            seg_t_al = time_us[s:e - best_lag] / 1e6
            seg_g_al = gyro_z[s:e - best_lag]
            seg_e1_al = enc1[s + best_lag:e]
            seg_e2_al = enc2[s + best_lag:e]
        elif best_lag < 0:
            seg_t_al = time_us[s - best_lag:e] / 1e6
            seg_g_al = gyro_z[s - best_lag:e]
            seg_e1_al = enc1[s:e + best_lag]
            seg_e2_al = enc2[s:e + best_lag]
        else:
            seg_t_al = seg_t
            seg_g_al = seg_g
            seg_e1_al = enc1[s:e]
            seg_e2_al = enc2[s:e]

        n_al = min(len(seg_g_al), len(seg_e1_al), len(seg_e2_al))
        H_al = np.column_stack([seg_e1_al[:n_al], seg_e2_al[:n_al]])
        c_al, _, _, _ = np.linalg.lstsq(H_al, seg_g_al[:n_al], rcond=None)
        pred_seg_al = H_al @ c_al

        segment_output = output_path.with_name(f"{output_path.stem}_segment_{i + 1}.png")
        plt.figure(figsize=(14, 6))
        plt.subplot(2, 1, 1)
        plt.plot(seg_t, seg_g, label="gyro_z", linewidth=1.0)
        plt.plot(seg_t, pred_seg0, label="a·enc1+b·enc2 (τ=0)", linewidth=1.0, alpha=0.8)
        plt.title(f"Segment {i + 1}  Before Alignment")
        plt.grid(True, alpha=0.3)
        plt.legend()

        plt.subplot(2, 1, 2)
        plt.plot(seg_t_al[:n_al], seg_g_al[:n_al], label="gyro_z", linewidth=1.0)
        plt.plot(seg_t_al[:n_al], pred_seg_al, label="a·enc1+b·enc2 (shifted)", linewidth=1.0, alpha=0.8)
        plt.title(f"Segment {i + 1}  After Alignment")
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        plt.savefig(segment_output, dpi=200)
        plt.close()


def main():
    args = parse_args()
    input_dir = Path(args.input_dir).expanduser().resolve()
    imu = np.atleast_2d(np.loadtxt(input_dir / "imu_zero_us_on_wheel.txt"))
    wheel = np.atleast_2d(np.loadtxt(input_dir / "wheel_zero_us.txt"))

    total = min(len(imu), len(wheel))
    start = max(0, args.start_row)
    end = total if args.end_row < 0 else min(args.end_row, total)
    imu = imu[start:end]
    wheel = wheel[start:end]

    time_us = wheel[:, 0]
    dt_us = float(np.median(np.diff(time_us)))
    gyro_z = imu[:, 3]
    enc1 = wheel[:, 1]
    enc2 = wheel[:, 2]

    lags, residuals, best_idx, best_lag, best_lag_us = search_tau(
        gyro_z, enc1, enc2, args.max_lag_us, dt_us
    )
    if best_lag > 0:
        g_aligned = gyro_z[:-best_lag]
        e1_aligned = enc1[best_lag:]
        e2_aligned = enc2[best_lag:]
    elif best_lag < 0:
        g_aligned = gyro_z[-best_lag:]
        e1_aligned = enc1[:best_lag]
        e2_aligned = enc2[:best_lag]
    else:
        g_aligned = gyro_z
        e1_aligned = enc1
        e2_aligned = enc2

    H_aligned = np.column_stack([e1_aligned, e2_aligned])
    coeffs, _, _, _ = np.linalg.lstsq(H_aligned, g_aligned, rcond=None)
    pred_aligned = H_aligned @ coeffs
    a, b = coeffs
    ss_res = float(np.dot(g_aligned - pred_aligned, g_aligned - pred_aligned))
    ss_tot = float(np.dot(g_aligned - g_aligned.mean(), g_aligned - g_aligned.mean()))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0

    H0 = np.column_stack([enc1, enc2])
    coeffs0, _, _, _ = np.linalg.lstsq(H0, gyro_z, rcond=None)
    pred0 = H0 @ coeffs0
    ss_res0 = float(np.dot(gyro_z - pred0, gyro_z - pred0))
    ss_tot0 = float(np.dot(gyro_z - gyro_z.mean(), gyro_z - gyro_z.mean()))
    r2_zero = 1.0 - ss_res0 / ss_tot0 if ss_tot0 > 0 else 0.0

    print(f"rows_used={len(imu)} dt_us={dt_us:.3f}")
    print(f"best lag_samples={best_lag} lag_us={best_lag_us:.3f}")
    print(f"a={a:.6f} b={b:.6f}")
    print(f"r2={r2:.6f}  r2_at_tau0={r2_zero:.6f}")

    output_path = input_dir / "time_offset_analysis.png"
    plot_result(
        time_us, gyro_z, enc1, enc2,
        lags, residuals, best_lag, best_lag_us,
        g_aligned, pred_aligned,
        output_path, args.segment_count,
    )
    print(f"plot_output={output_path}")

    result_path = input_dir / "time_offset_result.txt"
    result_path.parent.mkdir(parents=True, exist_ok=True)
    with result_path.open("w", encoding="utf-8") as f:
        f.write(f"rows_used={len(imu)}\n")
        f.write(f"dt_us={dt_us:.6f}\n")
        f.write(f"lag_samples={best_lag}\n")
        f.write(f"lag_us={best_lag_us:.6f}\n")
        f.write(f"a={a:.6f}\n")
        f.write(f"b={b:.6f}\n")
        f.write(f"r2={r2:.6f}\n")
        f.write(f"r2_at_tau0={r2_zero:.6f}\n")
        f.write(f"residual_best={residuals[best_idx]:.6f}\n")
    print(f"result_output={result_path}")


if __name__ == "__main__":
    main()
