import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import re
import os
import glob


def parse_segment_file(filepath):
    """
    Parses a single segment file.
    Returns list of (t, type, val).
    """
    data = []

    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            # Extract timestamp
            match_time = re.search(
                r'\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d+)\]', line)
            if not match_time:
                continue

            dt_str = match_time.group(1)
            try:
                dt = datetime.strptime(dt_str, "%Y-%m-%d %H:%M:%S.%f")
                t_stamp = dt.timestamp()
            except ValueError:
                continue

            # Extract velocities
            match_left = re.search(r'chassis left vel:\s*([-\d\.]+)', line)
            if match_left:
                data.append({
                    't': t_stamp,
                    'type': 'left',
                    'val': float(match_left.group(1))
                })

            match_right = re.search(r'chassis right vel:\s*([-\d\.]+)', line)
            if match_right:
                data.append({
                    't': t_stamp,
                    'type': 'right',
                    'val': float(match_right.group(1))
                })

    return data


def process_channel_fft(data_list, channel_name, target_vel=None):
    points = [d for d in data_list if d['type'] == channel_name]
    if len(points) < 2:
        return None, None, None, None, None

    points.sort(key=lambda x: x['t'])
    t = np.array([p['t'] for p in points])
    y = np.array([p['val'] for p in points])

    # Filter data: 3 seconds AFTER reaching target velocity
    if target_vel is not None and target_vel > 0:
        # Find first index where velocity >= 95% of target
        threshold = target_vel * 0.95
        # Use abs to handle potential negative directions
        idx_reached = np.where(np.abs(y) >= threshold)[0]

        if len(idx_reached) == 0:
            print(
                f"Warning: Target velocity {target_vel} never reached for channel {channel_name}"
            )
            return None, None, None, None, None

        t_reached = t[idx_reached[0]]
        t_start = t_reached + 3.0

        mask = t >= t_start
        t = t[mask]
        y = y[mask]
    elif len(t) > 0:
        # Fallback: Only keep the first 3 seconds of data
        mask = t <= (t[0] + 3.0)
        t = t[mask]
        y = y[mask]

    if len(t) < 2:
        return None, None, None, None, None

    # Calculate sampling rate
    dt_avg = np.mean(np.diff(t))
    if dt_avg <= 0:
        return None, None, None, None, None
    fs = 1.0 / dt_avg

    # Interpolate
    t_fixed = np.arange(t[0], t[-1], dt_avg)
    if len(t_fixed) < 2:
        return None, None, None, None, None
    y_fixed = np.interp(t_fixed, t, y)
    y_mean_removed = y_fixed - np.mean(y_fixed)

    n = len(y_mean_removed)
    fft_val = np.fft.fft(y_mean_removed)
    fft_freq = np.fft.fftfreq(n, d=dt_avg)

    pos_mask = fft_freq > 0
    freqs = fft_freq[pos_mask]
    amps = np.abs(fft_val[pos_mask]) / n * 2

    return freqs, amps, fs, t, y


def analyze_file(filepath, output_dir):
    filename = os.path.basename(filepath)
    name_no_ext = os.path.splitext(filename)[0]

    # Extract target velocity from filename (e.g., ..._v0.3.txt)
    target_vel = None
    match_vel = re.search(r'_v([\d\.]+)\.txt', filename)
    if match_vel:
        try:
            target_vel = float(match_vel.group(1))
        except ValueError:
            pass

    data = parse_segment_file(filepath)
    if not data:
        return None

    freq_l, amp_l, fs_l, t_l, y_l = process_channel_fft(data, 'left',
                                                        target_vel)
    freq_r, amp_r, fs_r, t_r, y_r = process_channel_fft(data, 'right',
                                                        target_vel)

    peak_freq = 0
    max_amp = 0

    # --- Plot Spectrum ---
    plt.figure(figsize=(10, 8))

    # Subplot 1: Frequency Spectrum
    plt.subplot(2, 1, 1)
    if freq_l is not None and amp_l is not None:
        plt.plot(freq_l, amp_l, label='Left FFT', alpha=0.7)
        idx = np.argmax(amp_l)
        if amp_l[idx] > max_amp:
            max_amp = amp_l[idx]
            peak_freq = freq_l[idx]

    if freq_r is not None and amp_r is not None:
        plt.plot(freq_r, amp_r, label='Right FFT', alpha=0.7)
        idx = np.argmax(amp_r)
        if amp_r[idx] > max_amp:
            max_amp = amp_r[idx]
            peak_freq = freq_r[idx]

    plt.title(f"{filename}\nPeak: {peak_freq:.2f} Hz")
    plt.xlabel("Freq (Hz)")
    plt.ylabel("Amp")
    plt.legend()
    plt.grid(True)
    plt.xlim(0, 20)

    # Subplot 2: Velocity vs Time
    plt.subplot(2, 1, 2)
    if t_l is not None:
        # Normalize time start to 0
        plt.plot(t_l - t_l[0], y_l, label='Left Vel', alpha=0.7)
    if t_r is not None:
        # Normalize time start to 0 if possible, or align with Left
        start_t = t_l[0] if t_l is not None else t_r[0]
        plt.plot(t_r - start_t, y_r, label='Right Vel', alpha=0.7)

    plt.title("Velocity vs Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()

    plot_path = os.path.join(output_dir, f"{name_no_ext}_analysis.png")
    plt.savefig(plot_path)
    plt.close()

    return peak_freq, max_amp


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.join(script_dir, "extracted_data")
    plots_dir = os.path.join(script_dir, "plots")

    if not os.path.exists(data_dir):
        print(
            f"Data directory {data_dir} not found. Run split_robot_data.py first."
        )
        return

    if not os.path.exists(plots_dir):
        os.makedirs(plots_dir)

    files = sorted(glob.glob(os.path.join(data_dir, "*.txt")))
    print(f"Found {len(files)} files to analyze.")

    # Store results by category
    results = {}  # key: 'bent_v0.3', val: [freqs]

    for f in files:
        fname = os.path.basename(f)
        print(f"Processing {fname}...")

        # Parse category from filename
        match = re.match(r'([a-z]+)_seg\d+_([a-z0-9\.]+)\.txt', fname)
        category = "unknown"
        if match:
            mode = match.group(1)
            vel = match.group(2)
            category = f"{mode}_{vel}"

        res = analyze_file(f, plots_dir)
        if res:
            freq, amp = res
            # Filter out very low freq noise if needed
            if freq > 0.1:
                if category not in results:
                    results[category] = []
                results[category].append(freq)

    print("\n" + "=" * 50)
    print("Summary of Dominant Frequencies by Category")
    print("=" * 50)

    # Print sorted by key
    for cat in sorted(results.keys()):
        freqs = results[cat]
        avg_f = np.mean(freqs)
        med_f = np.median(freqs)
        std_f = np.std(freqs)

        print(f"Category: {cat} (n={len(freqs)})")
        print(f"  Average: {avg_f:.4f} Hz")
        print(f"  Median:  {med_f:.4f} Hz")
        print(f"  StdDev:  {std_f:.4f} Hz")
        print(f"  Values:  {np.round(freqs, 3)}")
        print("-" * 30)

    print("=" * 50)
    print(f"Plots saved to: {plots_dir}")


if __name__ == "__main__":
    main()
