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

    # Filter data: from peak velocity point to max 3 seconds after
    if target_vel is not None and target_vel > 0:
        # Find the peak velocity
        peak_vel = np.max(np.abs(y))
        peak_threshold = peak_vel * 0.95

        # Find first index where velocity >= 95% of peak velocity
        idx_reached = np.where(np.abs(y) >= peak_threshold)[0]
        if len(idx_reached) == 0:
            print(
                f"Warning: Peak velocity threshold {peak_threshold} never reached for channel {channel_name}"
            )
            return None, None, None, None, None

        idx_start = idx_reached[0]

        # End point: maximum 3 seconds after start
        time_limit = t[idx_start] + 3.0
        idx_end = np.where(t <= time_limit)[0][-1]  # Last index within time limit

        # Use data from peak velocity to 3 seconds after
        mask = (t >= t[idx_start]) & (t <= t[idx_end])
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

    # Calculate start and end points for marking on plot
    left_start_idx = None
    left_end_idx = None
    right_start_idx = None
    right_end_idx = None

    if t_l is not None and len(t_l) > 0:
        points_left = [d for d in data if d['type'] == 'left']
        points_left.sort(key=lambda x: x['t'])
        y_full_left = np.array([p['val'] for p in points_left])
        t_full_left = np.array([p['t'] for p in points_left])

        peak_vel_left = np.max(np.abs(y_full_left))
        peak_threshold_left = peak_vel_left * 0.95
        idx_reached = np.where(np.abs(y_full_left) >= peak_threshold_left)[0]
        if len(idx_reached) > 0:
            left_start_idx = idx_reached[0]
            # End point: 3 seconds after start
            time_limit = t_full_left[left_start_idx] + 3.0
            idx_end_candidates = np.where(t_full_left <= time_limit)[0]
            if len(idx_end_candidates) > 0:
                left_end_idx = idx_end_candidates[-1]

    if t_r is not None and len(t_r) > 0:
        points_right = [d for d in data if d['type'] == 'right']
        points_right.sort(key=lambda x: x['t'])
        y_full_right = np.array([p['val'] for p in points_right])
        t_full_right = np.array([p['t'] for p in points_right])

        peak_vel_right = np.max(np.abs(y_full_right))
        peak_threshold_right = peak_vel_right * 0.95
        idx_reached = np.where(np.abs(y_full_right) >= peak_threshold_right)[0]
        if len(idx_reached) > 0:
            right_start_idx = idx_reached[0]
            # End point: 3 seconds after start
            time_limit = t_full_right[right_start_idx] + 3.0
            idx_end_candidates = np.where(t_full_right <= time_limit)[0]
            if len(idx_end_candidates) > 0:
                right_end_idx = idx_end_candidates[-1]

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
    plt.xlim(0, 10)
    plt.xticks(np.arange(0, 11, 1))  # Set ticks every 1 Hz from 0 to 10

    # Subplot 2: Velocity vs Time
    plt.subplot(2, 1, 2)

    # Plot full data for both channels
    points_left = [d for d in data if d['type'] == 'left']
    points_right = [d for d in data if d['type'] == 'right']

    if len(points_left) > 0:
        points_left.sort(key=lambda x: x['t'])
        t_full_left = np.array([p['t'] for p in points_left])
        y_full_left = np.array([p['val'] for p in points_left])
        plt.plot(t_full_left - t_full_left[0], y_full_left, label='Left Vel', alpha=0.7)

    if len(points_right) > 0:
        points_right.sort(key=lambda x: x['t'])
        t_full_right = np.array([p['t'] for p in points_right])
        y_full_right = np.array([p['val'] for p in points_right])
        start_t = t_full_left[0] if len(points_left) > 0 else t_full_right[0]
        plt.plot(t_full_right - start_t, y_full_right, label='Right Vel', alpha=0.7)

        # Mark start and end points for left channel
        if len(points_left) > 0 and left_start_idx is not None:
            start_marker_t = t_full_left[left_start_idx] - t_full_left[0]
            start_marker_y = y_full_left[left_start_idx]
            plt.plot(start_marker_t, start_marker_y, 'go', markersize=8, label='Left Start')

            if left_end_idx is not None:
                end_marker_t = t_full_left[left_end_idx] - t_full_left[0]
                end_marker_y = y_full_left[left_end_idx]
                plt.plot(end_marker_t, end_marker_y, 'ro', markersize=8, label='Left End')

        # Mark start and end points for right channel
        if right_start_idx is not None:
            start_marker_t = t_full_right[right_start_idx] - start_t
            start_marker_y = y_full_right[right_start_idx]
            plt.plot(start_marker_t, start_marker_y, 'g^', markersize=8, label='Right Start')

            if right_end_idx is not None:
                end_marker_t = t_full_right[right_end_idx] - start_t
                end_marker_y = y_full_right[right_end_idx]
                plt.plot(end_marker_t, end_marker_y, 'r^', markersize=8, label='Right End')

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
