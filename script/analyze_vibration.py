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


def process_channel_fft(data_list, channel_name, target_vel=None, phase='rise'):
    points = [d for d in data_list if d['type'] == channel_name]
    if len(points) < 2:
        return None, None, None, None, None

    points.sort(key=lambda x: x['t'])
    t = np.array([p['t'] for p in points])
    y = np.array([p['val'] for p in points])

    if target_vel is not None and target_vel > 0:
        if phase == 'rise':
            # Rising phase: from peak velocity point to max 3 seconds after
            peak_vel = np.max(np.abs(y))
            peak_threshold = peak_vel * 0.95

            idx_reached = np.where(np.abs(y) >= peak_threshold)[0]
            if len(idx_reached) == 0:
                print(
                    f"Warning: Peak velocity threshold {peak_threshold} never reached for channel {channel_name}"
                )
                return None, None, None, None, None

            idx_start = idx_reached[0]
            time_limit = t[idx_start] + 3.0
            idx_end = np.where(t <= time_limit)[0][-1]

        elif phase == 'fall':
            # Falling phase: find when velocity drops to 5% of peak
            peak_vel = np.max(np.abs(y))
            peak_threshold = peak_vel * 0.95

            # Find all points that reached at least 95% of peak
            peak_region = np.where(np.abs(y) >= peak_threshold)[0]

            if len(peak_region) == 0:
                print(
                    f"Warning: Peak region never reached for channel {channel_name}"
                )
                return None, None, None, None, None

            # Find when velocity drops to 5% of peak
            fall_threshold = peak_vel * 0.05
            idx_dropped = np.where(np.abs(y) <= fall_threshold)[0]

            # Only consider points after the peak region
            valid_drop_indices = idx_dropped[idx_dropped > peak_region[-1]]

            if len(valid_drop_indices) == 0:
                print(
                    f"Warning: Velocity never dropped to 5% of peak for channel {channel_name}"
                )
                return None, None, None, None, None

            idx_start = valid_drop_indices[0]
            time_limit = t[idx_start] + 3.0

            # Find the last available point within time limit, or use the very last point if less than 3 seconds available
            idx_end_candidates = np.where(t <= time_limit)[0]
            if len(idx_end_candidates) > 0:
                idx_end = idx_end_candidates[-1]
            else:
                idx_end = len(t) - 1  # Use the very last point

        # Use filtered data based on phase
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

    # Process both rising and falling phases
    freq_l_rise, amp_l_rise, fs_l_rise, t_l_rise, y_l_rise = process_channel_fft(
        data, 'left', target_vel, phase='rise')
    freq_r_rise, amp_r_rise, fs_r_rise, t_r_rise, y_r_rise = process_channel_fft(
        data, 'right', target_vel, phase='rise')

    freq_l_fall, amp_l_fall, fs_l_fall, t_l_fall, y_l_fall = process_channel_fft(
        data, 'left', target_vel, phase='fall')
    freq_r_fall, amp_r_fall, fs_r_fall, t_r_fall, y_r_fall = process_channel_fft(
        data, 'right', target_vel, phase='fall')

    # Calculate start and end points for marking on plot
    left_start_idx = None
    left_end_idx = None
    right_start_idx = None
    right_end_idx = None
    left_fall_idx = None
    right_fall_idx = None

    if t_l_rise is not None and len(t_l_rise) > 0:
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

        # Find fall start point
        fall_threshold_left = np.max(np.abs(y_full_left)) * 0.05
        fall_mask_left = np.abs(y_full_left) <= fall_threshold_left
        fall_indices_left = np.where(fall_mask_left)[0]

        # Only consider fall after peak region
        peak_region_left = np.where(np.abs(y_full_left) >= (np.max(np.abs(y_full_left)) * 0.95))[0]
        if len(peak_region_left) > 0 and len(fall_indices_left) > 0:
            last_peak_idx = peak_region_left[-1]
            valid_fall_indices = fall_indices_left[fall_indices_left > last_peak_idx]
            if len(valid_fall_indices) > 0:
                left_fall_idx = valid_fall_indices[0]

    if t_r_rise is not None and len(t_r_rise) > 0:
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

        # Find fall start point
        fall_threshold_right = np.max(np.abs(y_full_right)) * 0.05
        fall_mask_right = np.abs(y_full_right) <= fall_threshold_right
        fall_indices_right = np.where(fall_mask_right)[0]

        # Only consider fall after peak region
        peak_region_right = np.where(np.abs(y_full_right) >= (np.max(np.abs(y_full_right)) * 0.95))[0]
        if len(peak_region_right) > 0 and len(fall_indices_right) > 0:
            last_peak_idx = peak_region_right[-1]
            valid_fall_indices = fall_indices_right[fall_indices_right > last_peak_idx]
            if len(valid_fall_indices) > 0:
                right_fall_idx = valid_fall_indices[0]

    # Find peak frequencies for both phases
    rise_peak_freq = 0
    rise_max_amp = 0
    fall_peak_freq = 0
    fall_max_amp = 0

    # --- Plot Spectrum ---
    plt.figure(figsize=(12, 10))

    # Subplot 1: Rising Phase Frequency Spectrum
    plt.subplot(3, 1, 1)
    if freq_l_rise is not None and amp_l_rise is not None:
        plt.plot(freq_l_rise, amp_l_rise, label='Left FFT (Rise)', alpha=0.7)
        idx = np.argmax(amp_l_rise)
        if amp_l_rise[idx] > rise_max_amp:
            rise_max_amp = amp_l_rise[idx]
            rise_peak_freq = freq_l_rise[idx]

    if freq_r_rise is not None and amp_r_rise is not None:
        plt.plot(freq_r_rise, amp_r_rise, label='Right FFT (Rise)', alpha=0.7)
        idx = np.argmax(amp_r_rise)
        if amp_r_rise[idx] > rise_max_amp:
            rise_max_amp = amp_r_rise[idx]
            rise_peak_freq = freq_r_rise[idx]

    plt.title(f"{filename} - Rising Phase\nPeak: {rise_peak_freq:.2f} Hz")
    plt.xlabel("Freq (Hz)")
    plt.ylabel("Amp")
    plt.legend()
    plt.grid(True)
    plt.xlim(0, 10)
    plt.xticks(np.arange(0, 11, 1))

    # Subplot 2: Falling Phase Frequency Spectrum
    plt.subplot(3, 1, 2)
    if freq_l_fall is not None and amp_l_fall is not None:
        plt.plot(freq_l_fall, amp_l_fall, label='Left FFT (Fall)', alpha=0.7)
        idx = np.argmax(amp_l_fall)
        if amp_l_fall[idx] > fall_max_amp:
            fall_max_amp = amp_l_fall[idx]
            fall_peak_freq = freq_l_fall[idx]

    if freq_r_fall is not None and amp_r_fall is not None:
        plt.plot(freq_r_fall, amp_r_fall, label='Right FFT (Fall)', alpha=0.7)
        idx = np.argmax(amp_r_fall)
        if amp_r_fall[idx] > fall_max_amp:
            fall_max_amp = amp_r_fall[idx]
            fall_peak_freq = freq_r_fall[idx]

    plt.title(f"Falling Phase\nPeak: {fall_peak_freq:.2f} Hz")
    plt.xlabel("Freq (Hz)")
    plt.ylabel("Amp")
    plt.legend()
    plt.grid(True)
    plt.xlim(0, 10)
    plt.xticks(np.arange(0, 11, 1))

    # Subplot 3: Velocity vs Time
    plt.subplot(3, 1, 3)

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

        # Mark rise start and end points for left channel
        if len(points_left) > 0 and left_start_idx is not None:
            start_marker_t = t_full_left[left_start_idx] - t_full_left[0]
            start_marker_y = y_full_left[left_start_idx]
            plt.plot(start_marker_t, start_marker_y, 'go', markersize=8, label='Left Rise Start')

            if left_end_idx is not None:
                end_marker_t = t_full_left[left_end_idx] - t_full_left[0]
                end_marker_y = y_full_left[left_end_idx]
                plt.plot(end_marker_t, end_marker_y, 'ro', markersize=8, label='Left Rise End')

        # Find fall end point for marking
        if left_fall_idx is not None:
            time_limit = t_full_left[left_fall_idx] + 3.0
            idx_end_candidates = np.where(t_full_left <= time_limit)[0]
            if len(idx_end_candidates) > 0:
                left_fall_end_idx = idx_end_candidates[-1]
            else:
                left_fall_end_idx = len(t_full_left) - 1  # Use last point if less than 3s available

            # Mark fall start and end points for left channel
            fall_start_marker_t = t_full_left[left_fall_idx] - t_full_left[0]
            fall_start_marker_y = y_full_left[left_fall_idx]
            plt.plot(fall_start_marker_t, fall_start_marker_y, 'mo', markersize=8, label='Left Fall Start')

            fall_end_marker_t = t_full_left[left_fall_end_idx] - t_full_left[0]
            fall_end_marker_y = y_full_left[left_fall_end_idx]
            plt.plot(fall_end_marker_t, fall_end_marker_y, 'co', markersize=8, label='Left Fall End')

        # Mark rise start and end points for right channel
        if right_start_idx is not None:
            start_marker_t = t_full_right[right_start_idx] - start_t
            start_marker_y = y_full_right[right_start_idx]
            plt.plot(start_marker_t, start_marker_y, 'g^', markersize=8, label='Right Rise Start')

            if right_end_idx is not None:
                end_marker_t = t_full_right[right_end_idx] - start_t
                end_marker_y = y_full_right[right_end_idx]
                plt.plot(end_marker_t, end_marker_y, 'r^', markersize=8, label='Right Rise End')

        # Find fall end point for right channel
        right_fall_end_idx = None
        if right_fall_idx is not None:
            time_limit = t_full_right[right_fall_idx] + 3.0
            idx_end_candidates = np.where(t_full_right <= time_limit)[0]
            if len(idx_end_candidates) > 0:
                right_fall_end_idx = idx_end_candidates[-1]
            else:
                right_fall_end_idx = len(t_full_right) - 1  # Use last point if less than 3s available

        # Mark fall start point for right channel
        if right_fall_idx is not None:
            fall_start_marker_t = t_full_right[right_fall_idx] - start_t
            fall_start_marker_y = y_full_right[right_fall_idx]
            plt.plot(fall_start_marker_t, fall_start_marker_y, 'm^', markersize=8, label='Right Fall Start')

            if right_fall_end_idx is not None:
                fall_end_marker_t = t_full_right[right_fall_end_idx] - start_t
                fall_end_marker_y = y_full_right[right_fall_end_idx]
                plt.plot(fall_end_marker_t, fall_end_marker_y, 'c^', markersize=8, label='Right Fall End')

    plt.title("Velocity vs Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()

    plot_path = os.path.join(output_dir, f"{name_no_ext}_analysis.png")
    plt.savefig(plot_path)
    plt.close()

    return rise_peak_freq, rise_max_amp, fall_peak_freq, fall_max_amp


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
    rise_results = {}  # key: 'bent_v0.3', val: [freqs]
    fall_results = {}

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
            rise_freq, rise_amp, fall_freq, fall_amp = res
            # Filter out very low freq noise if needed
            if rise_freq > 0.1:
                if category not in rise_results:
                    rise_results[category] = []
                rise_results[category].append(rise_freq)

            if fall_freq > 0.1:
                if category not in fall_results:
                    fall_results[category] = []
                fall_results[category].append(fall_freq)

    print("\n" + "=" * 50)
    print("Summary of Dominant Frequencies by Category")
    print("=" * 50)

    # Print rising phase results
    print("\n--- RISING PHASE ---")
    for cat in sorted(rise_results.keys()):
        freqs = rise_results[cat]
        avg_f = np.mean(freqs)
        med_f = np.median(freqs)
        std_f = np.std(freqs)

        print(f"Category: {cat} (n={len(freqs)})")
        print(f"  Average: {avg_f:.4f} Hz")
        print(f"  Median:  {med_f:.4f} Hz")
        print(f"  StdDev:  {std_f:.4f} Hz")
        print(f"  Values:  {np.round(freqs, 3)}")
        print("-" * 30)

    # Print falling phase results
    print("\n--- FALLING PHASE ---")
    for cat in sorted(fall_results.keys()):
        freqs = fall_results[cat]
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
