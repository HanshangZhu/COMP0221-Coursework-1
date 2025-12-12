#!/usr/bin/env python3
"""
COMP0221 CW1 - Log Parser and Analysis Tool
Parses CSV output from ESP32 serial and generates analysis plots.

Usage:
    1. Capture serial output to file: idf.py monitor > log.txt
    2. Run: python analyze_stats.py log.txt
"""

import sys
import re
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
from datetime import datetime

# Configure matplotlib for larger fonts (2.5x bigger)
plt.rcParams.update({
    'font.size': 25,           # Base font size (default ~10)
    'axes.titlesize': 30,      # Title font size
    'axes.labelsize': 25,      # Axis label font size
    'xtick.labelsize': 20,     # X tick label size
    'ytick.labelsize': 20,     # Y tick label size
    'legend.fontsize': 20,     # Legend font size
    'figure.titlesize': 32,    # Figure title size
})

# Data containers
global_stats = []       # List of (rx_count, rx_valid, rx_invalid, avg_lat, min_lat, max_lat, jitter, loss, timeouts)
neighbor_stats = defaultdict(list)  # MAC -> list of (pkt_count, avg_lat, jitter, x, y, z)
flock_stats = []        # List of (nb_count, centroid_dist, min_sep, x, y, z)
latencies = []          # Raw latency values extracted from logs
security_stats = []     # List of (elapsed_s, replay_blocked, replay_passed, cmac_fail, block_rate, attack_sent)

def parse_log(filename):
    """Parse log file and extract CSV data."""
    print(f"Parsing {filename}...")
    
    with open(filename, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            # CSV_GLOBAL,elapsed_s,rx,valid,invalid,avg_lat,min_lat,max_lat,jitter,loss,timeouts
            if line.startswith('CSV_GLOBAL,'):
                parts = line.strip().split(',')
                if len(parts) >= 11:
                    try:
                        global_stats.append({
                            'elapsed_s': int(parts[1]),
                            'rx_count': int(parts[2]),
                            'rx_valid': int(parts[3]),
                            'rx_invalid': int(parts[4]),
                            'avg_lat': int(parts[5]),
                            'min_lat': int(parts[6]),
                            'max_lat': int(parts[7]),
                            'jitter': float(parts[8]),
                            'loss': float(parts[9]),
                            'timeouts': int(parts[10])
                        })
                    except ValueError:
                        pass
            
            # CSV_NEIGHBOR,elapsed_s,MAC,pkt_count,avg_lat,jitter,x,y,z
            elif line.startswith('CSV_NEIGHBOR,'):
                parts = line.strip().split(',')
                if len(parts) >= 9:
                    try:
                        elapsed = int(parts[1])
                        mac = parts[2]
                        neighbor_stats[mac].append({
                            'elapsed_s': elapsed,
                            'pkt_count': int(parts[3]),
                            'avg_lat': int(parts[4]),
                            'jitter': float(parts[5]),
                            'x': float(parts[6]),
                            'y': float(parts[7]),
                            'z': float(parts[8])
                        })
                    except ValueError:
                        pass
            
            # CSV_FLOCK,elapsed_s,nb_count,centroid_dist,min_sep,x,y,z
            elif line.startswith('CSV_FLOCK,'):
                parts = line.strip().split(',')
                if len(parts) >= 8:
                    try:
                        flock_stats.append({
                            'elapsed_s': int(parts[1]),
                            'nb_count': int(parts[2]),
                            'centroid_dist': float(parts[3]),
                            'min_sep': float(parts[4]),
                            'x': float(parts[5]),
                            'y': float(parts[6]),
                            'z': float(parts[7])
                        })
                    except ValueError:
                        pass
            
            # CSV_LATENCY,elapsed_s,latency_ms - individual latency for histogram
            elif line.startswith('CSV_LATENCY,'):
                parts = line.strip().split(',')
                if len(parts) >= 3:
                    try:
                        latencies.append({
                            'elapsed_s': int(parts[1]),
                            'latency': int(parts[2])
                        })
                    except ValueError:
                        pass
            
            # CSV_SECURITY,elapsed_s,blocked,passed,cmac_fail,block_rate,attack_sent
            elif line.startswith('CSV_SECURITY,'):
                parts = line.strip().split(',')
                if len(parts) >= 7:
                    try:
                        security_stats.append({
                            'elapsed_s': int(parts[1]),
                            'replay_blocked': int(parts[2]),
                            'replay_passed': int(parts[3]),
                            'cmac_fail': int(parts[4]),
                            'block_rate': float(parts[5]),
                            'attack_sent': int(parts[6])
                        })
                    except ValueError:
                        pass
            
            # Fallback: Extract latency from LORA_RX logs
            # Example: "LORA_RX: Valid packet -> Seq:123 Z:50000mm Lat:45ms"
            else:
                lat_match = re.search(r'Lat:(\d+)ms', line)
                if lat_match:
                    latencies.append({
                        'elapsed_s': 0,
                        'latency': int(lat_match.group(1))
                    })
    
    print(f"Found: {len(global_stats)} global stats, "
          f"{len(neighbor_stats)} neighbors, "
          f"{len(flock_stats)} flock samples, "
          f"{len(latencies)} latency values")

def plot_latency_histogram(suffix=""):
    """Plot latency distribution histogram."""
    if not latencies:
        print("No latency data to plot")
        return
    
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Extract latency values from dict (use abs() to handle NTP clock offset)
    lat_values = [abs(l['latency']) for l in latencies if abs(l['latency']) < 1000]
    
    if not lat_values:
        print("No valid latency data after filtering")
        return
    
    ax.hist(lat_values, bins=50, edgecolor='black', alpha=0.7)
    ax.set_xlabel('Latency (ms)')
    ax.set_ylabel('Frequency')
    ax.set_title(f'LoRa Packet Latency Distribution (n={len(lat_values)})')
    ax.axvline(np.mean(lat_values), color='r', linestyle='--', 
               label=f'Mean: {np.mean(lat_values):.1f}ms')
    ax.axvline(np.median(lat_values), color='g', linestyle='--', 
               label=f'Median: {np.median(lat_values):.1f}ms')
    ax.legend()
    
    plt.tight_layout()
    plt.savefig(f'latency_histogram{suffix}.png', dpi=150)
    print(f"Saved: latency_histogram{suffix}.png")

def plot_latency_over_time(suffix=""):
    """Plot latency statistics over time."""
    if not global_stats:
        print("No global stats to plot")
        return
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    
    # Use real elapsed time (convert to minutes)
    times = [s['elapsed_s'] / 60.0 for s in global_stats]
    avg_lats = [s['avg_lat'] for s in global_stats]
    min_lats = [s['min_lat'] for s in global_stats]
    max_lats = [s['max_lat'] for s in global_stats]
    jitters = [s['jitter'] for s in global_stats]
    
    # Latency plot
    ax1.fill_between(times, min_lats, max_lats, alpha=0.3, label='Min-Max Range')
    ax1.plot(times, avg_lats, 'b-', linewidth=2, label='Average')
    ax1.set_ylabel('Latency (ms)')
    ax1.set_title('Latency Over Time')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Jitter plot
    ax2.plot(times, jitters, 'r-', linewidth=2)
    ax2.set_xlabel('Time (minutes)')
    ax2.set_ylabel('Jitter (ms)')
    ax2.set_title('Jitter (Std Dev of Latency) Over Time')
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(f'latency_over_time{suffix}.png', dpi=150)
    print(f"Saved: latency_over_time{suffix}.png")

def plot_neighbor_availability(suffix=""):
    """Plot neighbor availability and packet loss."""
    if not global_stats:
        print("No global stats to plot")
        return
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    
    minutes = range(len(global_stats))
    rx_valid = [s['rx_valid'] for s in global_stats]
    rx_invalid = [s['rx_invalid'] for s in global_stats]
    loss_rate = [s['loss'] for s in global_stats]
    timeouts = [s['timeouts'] for s in global_stats]
    
    # Packet counts
    ax1.bar(minutes, rx_valid, label='Valid', alpha=0.7)
    ax1.bar(minutes, rx_invalid, bottom=rx_valid, label='Invalid', alpha=0.7, color='red')
    ax1.set_ylabel('Packet Count (cumulative)')
    ax1.set_title('Packet Reception Statistics')
    ax1.legend()
    
    # Loss rate
    ax2.plot(minutes, loss_rate, 'r-', linewidth=2, label='Loss Rate')
    ax2.set_xlabel('Time (minutes)')
    ax2.set_ylabel('Loss Rate (%)')
    ax2.set_title('Packet Loss Rate Over Time')
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(f'neighbor_availability{suffix}.png', dpi=150)
    print(f"Saved: neighbor_availability{suffix}.png")

def plot_flock_stability(suffix=""):
    """Plot flocking stability metrics."""
    if not flock_stats:
        print("No flock stats to plot")
        return
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    samples = range(len(flock_stats))
    nb_counts = [s['nb_count'] for s in flock_stats]
    centroid_dists = [s['centroid_dist'] for s in flock_stats]
    min_seps = [s['min_sep'] for s in flock_stats]
    
    # Neighbor count
    axes[0, 0].plot(samples, nb_counts, 'b-', linewidth=2)
    axes[0, 0].set_ylabel('Active Neighbors')
    axes[0, 0].set_title('Number of Active Neighbors')
    axes[0, 0].grid(True, alpha=0.3)
    
    # Centroid distance
    axes[0, 1].plot(samples, centroid_dists, 'g-', linewidth=2)
    axes[0, 1].set_ylabel('Distance (m)')
    axes[0, 1].set_title('Distance to Flock Centroid')
    axes[0, 1].grid(True, alpha=0.3)
    
    # Min separation
    axes[1, 0].plot(samples, min_seps, 'r-', linewidth=2)
    axes[1, 0].axhline(5.0, color='k', linestyle='--', label='Separation Threshold (5m)')
    axes[1, 0].set_xlabel('Time (minutes)')
    axes[1, 0].set_ylabel('Distance (m)')
    axes[1, 0].set_title('Minimum Separation to Nearest Neighbor')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    
    # 3D trajectory
    ax3d = fig.add_subplot(2, 2, 4, projection='3d')
    xs = [s['x'] for s in flock_stats]
    ys = [s['y'] for s in flock_stats]
    zs = [s['z'] for s in flock_stats]
    ax3d.plot(xs, ys, zs, 'b-', linewidth=1, alpha=0.7)
    ax3d.scatter(xs[0], ys[0], zs[0], color='g', s=100, label='Start')
    ax3d.scatter(xs[-1], ys[-1], zs[-1], color='r', s=100, label='End')
    ax3d.set_xlabel('X (m)')
    ax3d.set_ylabel('Y (m)')
    ax3d.set_zlabel('Z (m)')
    ax3d.set_title('Drone Trajectory')
    ax3d.legend()
    
    plt.tight_layout()
    plt.savefig(f'flock_stability{suffix}.png', dpi=150)
    print(f"Saved: flock_stability{suffix}.png")

def plot_neighbor_count(suffix=""):
    """Plot neighbor count over time - useful for SF7 vs SF9 comparison."""
    if not flock_stats:
        print("No flock stats to plot neighbor count")
        return
    
    fig, ax = plt.subplots(figsize=(12, 5))
    
    times = [s['elapsed_s'] / 60.0 for s in flock_stats]
    nb_counts = [s['nb_count'] for s in flock_stats]
    
    ax.plot(times, nb_counts, 'b-', linewidth=2, marker='o', markersize=4)
    ax.axhline(2, color='g', linestyle='--', alpha=0.7, label='Expected (2 neighbours)')
    ax.set_xlabel('Time (minutes)')
    ax.set_ylabel('Active Neighbour Count')
    ax.set_title(f'Neighbour Visibility Over Time')
    ax.set_ylim(-0.5, 3.5)
    ax.set_yticks([0, 1, 2, 3])
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Calculate statistics
    avg_nb = np.mean(nb_counts)
    time_with_2 = sum(1 for n in nb_counts if n == 2) / len(nb_counts) * 100
    print(f"  Avg neighbours: {avg_nb:.2f}, Time with 2 neighbours: {time_with_2:.1f}%")
    
    plt.tight_layout()
    plt.savefig(f'neighbor_count{suffix}.png', dpi=150)
    print(f"Saved: neighbor_count{suffix}.png")

def plot_per_neighbor_comparison(suffix=""):
    """Plot per-neighbor latency comparison."""
    if not neighbor_stats:
        print("No neighbor stats to plot")
        return
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    
    macs = list(neighbor_stats.keys())
    avg_lats = []
    avg_jitters = []
    
    for mac in macs:
        data = neighbor_stats[mac]
        if data:
            avg_lats.append(np.mean([d['avg_lat'] for d in data]))
            avg_jitters.append(np.mean([d['jitter'] for d in data]))
    
    # Short MAC labels
    labels = [m[-4:] for m in macs]
    
    # Latency comparison
    bars1 = ax1.bar(labels, avg_lats, color='steelblue', edgecolor='black')
    ax1.set_xlabel('Neighbor (last 4 hex)')
    ax1.set_ylabel('Average Latency (ms)')
    ax1.set_title('Per-Neighbor Average Latency')
    ax1.tick_params(axis='x', rotation=45)
    
    # Jitter comparison
    bars2 = ax2.bar(labels, avg_jitters, color='coral', edgecolor='black')
    ax2.set_xlabel('Neighbor (last 4 hex)')
    ax2.set_ylabel('Average Jitter (ms)')
    ax2.set_title('Per-Neighbor Average Jitter')
    ax2.tick_params(axis='x', rotation=45)
    
    plt.tight_layout()
    plt.savefig(f'per_neighbor_stats{suffix}.png', dpi=150)
    print(f"Saved: per_neighbor_stats{suffix}.png")

def print_summary():
    """Print statistical summary."""
    print("\n" + "="*60)
    print("STATISTICAL SUMMARY")
    print("="*60)
    
    if latencies:
        # Use abs() to handle NTP clock offset (negative latencies)
        lat_arr = np.array([abs(l['latency']) for l in latencies if abs(l['latency']) < 1000])
        if len(lat_arr) > 0:
            print(f"\nLatency Statistics (n={len(lat_arr)}):")
            print(f"  Mean:   {np.mean(lat_arr):.2f} ms")
            print(f"  Median: {np.median(lat_arr):.2f} ms")
            print(f"  Std:    {np.std(lat_arr):.2f} ms")
            print(f"  Min:    {np.min(lat_arr)} ms")
            print(f"  Max:    {np.max(lat_arr)} ms")
            print(f"  P95:    {np.percentile(lat_arr, 95):.2f} ms")
            print(f"  P99:    {np.percentile(lat_arr, 99):.2f} ms")
    
    if global_stats:
        total_valid = global_stats[-1]['rx_valid']
        total_invalid = global_stats[-1]['rx_invalid']
        total = total_valid + total_invalid
        print(f"\nPacket Statistics:")
        print(f"  Total RX:  {total}")
        print(f"  Valid:     {total_valid} ({100*total_valid/total:.1f}%)")
        print(f"  Invalid:   {total_invalid} ({100*total_invalid/total:.1f}%)")
        print(f"  Timeouts:  {global_stats[-1]['timeouts']}")
    
    if flock_stats:
        cd = [s['centroid_dist'] for s in flock_stats]
        ms = [s['min_sep'] for s in flock_stats]
        print(f"\nFlocking Statistics:")
        print(f"  Avg Centroid Distance: {np.mean(cd):.2f} m")
        print(f"  Avg Min Separation:    {np.mean(ms):.2f} m")
        print(f"  Separation Violations: {sum(1 for s in ms if s < 5)}")
    
    print("="*60)

def plot_security_stats():
    """Output security/attack defense statistics as table data."""
    if not security_stats:
        print("No security data found")
        return
    
    # Get final values (cumulative)
    final = security_stats[-1]
    blocked = final['replay_blocked']
    passed = final['replay_passed']
    cmac_fail = final['cmac_fail']
    attack_sent = final['attack_sent']
    
    total_checked = blocked + passed
    block_rate = (100.0 * blocked / total_checked) if total_checked > 0 else 0
    
    print("\n" + "="*60)
    print("SECURITY STATISTICS")
    print("="*60)
    print(f"  Replay Blocked:    {blocked}")
    print(f"  Replay Passed:     {passed}")
    print(f"  Block Rate:        {block_rate:.1f}%")
    print(f"  CMAC Failures:     {cmac_fail}")
    print(f"  Attack Packets TX: {attack_sent}")
    print("="*60)
    
    # Output LaTeX table format
    print("\n% LaTeX table for report:")
    print("\\begin{table}[h]")
    print("\\centering")
    print("\\caption{Security Evaluation Results}")
    print("\\label{tab:security}")
    print("\\begin{tabular}{@{}lc@{}}")
    print("\\toprule")
    print("\\textbf{Metric} & \\textbf{Value} \\\\")
    print("\\midrule")
    print(f"Replay Blocked & {blocked} \\\\")
    print(f"Replay Passed & {passed} \\\\")
    print(f"Block Rate & {block_rate:.1f}\\% \\\\")
    print(f"CMAC Failures & {cmac_fail} \\\\")
    print(f"Attack Packets TX & {attack_sent} \\\\")
    print("\\bottomrule")
    print("\\end{tabular}")
    print("\\end{table}")

def main():
    if len(sys.argv) < 2:
        print("Usage: python analyze_stats.py <logfile> [suffix]")
        print("Example: python analyze_stats.py serial_log.txt sf9")
        sys.exit(1)
    
    # Optional suffix for output files
    suffix = ""
    if len(sys.argv) >= 3:
        suffix = "_" + sys.argv[2]
    
    parse_log(sys.argv[1])
    
    if not any([global_stats, neighbor_stats, flock_stats, latencies, security_stats]):
        print("No data found in log file!")
        sys.exit(1)
    
    # Generate all plots with optional suffix
    plot_latency_histogram(suffix)
    plot_latency_over_time(suffix)
    plot_neighbor_availability(suffix)
    plot_flock_stability(suffix)
    plot_neighbor_count(suffix)
    plot_per_neighbor_comparison(suffix)
    plot_security_stats()
    
    # Print summary
    print_summary()
    
    print(f"\nDone! Check the generated PNG files{suffix}.")

if __name__ == '__main__':
    main()
