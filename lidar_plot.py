import numpy as np
import matplotlib.pyplot as plt
import ast
import matplotlib.animation as animation
from time import sleep

def parse_laser_line(line):
    """Parse a line from your CSV"""
    line = line.strip().rstrip(';')
    parts = line.strip().split('; ')
    
    # Parse the array string - handle inf values
    ranges_str = parts[0].replace("array('f', ", "").rstrip(')')
    
    # Replace inf with np.inf for evaluation
    ranges_str = ranges_str.replace('inf', 'np.inf')
    
    # Evaluate the list
    ranges = eval(ranges_str)
    ranges = np.array(ranges)
    
    # Parse angle increment and timestamp
    angle_increment = float(parts[1])
    timestamp = int(parts[2])
    
    return ranges, angle_increment, timestamp

def plot_laser_scan(ranges, angle_increment, timestamp):
    """Plot a single laser scan"""
    # Create angle array
    angles = np.arange(len(ranges)) * angle_increment
    
    # Filter out inf values
    valid = np.isfinite(ranges)
    angles_valid = angles[valid]
    ranges_valid = ranges[valid]
    
    # Convert to Cartesian coordinates
    x = ranges_valid * np.cos(angles_valid)
    y = ranges_valid * np.sin(angles_valid)
    
    # Plot
    plt.figure(figsize=(10, 10))
    plt.scatter(x, y, s=5, c='blue', alpha=0.6)
    plt.scatter(0, 0, s=100, c='red', marker='x', label='Laser origin')
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.title(f'Laser Scan at timestamp {timestamp}')
    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    
    # Print stats
    print(f"Timestamp: {timestamp}")
    print(f"Total readings: {len(ranges)}")
    print(f"Valid readings: {np.sum(valid)}")
    print(f"Invalid (inf): {np.sum(~valid)}")
    print(f"Min range: {np.min(ranges_valid):.3f}m")
    print(f"Max range: {np.max(ranges_valid):.3f}m")
    print(f"Angular coverage: {np.degrees(len(ranges) * angle_increment):.1f}°")
    
    return plt

def process_laser_csv(filename):
    """Process entire CSV file"""
    scans = []
    
    with open(filename, 'r') as f:
        for i, line in enumerate(f):
            if i == 0:
                continue
            if line.strip():  # Skip empty lines
                print(line)
                ranges, angle_increment, timestamp = parse_laser_line(line)
                scans.append({
                    'ranges': ranges,
                    'angle_increment': angle_increment,
                    'timestamp': timestamp
                })
    
    return scans

def animate_scans_stationary(scans, interval=100):
    """
    Animate scans with stationary robot at origin
    interval: time between frames in milliseconds (100ms = 0.1 seconds)
    """
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # Find global bounds for consistent view
    all_x, all_y = [], []
    for scan in scans:
        ranges = scan['ranges']
        angle_increment = scan['angle_increment']
        angles = np.arange(len(ranges)) * angle_increment
        valid = np.isfinite(ranges)
        x = ranges[valid] * np.cos(angles[valid])
        y = ranges[valid] * np.sin(angles[valid])
        all_x.extend(x)
        all_y.extend(y)
    
    if all_x and all_y:
        xlim = [min(all_x) - 0.5, max(all_x) + 0.5]
        ylim = [min(all_y) - 0.5, max(all_y) + 0.5]
    else:
        xlim, ylim = [-4, 4], [-4, 4]
    
    # Initialize plot elements
    scan_points = ax.scatter([], [], s=5, c='blue', alpha=0.6, label='Laser scan')
    robot_pos = ax.scatter([0], [0], s=200, c='red', marker='o', 
                          edgecolors='black', linewidths=2, label='Robot', zorder=5)
    
    ax.set_xlabel('X (meters)', fontsize=12)
    ax.set_ylabel('Y (meters)', fontsize=12)
    ax.axis('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    ax.legend(loc='upper right')
    ax.set_title('Laser Scan Visualization', fontsize=14)
    
    # Text for info
    info_text = ax.text(0.02, 0.98, '', transform=ax.transAxes, 
                       verticalalignment='top', fontsize=10,
                       bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    def update(frame):
        scan = scans[frame]
        ranges = scan['ranges']
        angle_increment = scan['angle_increment']
        timestamp = scan['timestamp']
        
        # Create angles and filter valid
        angles = np.arange(len(ranges)) * angle_increment
        valid = np.isfinite(ranges)
        
        # Convert to Cartesian (robot at origin)
        x = ranges[valid] * np.cos(angles[valid])
        y = ranges[valid] * np.sin(angles[valid])
        
        # Update scan points
        scan_points.set_offsets(np.c_[x, y])
        
        # Calculate statistics
        valid_count = np.sum(valid)
        if valid_count > 0:
            min_range = np.min(ranges[valid])
            max_range = np.max(ranges[valid])
            avg_range = np.mean(ranges[valid])
            stats_text = f'Min: {min_range:.2f}m\nMax: {max_range:.2f}m\nAvg: {avg_range:.2f}m'
        else:
            stats_text = 'No valid readings'
        
        # Update info text
        info_text.set_text(f'Scan: {frame+1}/{len(scans)}\n'
                          f'Timestamp: {timestamp}\n'
                          f'Valid points: {valid_count}/{len(ranges)}\n'
                          f'{stats_text}')
        
        return scan_points, robot_pos, info_text
    
    anim = animation.FuncAnimation(fig, update, frames=len(scans), 
                                   interval=interval, repeat=True, blit=True)
    plt.tight_layout()
    plt.show()
    return anim


# Load all scans
scans = process_laser_csv('laser_content_line.csv')
print(f"Loaded {len(scans)} scans")
# Run with 0.1 second intervals (100ms)
anim = animate_scans_stationary(scans, interval=100)