import numpy as np
import matplotlib.pyplot as plt

# Parameters
dt = 0.01
total_time = 3.0
time = np.arange(0, total_time, dt)

# Initialize arrays
theta = np.zeros(len(time))
u1 = np.zeros(len(time))
u2 = np.zeros(len(time))
u3 = np.zeros(len(time))

# Commands for each second
for i, t in enumerate(time):
    if t < 1.0:
        # Move along y-axis at 1 m/s
        x_dot, y_dot, theta_dot = 0, 1, 0
    elif t < 2.0:
        # Rotate CCW 90 degrees in 1 second
        x_dot, y_dot, theta_dot = 0, 0, np.pi/2
        theta[i] = theta[i-1] + theta_dot * dt
    else:
        # Move along x-axis at 1 m/s
        x_dot, y_dot, theta_dot = 1, 0, 0
        theta[i] = theta[i-1]
    
    # Motor equation matrix
    theta_val = theta[i]
    M = np.sqrt(2) * np.array([
        [np.cos(theta_val + np.pi/2), np.sin(theta_val + np.pi/2), np.sqrt(2)],
        [np.cos(theta_val + 7*np.pi/6), np.sin(theta_val + 7*np.pi/6), np.sqrt(2)],
        [np.cos(theta_val + 11*np.pi/6), np.sin(theta_val + 11*np.pi/6), np.sqrt(2)]
    ])
    
    # Calculate wheel speeds
    velocities = np.array([x_dot, y_dot, theta_dot])
    u = M @ velocities
    u1[i], u2[i], u3[i] = u

# Plot
fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True, sharey=True)

axes[0].plot(time, u1, linewidth=2, color='C0')
axes[0].set_ylabel('u1 (deg/s)')
axes[0].grid(True, alpha=0.3)
axes[0].axvline(x=1.0, color='gray', linestyle='--', alpha=0.5)
axes[0].axvline(x=2.0, color='gray', linestyle='--', alpha=0.5)

axes[1].plot(time, u2, linewidth=2, color='C1')
axes[1].set_ylabel('u2 (deg/s)')
axes[1].grid(True, alpha=0.3)
axes[1].axvline(x=1.0, color='gray', linestyle='--', alpha=0.5)
axes[1].axvline(x=2.0, color='gray', linestyle='--', alpha=0.5)

axes[2].plot(time, u3, linewidth=2, color='C2')
axes[2].set_ylabel('u3 (deg/s)')
axes[2].set_xlabel('Time (s)')
axes[2].grid(True, alpha=0.3)
axes[2].axvline(x=1.0, color='gray', linestyle='--', alpha=0.5)
axes[2].axvline(x=2.0, color='gray', linestyle='--', alpha=0.5)

plt.suptitle('Wheel Speeds Over Time')
plt.tight_layout()
plt.show()