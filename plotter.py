import csv
import matplotlib.pyplot as plt

# Read the data
stamps = []
odom_x = []
odom_y = []
pf_x = []
pf_y = []

file = 'part5/robotPose_fixed.csv'

with open(file, 'r') as f:
    reader = csv.reader(f)
    next(reader)  # Skip header
    
    for row in reader:
        # print(row)
        if len(row) == 10:
            odom_x.append(float(row[0]))
            pf_x.append(float(row[5]))
            odom_y.append(float(row[1]))
            pf_y.append(float(row[6]))

# Plot
plt.plot(odom_x, odom_y, 'bo-', label='Odometry')
plt.plot(pf_x, pf_y, 'rs-', label='Particle Filter')
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.title("Part 5")
plt.legend()
plt.grid(True)
plt.show()