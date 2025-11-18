import csv

input_file = 'part5point/robotPose.csv'
output_file = 'part5point/robotPose_fixed.csv'

with open(input_file, 'r') as f_in, open(output_file, 'w', newline='') as f_out:
    reader = csv.reader(f_in)
    writer = csv.writer(f_out)
    
    for row in reader:
        if len(row) == 9:  # Data rows (missing odom_th)
            row.insert(2, '0.0')
        writer.writerow(row)