import csv
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt

# Specify the file name
filename = '/home/eslam/catkin_workspace/src/t2/scripts/Real_Time_Demo/vrep2_steering.csv'

# Read the data from the CSV file
columnData_X = []
columnData_Y = []
columnData_yaw = []
columnData_time = []

with open(filename, newline='') as csvfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
        if row['positions_x_odom'] and row['positions_y_odom'] and row['steering_angle'] and row['Time_Sec']:
            columnData_X.append(float(row['positions_x_odom']))
            columnData_Y.append(float(row['positions_y_odom']))
            columnData_yaw.append(float(row['steering_angle']))
            columnData_time.append(float(row['Time_Sec']))

# Convert lists to numpy arrays and remove NaNs
columnData_X = np.array(columnData_X)
columnData_Y = np.array(columnData_Y)
columnData_yaw = np.array(columnData_yaw)
columnData_time = np.array(columnData_time)

columnData_X_clean = columnData_X[~np.isnan(columnData_X)]
columnData_Y_clean = columnData_Y[~np.isnan(columnData_Y)]
columnData_yaw_clean = columnData_yaw[~np.isnan(columnData_yaw)]
columnData_time_clean = columnData_time[~np.isnan(columnData_time)]

# Apply Noise to x y yaw and store it in a New CSV called NoisyOdom.csv for line path------------------

A = np.eye(3)  # State transition matrix
H = np.array([[1, 0, 0]])  # Observation matrix (as a row vector)

Q = 0.629 * np.eye(3)   # Process noise covariance
R = 80 * np.eye(1)      # Measurement noise covariance

x = np.zeros(3)         # Initial state estimate for x, y directions and yaw.
P = np.eye(3)           # Initial error covariance

filtered_yawPoints = np.zeros_like(columnData_yaw)
for i in range(len(columnData_yaw)):
    # Prediction step
    x = A @ x  # Predicted state estimate
    P = A @ P @ A.T + Q  # Predicted error covariance
    
    # Update step
    y = np.array([columnData_yaw[i]])  # Measurement as a 1D array
    K = P @ H.T @ np.linalg.inv(H @ P @ H.T + R)  # Kalman gain
    x = x + K @ (y - H @ x)  # Updated state estimate
    P = P - K @ H @ P  # Updated error covariance
    
    filtered_yawPoints[i] = x[0]  # Store filtered position

# Low-pass filter
def lowpass(data, cutoff, fs=1.0, order=5):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data)
    return y

low = lowpass(filtered_yawPoints, 0.01)

# Plotting
plt.figure(figsize=(10, 6))

plt.plot(columnData_time, columnData_yaw, '-r', label='noisy signal')
plt.plot(columnData_time, filtered_yawPoints, '-g', label='kalman signal')
plt.plot(columnData_time, low, '-b', label='signal after kalman and low pass filter')

plt.xlabel('time')
plt.ylabel('steering_angle')
plt.grid(True)
plt.legend()

plt.show()
