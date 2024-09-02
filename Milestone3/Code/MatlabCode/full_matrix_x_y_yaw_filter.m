clear;
clc;

% Specify the file name
filename = '/home/eslam/catkin_workspace/src/t2/scripts/milestone3/Graphs3/change_lane_vel_acc_RMS.csv';

% Read the data from the CSV file
data = readtable(filename);

% Extract and clean data
columnData_X = data.positions_x_odom; 
columnData_X_clean = columnData_X(~isnan(columnData_X));

columnData_Y = data.positions_y_odom; 
columnData_Y_clean = columnData_Y(~isnan(columnData_Y));

columnData_yaw = data.yaw_odom;
columnData_yaw_clean = columnData_yaw(~isnan(columnData_yaw));

columnData_time = data.Time_Sec;
columnData_time_clean = columnData_time(~isnan(columnData_time));

data_rms_x = data.rms_x_odom;
data_rms_x_clean = data_rms_x(~isnan(data_rms_x));

data_rms_y = data.rms_y_odom;
data_rms_y_clean = data_rms_y(~isnan(data_rms_y));

data_rms_yaw = data.rms_yaw_odom;
data_rms_yaw_clean = data_rms_yaw(~isnan(data_rms_yaw));

% Add noise to the data
xPointsM = awgn(columnData_X_clean, 30.78, "measured");
yPointsM = awgn(columnData_Y_clean, 25.9, "measured");
yawPointsM = awgn(columnData_yaw_clean, 0.075, "measured");

% Combine x, y, and yaw data into a single matrix
noisyData = [xPointsM'; yPointsM'; yawPointsM'];

% Kalman filter parameters
A =  [1 0 0; 
      0 1 0;
      0 0 1];    % State transition matrix

B = [0 1 1;
     0 1 1;
     1 0 0];    % Control input matrix

U = [1; 1; 1]; % Control input vector
H = eye(3);  % Observation matrix
Q = 0.629 * eye(3); % Process noise covariance
R = 10 * eye(3);    % Measurement noise covariance
x = zeros(3, 1);    % Initial state estimate for x, y, and yaw
P = eye(3);    % Initial error covariance

% Apply Kalman filter
filteredData = applyKalmanFilter(noisyData, A, H, Q, R, x, P);

% Extract the filtered x, y, and yaw data
filtered_xPoints = filteredData(1, :);
filtered_yPoints = filteredData(2, :);
filtered_yawPoints = filteredData(3, :);

% Plot noisy and filtered data
figure;
plot(xPointsM, yPointsM, '-r');
xlabel('noised x');
ylabel('noised y');
grid on;
hold on;

plot(filtered_xPoints, filtered_yPoints, '-g');
xlabel('x');
ylabel('y');
grid on;
hold on;

legend('noisy signal', 'filtered signal');

% Save filtered data to a new CSV file
new_data = table(columnData_X_clean, columnData_Y_clean, columnData_yaw_clean, ...
                 xPointsM, yPointsM, yawPointsM, ...
                 filtered_xPoints', filtered_yPoints', filtered_yawPoints', ...
                 'VariableNames', {'x_odom', 'y_odom', 'yaw_odom', ...
                                   'x_noisy', 'y_noisy', 'yaw_noisy', ...
                                   'x_filtered', 'y_filtered', 'yaw_filtered'});

new_file = '/home/eslam/catkin_workspace/src/t2/scripts/trial2.csv';
writetable(new_data, new_file);

overall_RMS_x = mean(data_rms_x_clean)
overall_RMS_y = mean(data_rms_y_clean)
overall_RMS_yaw = mean(data_rms_yaw_clean)

% Kalman filter function for multi-dimensional data
function filteredData = applyKalmanFilter(noisyData, A, H, Q, R, x, P)
    filteredData = zeros(size(noisyData));
    
    for i = 1:size(noisyData, 2)
        % Prediction step with control input
        x = A * x;
        P = A * P * A' + Q;
        
        % Measurement update
        K = P * H' / (H * P * H' + R); % Kalman gain
        x = x + K * (noisyData(:, i) - H * x); % Update estimate
        P = (eye(size(K,1)) - K * H) * P; % Update error covariance
        
        filteredData(:, i) = x;
    end
end
