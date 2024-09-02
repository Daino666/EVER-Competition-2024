clear;
clc;

% Specify the file name
filename = '/home/eslam/catkin_workspace/src/t2/scripts/milestone3/Graphs3/change_lane_vel_acc_RMS.csv';

% Read the data from the CSV file
data = readtable(filename);

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

xPointsM= awgn(columnData_X_clean, 30.78, "measured");     % old = 0.0313
yPointsM= awgn(columnData_Y_clean, 25.9, "measured");       % old = 46.3
yawPointsM= awgn(columnData_yaw_clean,0.075,"measured");


% Kalman filter parameters
A = [ 1 0 0; 
      0 1 0;
      0 0 1];    % State transition matrix

B = [0 1 1;
     0 1 1;
     1 0 0;];    % control input matrix

U = [1;
     1;
     1];         % control input vector

H = [1;
     0;
     0];          % Observation matrix

Q = 0.629*eye(3);     % Process noise covariance to be applied to the whole matrix

R = 10*eye(3);           % Measurement noise covariance old 0.1

x = [0; 
     0;
     0];         % Initial state

P = eye(3);         % Initial error covariance to be large number



% using fuction with single variable to try each parameter in the filter
function filteredData = applyKalmanFilter(noisyData, A, H, Q, R, x, P)
    % Initialize the filtered data array
    filteredData = zeros(size(noisyData));
    
    for i = 1:length(noisyData)
        % Prediction step
        x = A .* x;                        % Predicted state estimate
        P = A .* P .* A' + Q;               % Predicted error covariance
        
        % Update step
        K = P .* H' / ((H .* P) .* H' + R);    % Kalman gain
        x = x + K .* (noisyData(i) - H .* x); % Updated state estimate
        P = P - K .* H .* P;                % Updated error covariance
        
        filteredData(i) = x(1);   
    end
end



% Apply Kalman filter
filtered_xPoints = applyKalmanFilter(xPointsM, A, H, Q, R, x, P);
filtered_yPoints = applyKalmanFilter(yPointsM, A, H, Q, R, x, P);
filtered_yawPoints = applyKalmanFilter(yawPointsM, A, H, Q, R, x, P);

% Plot noisy and filtered data
plot(xPointsM, yPointsM, '-r');
xlabel('noised x');
ylabel('noised y');
grid on;
hold on;

plot(filtered_xPoints, filtered_yPoints, '-g');
xlabel('filtered x');
ylabel('filtered y');
grid on;
hold on;

legend('noisy signal', 'filtered signal')

overall_RMS_x = mean(data_rms_x_clean)
overall_RMS_y = mean(data_rms_y_clean)
overall_RMS_yaw = mean(data_rms_yaw_clean)


new_data = table(columnData_X_clean, columnData_Y_clean, columnData_yaw_clean, xPointsM, yPointsM, yawPointsM, filtered_xPoints, filtered_yPoints, filtered_yawPoints, ...
                 'VariableNames', {'x_odom', 'y_odom', 'yaw_odom', 'x_noisy', 'y_noisy', 'yaw_noisy', 'x_filtered', 'y_filtered', 'yaw_filtered'});

new_file = '/home/eslam/catkin_workspace/src/t2/scripts/trial1.csv';

writetable(new_data, new_file);
