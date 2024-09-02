clear;
clc;

% Specify the file name
filename = '/home/eslam/catkin_workspace/src/t2/scripts/Real_Time_Demo/vrep2_steering.csv';

% Read the data from the CSV file
data = readtable(filename);

% Access data by column name
columnData_X = data.positions_x_odom; 
columnData_X_clean = columnData_X(~isnan(columnData_X));

columnData_Y = data.positions_y_odom; 
columnData_Y_clean = columnData_Y(~isnan(columnData_Y));

columnData_yaw = data.steering_angle;
columnData_yaw_clean = columnData_yaw(~isnan(columnData_yaw));

columnData_time = data.Time_Sec;
columnData_time_clean = columnData_time(~isnan(columnData_time));

% Apply Noise to x y yaw and store it in a New CSV called NoisyOdom.csv for line path------------------

A = [ 1 0 0; 
      0 1 0;
      0 0 1];   

B = [0 1 1;
     0 1 1;
     1 0 0;]; 

H = [1;
     0;
     0];          % Observation matrix

Q = 0.1*eye(3);     % Process noise covariance to be applied to the whole matrix [2 x 2]

R = 50*eye(3);           % Measurement noise covariance old 0.1

x = [0; 
     0;
     0];         % Initial state estimate for x, y directions and yaw.

P = eye(3);         % Initial error covariance to be large number
   % Initial error covariance to be large number


columnData_YawN = columnData_yaw_clean;
filtered_yawPoints = zeros(size(columnData_YawN));
for i = 1:length(columnData_YawN)
    % Predictionstep
    x = A.*x;                        % Predicted state estimate
    P = A.*P.*A' + Q;              % Predicted error covariance
    % P = A.*P. *A' + Q;               % Predicted error covariance
    
    % Update step
    K = P.*H'/((H.*P).*H' + R);          % Kalman gain
    % K = eye(2).*H'/(H.*eye(2).*H' + R);  % Kalman gain (using identity matrix for P)

    x = x + K.*(columnData_YawN(i) - H.*x); % Updated state estimate
    P = P - K.*(H.*P);           % Updated error covariance  (use TIMES (.) for elementwise multiplication.)
    
    filtered_yawPoints(i) = x(1);    % Store filtered position
end

low = lowpass(filtered_yawPoints,0.01);

%noise plot
plot(columnData_time_clean,columnData_yaw_clean, '-r');
xlabel('time');
ylabel('steering_angle');
grid on;
hold on;

%filter plot
plot(columnData_time_clean,filtered_yawPoints, '-g');
xlabel('time');
ylabel('steering_angle');
grid on;
hold on;

%filter plot
plot(columnData_time_clean,low, '-b');
xlabel('time');
ylabel('steering_angle');
grid on;
hold on;

legend ('noisy signal','kalman signal', 'signal after kalman and low pass filter')