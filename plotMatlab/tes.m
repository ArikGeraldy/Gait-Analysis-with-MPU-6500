clear;
close all;
clc;
addpath('Quaternions');
addpath('ximu_matlab_library');

% -------------------------------------------------------------------------
% Select dataset (comment in/out)

filePath = 'Datasets/data20hz11ss';
startTime = 1;
stopTime = 10;


samplePeriod = 1/20;
xIMUdata = klasifikasidata(filePath, 'InertialMagneticSampleRate', 1/samplePeriod);
time = xIMUdata.CalInertialAndMagneticData.Time;
gyrX = xIMUdata.CalInertialAndMagneticData.Gyroscope.X;
gyrY = xIMUdata.CalInertialAndMagneticData.Gyroscope.Y;
gyrZ = xIMUdata.CalInertialAndMagneticData.Gyroscope.Z;
accX = xIMUdata.CalInertialAndMagneticData.Accelerometer.X;
accY = xIMUdata.CalInertialAndMagneticData.Accelerometer.Y;
accZ = xIMUdata.CalInertialAndMagneticData.Accelerometer.Z;
mgX = xIMUdata.CalInertialAndMagneticData.Magnetometer.X;
mgY = xIMUdata.CalInertialAndMagneticData.Magnetometer.Y;
mgZ = xIMUdata.CalInertialAndMagneticData.Magnetometer.Z;
clear('xIMUdata');

indexSel = find(sign(time-startTime)+1, 1) : find(sign(time-stopTime)+1, 1);
time = time(indexSel);
gyrX = gyrX(indexSel, :);
gyrY = gyrY(indexSel, :);
gyrZ = gyrZ(indexSel, :);
accX = accX(indexSel, :);
accY = accY(indexSel, :);
accZ = accZ(indexSel, :);
mgX = mgX(indexSel, :);
mgY = mgY(indexSel, :);
mgZ = mgZ(indexSel, :);

% -------------------------------------------------------------------------
% Detect stationary periods

% Compute accelerometer magnitude
acc_mag = sqrt(accX.*accX + accY.*accY + accZ.*accZ);

% HP filter accelerometer data
filtCutOff = 0.1;
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'high');
acc_magFilt = filtfilt(b, a, acc_mag);

% Compute absolute value
acc_magFilt = abs(acc_magFilt);

% LP filter accelerometer data
filtCutOff = 2;
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');
acc_magFilt = filtfilt(b, a, acc_magFilt);

% Threshold detection
stationary = acc_magFilt < 0.03;

% -------------------------------------------------------------------------
% Plot data raw sensor data and stationary periods

figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'Sensor Data');
ax(1) = subplot(3,1,1);
    hold on;
    plot(time, gyrX, 'r');
    plot(time, gyrY, 'g');
    plot(time, gyrZ, 'b');
    title('Gyroscope');
    xlabel('Time (s)');
    ylabel('Angular velocity (^\circ/s)');
    legend('X', 'Y', 'Z');
    hold off;
ax(2) = subplot(3,1,2);
    hold on;
    plot(time, accX, 'r');
    plot(time, accY, 'g');
    plot(time, accZ, 'b');
    plot(time, acc_magFilt, ':k');
    plot(time, stationary, 'k', 'LineWidth', 2);
    title('Accelerometer');
    xlabel('Time (s)');
    ylabel('Acceleration (g)');
    legend('X', 'Y', 'Z', 'Filtered', 'Stationary');
    hold off;
linkaxes(ax,'x');
ax(1) = subplot(3,1,3);
    hold on;
    plot(time, mgX, 'r');
    plot(time, mgY, 'g');
    plot(time, mgZ, 'b');
    title('Magnetometer');
    xlabel('Time (s)');
    ylabel('uTesla');
    legend('X', 'Y', 'Z');
    hold off;
    
    % Compute accelerometer magnitude
    mg_mag = sqrt(mgX.*mgX + mgY.*mgY + mgZ.*mgZ);
    % Plot translational velocity
figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'Velocity');
hold on;
plot(time, mg_mag(:,1), 'r');
title('arah compas');
xlabel('Time (s)');
ylabel('arah');
legend('H');
hold off;

 % Compute accelerometer magnitude
    hd = atan2(mgX,mgY*180/3.1428);
    % Plot translational velocity
figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'Velocity');
hold on;
plot(time, hd(:,1), 'r');
title('arah compas');
xlabel('Time (s)');
ylabel('arah');
legend('H');
hold off;

