clear;
close all;
clc;
addpath('Quaternions');
addpath('ximu_matlab_library');
addpath('Datasets');

% -------------------------------------------------------------------------
% Select dataset (comment in/out)

 filePath = 'Datasets/ericho5';
 startTime = 0;
 stopTime = 16.5;

%  filePath = 'Datasets/stairsAndCorridor';
%  startTime = 5;
%  stopTime = 53;

% filePath = 'Datasets/spiralStairs';
% startTime = 4;
% stopTime = 47;

% -------------------------------------------------------------------------
% Import data

samplePeriod = 1/100;
xIMUdata = klasifikasidata(filePath, 'InertialMagneticSampleRate', 1/samplePeriod);
time = xIMUdata.CalInertialAndMagneticData.Time;
%dely = xIMUdata.CalInertialAndMagneticData.Delay;
gyrY = xIMUdata.CalInertialAndMagneticData.Gyroscope.X;
gyrX = xIMUdata.CalInertialAndMagneticData.Gyroscope.Y;
gyrZ = xIMUdata.CalInertialAndMagneticData.Gyroscope.Z;
accY = xIMUdata.CalInertialAndMagneticData.Accelerometer.X;
accX = xIMUdata.CalInertialAndMagneticData.Accelerometer.Y;
accZ = xIMUdata.CalInertialAndMagneticData.Accelerometer.Z;
clear('xIMUdata');

% -------------------------------------------------------------------------
% Manually frame data

% startTime = 0;
% stopTime = 10;

indexSel = find(sign(time-startTime)+1, 1) : find(sign(time-stopTime)+1, 1);
time = time(indexSel);
%delay = delay(indexSel);
gyrX = gyrX(indexSel, :);%+0.3646;
gyrY = gyrY(indexSel, :);%+0.00339;
gyrZ = gyrZ(indexSel, :);%-1.29945;
accX = accX(indexSel, :);%-0.0053;
accY = accY(indexSel, :);%-0.0726;
accZ = accZ(indexSel, :);%+0.026;

% -------------------------------------------------------------------------
% Detect stationary periods

% Compute accelerometer magnitude
acc_mag = sqrt(accX.*accX + accY.*accY + accZ.*accZ);

% HP filter accelerometer data
filtCutOff = 0.01
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'high');
acc_magFilt = filtfilt(b, a, acc_mag);

% Compute absolute value
acc_magFilt = abs(acc_magFilt);

% LP filter accelerometer data
filtCutOff = 3;
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');
acc_magFilt = filtfilt(b, a, acc_magFilt);

%acc_magFilt = acc_magFilt-2.4;
% Threshold detection
stationary = acc_magFilt < 0.05;

% -------------------------------------------------------------------------
% Plot data raw sensor data and stationary periods

 figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'Sensor Data');
ax(1) = subplot(2,1,1);
    hold on;
    plot(time, gyrX, 'r');
    plot(time, gyrY, 'g');
    plot(time, gyrZ, 'b');
    title('Gyroscope');
    xlabel('Time (s)');
    ylabel('Angular velocity (^\circ/s)');
    legend('X', 'Y', 'Z');
    hold off;
ax(2) = subplot(2,1,2);
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

% -------------------------------------------------------------------------

figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'Sensor Data');
ax(1) = subplot(3,1,1);
    hold on;
    plot(time, accX, 'r');
    plot(time, accY, 'g');
    plot(time, accZ, 'b');
    title('Accelerometer');
    xlabel('Time (s)');
    ylabel('Acceleration (g)');
    legend('X', 'Y', 'Z');
    hold off;
ax(2) = subplot(3,1,2);
    hold on;
    plot(time, acc_magFilt, 'm');
    title('Magnitude Accelerometer Filter');
    xlabel('Time (s)');
    ylabel('Acceleration (g)');
    legend('Filtered');
    hold off;
ax(3) = subplot(3,1,3);
    hold on;
    plot(time, stationary, 'k', 'LineWidth', 2);
    title('Stasionary');
    xlabel('Time (s)');
    ylabel('0=bergerak, 1=diam' );
    legend('Stationary');
    hold off;    
linkaxes(ax,'x');

% -------------------------------------------------------------------------

% Compute orientation

quat = zeros(length(time), 4);
AHRSalgorithm = AHRS('SamplePeriod', samplePeriod, 'Kp', 1, 'KpInit', 1);

% Initial convergence
initPeriod = 1;
indexSel = 1 : find(sign(time-(time(1)+initPeriod))+1, 1);
for i = 1:10
    AHRSalgorithm.UpdateIMU([0 0 0], [mean(accX(indexSel)) mean(accY(indexSel)) mean(accZ(indexSel))]);
end

% For all data
for t = 1:length(time)
    if(stationary(t))
        AHRSalgorithm.Kp = 0.5;
    else
        AHRSalgorithm.Kp = 0;
    end
    AHRSalgorithm.UpdateIMU(deg2rad([gyrX(t) gyrY(t) gyrZ(t)]), [accX(t) accY(t) accZ(t)]);
    quat(t,:) = AHRSalgorithm.Quaternion;
   
end

% -------------------------------------------------------------------------
% Compute translational accelerations

% Rotate body accelerations to Earth frame
acc = quaternRotate([accX accY accZ], quaternConj(quat));
%acb = quaternRotate([accX accY accZ], quaternConj(quat));

% % Remove gravity from measurements
% acc = acc - [zeros(length(time), 2) ones(length(time), 1)];     % unnecessary due to velocity integral drift compensation

% Convert acceleration measurements to m/s/s
acc = acc * 9.81;

% Plot translational accelerations
figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'Accelerations');
hold on;
plot(time, acc(:,1), 'r');
plot(time, acc(:,2), 'g');
plot(time, acc(:,3), 'b');
title('Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (m/s/s)');
legend('X', 'Y', 'Z');
hold off;

% -------------------------------------------------------------------------
% Compute translational velocities

acc(:,3) = acc(:,3) - 9.81;

% Integrate acceleration to yield velocity
vel = zeros(size(acc));
for t = 2:length(vel)
    vel(t,:) = vel(t-1,:) + acc(t,:) * samplePeriod;
    if(stationary(t) == 1)
        vel(t,:) = [0 0 0];     % force zero velocity when foot stationary
    end
end

% Integrate acceleration to yield velocity
vel1 = zeros(size(acc));
for t = 2:length(vel1)
    vel1(t,:) = vel1(t-1,:) + acc(t,:) * samplePeriod;
    if(stationary(t) == 1)
        vel1(t,:) = [0 0 0];     % force zero velocity when foot stationary
    end
end


% Compute integral drift during non-stationary periods
velDrift = zeros(size(vel1));
stationaryStart = find([0; diff(stationary)] == -1);
stationaryEnd = find([0; diff(stationary)] == 1);
for i = 1:numel(stationaryEnd)
    driftRate = vel(stationaryEnd(i)-1, :) / (stationaryEnd(i) - stationaryStart(i)); %cari drift rate
    enum = 1:(stationaryEnd(i) - stationaryStart(i)); %hitung banyak data i saat SE-SS
    drift = [enum'*driftRate(1) enum'*driftRate(2) enum'*driftRate(3)]; %tranpose enum, dikalikan dengan x,y,z setiap data saat enum
    velDrift(stationaryStart(i):stationaryEnd(i)-1, :) = drift;
end



% Plot translational velocity
figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'drift');
hold on;
plot(time, velDrift(:,1), 'r');
plot(time, velDrift(:,2), 'g');
plot(time, velDrift(:,3), 'b');
title('drift');
xlabel('Time (s)');
ylabel('drift (m/s)');
legend('X', 'Y', 'Z');
hold off;


% Remove integral drift
vel = vel1 - velDrift;

% Plot translational velocity
figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'Velocity');
hold on;
plot(time, vel(:,1), 'r');
plot(time, vel(:,2), 'g');
plot(time, vel(:,3), 'b');
title('Velocity');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('X', 'Y', 'Z');
hold off;

% Plot translational velocity
figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'Velocity');
hold on;
plot(time, vel1(:,1), 'r');
plot(time, vel1(:,2), 'g');
plot(time, vel1(:,3), 'b');
title('Velocity');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('X', 'Y', 'Z');
hold off;

% -------------------------------------------------------------------------
% Compute translational position

% Integrate velocity to yield position
pos = zeros(size(vel));
for t = 2:length(pos)
    pos(t,:) = pos(t-1,:) + vel(t,:) * samplePeriod + (1/2 * acc(t-1,:)* (samplePeriod*samplePeriod));    % integrate velocity to yield position
end

% Plot translational position
figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'Position');
hold on;
plot(time, pos(:,1), 'r');
plot(time, pos(:,2), 'g');
plot(time, pos(:,3), 'b');
title('Position');
xlabel('Time (s)');
ylabel('Position (m)');
legend('X', 'Y', 'Z');
hold off;

% -------------------------------------------------------------------------
% -------------------------------------------------------------------------

figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'Akselerasi ke Posisi');
ax(1) = subplot(3,1,1);
    hold on;
    plot(time, accX, 'r');
    plot(time, accY, 'g');
    plot(time, accZ, 'b');
    title('Accelerometer');
    xlabel('Time (s)');
    ylabel('Acceleration (g)');
    legend('X', 'Y', 'Z');
    hold off;
ax(2) = subplot(3,1,2);
    hold on;
    plot(time, vel(:,1), 'r');
    plot(time, vel(:,2), 'g');
    plot(time, vel(:,3), 'b');
    title('Velocity');
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    hold off;
ax(3) = subplot(3,1,3);
    hold on;
    plot(time, pos(:,1), 'r');
    plot(time, pos(:,2), 'g');
    plot(time, pos(:,3), 'b');
    title('Position');
    xlabel('Time (s)');
    ylabel('Position (m)');
    hold off;    
linkaxes(ax,'x');

% -------------------------------------------------------------------------

% Plot 3D foot trajectory

% % Remove stationary periods from data to plot
 posPlot = pos(find(~stationary), :);
 quatPlot = quat(find(~stationary), :);
posPlot = pos;
quatPlot = quat;

% Extend final sample to delay end of animation
extraTime = 1;
onesVector = ones(extraTime*(1/samplePeriod), 1);
posPlot = [posPlot; [posPlot(end, 1)*onesVector, posPlot(end, 2)*onesVector, posPlot(end, 3)*onesVector]];
quatPlot = [quatPlot; [quatPlot(end, 1)*onesVector, quatPlot(end, 2)*onesVector, quatPlot(end, 3)*onesVector, quatPlot(end, 4)*onesVector]];

% Create 6 DOF animation
SamplePlotFreq = 5;
Spin = 90;
SixDofAnimation(posPlot, quatern2rotMat(quatPlot), ...
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'DotsOnly', ...
                'Position', [9 39 1024 768], 'View', [(100:(Spin/(length(posPlot)-1)):(100+Spin))', 10*ones(length(posPlot), 1)], ...
                'AxisLength', 0.1, 'ShowArrowHead', false, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, ...
                'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));
