% Final Project - Localization
clc; clear all; close all;

%% Load map
%map_rgb = imread("Shannon_Bitmap_JPG.jpg");
%map_rgb = imread("Shannon_v2.jpg");
map_rgb = imread("Shannon_v3.jpg");
map_gray = rgb2gray(map_rgb);
map_bin_inv = im2bw(map_gray,0.5);
map_bin = ~map_bin_inv;
map = occupancyMap(map_bin,21);
map2 = occupancyMap(map_bin,21);
inflate(map2,0.5);
show(map)

% Generate path nodes
prm = mobileRobotPRM(map2,1000);
prm.ConnectionDistance = 5;
prm_start = [48 25];
prm_end_1 = [5 10];
prm_end_2 = [26.7 2.38];
show(prm)
path = findpath(prm,prm_start,prm_end_1);

figure(1);
% Unused
figure(2);
show(map);
hold on, plot(prm_start(1), prm_start(2), 'r*'), text(prm_start(1), prm_start(2), 'START')
hold on, plot(prm_end_1(1), prm_end_1(2), 'ro'), text(prm_end_1(1), prm_end_2(2), 'CP1')
hold on, plot(prm_end_2(1), prm_end_2(2), 'ro'), text(prm_end_2(2), prm_end_2(2), 'CP2')
plot(path(:,1), path(:,2))
%%
% ROS setup
robotCmd = rospublisher("/cmd_vel", "DataFormat","struct");
velMsg = rosmessage(robotCmd);
odomSub = rossubscriber("/odom");
odomMsg = receive(odomSub,3);
scansub = rossubscriber("/scan");
pause(1) %Wait to allow for setup

% PP controller setup
p_con = controllerPurePursuit();
p_con.LookaheadDistance = 0.8;
p_con.DesiredLinearVelocity = 3;
p_con.MaxAngularVelocity = pi/2;
p_con.Waypoints = path;

% Calculating initial position. (Kinda unused for monte carlo)
odomMsg = receive(odomSub,3);
pose = odomMsg.Pose.Pose;
x = pose.Position.X;
y = pose.Position.Y;
quat = pose.Orientation;
angles = quat2eul([quat.W quat.X quat.Y quat.Z]);  
x_zero = x;
y_zero = y;

% Path history for plotting
x_hist = [];
y_hist = [];
theta_hist = [];

% Monte Carlo Localization
odometryModel = odometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];
rangeFinderModel = likelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0.15 4];
rangeFinderModel.Map = map;
rangeFinderModel.SensorPose = [0 0 0];
rangeFinderModel.NumBeams = 360;
rangeFinderModel.MeasurementNoise = 1;

mcl = monteCarloLocalization;
mcl.SensorModel.Map = map;

mcl.UseLidarScan = true;
mcl.MotionModel = odometryModel;
mcl.SensorModel = rangeFinderModel;

mcl.UpdateThresholds = [0.0, 0.0, 0.0];
mcl.ResamplingInterval = 3;
%mcl.ResamplingInterval = 1;

mcl.ParticleLimits = [500 5000];
%mcl.ParticleLimits = [500 2000];
mcl.GlobalLocalization = false;
mcl.InitialPose = [48 25.0 1.5*pi];
mcl.InitialCovariance = eye(3)*0.5;


visualizationHelper = ExampleHelperAMCLVisualization(map);
i = 0;
atGoal = 0;
while (atGoal == 0)
    % Get scans and odometry data
    scan = lidarScan(receive(scansub));
    ranges = double(scan.Ranges);

    range_min = min(ranges);
    if range_min < 0.40 %if something is within 40 cm
        
    end

    %angle = scan.AngleMin:scan.AngleIncrement:scan.AngleMax;

    %odomMsg = receive(odomSub,3);
    odomMsg = odomSub.LatestMessage;
    odomQuat = [odomMsg.Pose.Pose.Orientation.W, odomMsg.Pose.Pose.Orientation.X, ...
    odomMsg.Pose.Pose.Orientation.Y, odomMsg.Pose.Pose.Orientation.Z];
    odomRotation = quat2eul(odomQuat);
    pose = [odomMsg.Pose.Pose.Position.X, odomMsg.Pose.Pose.Position.Y odomRotation(1)];

    %pose = odomMsg.Pose.Pose;
    %x = pose.Position.X;
    %y = pose.Position.Y;
    %z = pose.Position.Z;

    %quat = pose.Orientation;
    %angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
    %position = [x y angles(1)];
    
    % Monto carlo localization
    %[isUpdated,est_position,covariance] = mcl(position,lidarScan(ranges,angle));
    %[isUpdated,est_position,covariance] = mcl(pose,lidarScan(ranges,angle));
    [isUpdated,est_position,covariance] = mcl(pose,scan);

    [v,ang_v] = p_con([est_position(1),est_position(2),est_position(3)]);    
    velMsg.Linear.X = v;
    velMsg.Angular.Z = ang_v;
    send(robotCmd, velMsg);
    
    % Save position for plotting
    x_hist(end+1) = est_position(1);
    y_hist(end+1) = est_position(2);
    theta_hist(end+1) = est_position(3);
    
    if isUpdated
        i = i + 1;
        plotStep(visualizationHelper, mcl, est_position, scan, i)
        if mod(i,70) == 0
            release(mcl)
            mcl = monteCarloLocalization;
            mcl.SensorModel.Map = map;
            
            mcl.UseLidarScan = true;
            mcl.MotionModel = odometryModel;
            mcl.SensorModel = rangeFinderModel;
            
            mcl.UpdateThresholds = [0.0, 0.0, 0.0];
            mcl.ResamplingInterval = 3;
            %mcl.ResamplingInterval = 1;
            
            mcl.ParticleLimits = [500 5000];
            %mcl.ParticleLimits = [500 2000];
            mcl.InitialPose = est_position;
            mcl.InitialCovariance = eye(3)*0.5;
        end
    end
    
end

%% Find circle
FCC = FindCircleClass;
FCC.driveToCircle();

%%
% Generate path nodes
prm = mobileRobotPRM(map2,1000);
prm.ConnectionDistance = 5;
show(prm)
path = findpath(prm,prm_start,prm_end_1);

figure(1);
% Unused
figure(2);
show(map);
hold on, plot(prm_start(1), prm_start(2), 'r*'), text(prm_start(1), prm_start(2), 'START')
hold on, plot(prm_end_1(1), prm_end_1(2), 'ro'), text(prm_end_1(1), prm_end_2(2), 'CP1')
hold on, plot(prm_end_2(1), prm_end_2(2), 'ro'), text(prm_end_2(2), prm_end_2(2), 'CP2')
plot(path(:,1), path(:,2))



%%
% Monte Carlo Localization
odometryModel = odometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];
rangeFinderModel = likelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0.15 4];
rangeFinderModel.Map = map;
rangeFinderModel.SensorPose = [0 0 0];
rangeFinderModel.NumBeams = 360;
rangeFinderModel.MeasurementNoise = 1;

mcl = monteCarloLocalization;
mcl.SensorModel.Map = map;

mcl.UseLidarScan = true;
mcl.MotionModel = odometryModel;
mcl.SensorModel = rangeFinderModel;

mcl.UpdateThresholds = [0.0, 0.0, 0.0];
mcl.ResamplingInterval = 3;
%mcl.ResamplingInterval = 1;

mcl.ParticleLimits = [500 5000];
%mcl.ParticleLimits = [500 2000];
mcl.GlobalLocalization = false;
mcl.InitialPose = est_position;
mcl.InitialCovariance = eye(3)*0.5;

% setup values for PD
avoid_dist = 0.30;
error_old = 0;

% Distance to goal that is accepted
goalRadius = 0.3;

visualizationHelper = ExampleHelperAMCLVisualization(map);
i = 0;
atB = 0;
while (atB == 0)
    % Get scans and odometry data
    scan = lidarScan(receive(scansub));
    ranges = double(scan.Ranges);
    odomMsg = odomSub.LatestMessage;
    odomQuat = [odomMsg.Pose.Pose.Orientation.W, odomMsg.Pose.Pose.Orientation.X, ...
    odomMsg.Pose.Pose.Orientation.Y, odomMsg.Pose.Pose.Orientation.Z];
    odomRotation = quat2eul(odomQuat);
    pose = [odomMsg.Pose.Pose.Position.X, odomMsg.Pose.Pose.Position.Y odomRotation(1)];

    [isUpdated,est_position,covariance] = mcl(pose,scan);

    [v,ang_v] = p_con([est_position(1),est_position(2),est_position(3)]);    
    velMsg.Linear.X = v;
    velMsg.Angular.Z = ang_v;
    send(robotCmd, velMsg);
    
    error_old = 0; %There is no error

    % Save position for plotting
    x_hist(end+1) = est_position(1);
    y_hist(end+1) = est_position(2);
    theta_hist(end+1) = est_position(3);
    
    if isUpdated
        i = i + 1;
        plotStep(visualizationHelper, mcl, est_position, scan, i)
        if mod(i,50) == 0
            release(mcl)
            mcl = setupMCL(odometryModel,rangeFinderModel,est_position,map);
        end
    end

    distanceToGoal = norm(est_position(1:2) - prm_end_2);
    if distanceToGoal < goalRadius
        atGoal = 1;
    end
end


%% Plotting
figure(4)
hold on
show(map);
hold on, plot(x_hist, y_hist);
