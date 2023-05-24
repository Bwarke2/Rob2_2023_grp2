% Final Project - Localization
clc; clear all; close all;

%% Load map
map_rgb = imread("Shannon_Bitmap_JPG.jpg");
map_gray = rgb2gray(map_rgb);
map_bin_inv = im2bw(map_gray,0.5);
map_bin = ~map_bin_inv;
map = occupancyMap(map_bin,21);
map2 = occupancyMap(map_bin,21);
inflate(map2,0.5);

% Generate path nodes
prm = mobileRobotPRM(map2,1000);
prm.ConnectionDistance = 5;
prm_start = [46.2 26.9];
prm_end_1 = [4.76 10.24];
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
odomSub = rossubscriber("/odom","DataFormat","struct");
odomMsg = receive(odomSub,3);
pause(1) %Wait to allow for setup
scansub = rossubscriber('/scan');

% PP controller setup
p_con = controllerPurePursuit();
p_con.LookaheadDistance = 0.8;
p_con.DesiredLinearVelocity = 10;
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
rangeFinderModel.SensorLimits = [0.45 8];
rangeFinderModel.Map = map;


%sensorTransform = getTransform(tftree,'/base_link', '/base_scan');
%laserQuat = [sensorTransform.Transform.Rotation.W sensorTransform.Transform.Rotation.X ...
%    sensorTransform.Transform.Rotation.Y sensorTransform.Transform.Rotation.Z];
%laserRotation = quat2eul(laserQuat, 'ZYX');

%rangeFinderModel.SensorPose = ...
%    [sensorTransform.Transform.Translation.X sensorTransform.Transform.Translation.Y laserRotation(1)];


mcl = monteCarloLocalization;

mcl.MotionModel = odometryModel;
mcl.SensorModel = rangeFinderModel;

mcl.UseLidarScan = false;
mcl.InitialPose = [46.2 26.9 angles(1)*pi/2];
mcl.SensorModel.Map = map;
%[isUpdated,pose,covariance] = mcl(odomPose,scan);

atGoal = 0;
while (atGoal == 0)
    % Get odometry data
    scan = receive(scansub);
    odomMsg = receive(odomSub,3);
    pose = odomMsg.Pose.Pose;
    x = pose.Position.X - x_zero + 46.2;
    y = pose.Position.Y - y_zero + 26.9;
    z = pose.Position.Z;

    quat = pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
    position = [x y angles(1)];
    
    % Get scan and calculate monto carlo localization
    ranges = scan.Ranges;
    angle = scan.AngleMin:scan.AngleIncrement:scan.AngleMax;
    %[isUpdated,position,covariance] = mcl(position,ranges,angle);

    [v,ang_v] = p_con([position(1),position(2),position(3)]);    
    velMsg.Linear.X = v;
    velMsg.Angular.Z = ang_v;
    send(robotCmd, velMsg);
    
    % Save position for plotting
    x_hist(end+1) = position(1);
    y_hist(end+1) = position(2);
    theta_hist(end+1) = position(3);

    %Check if near goal
    %RobotLocation = [x-x_zero,y-y_zero];
    %distanceToGoal = norm(RobotLocation - Goal);
    %if distanceToGoal < goalRadius
    %    atGoal = 1;
    %end
end

%% Plotting
figure(3)
hold on
show(map);
hold on, plot(x_hist, y_hist);
