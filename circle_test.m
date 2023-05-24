%% Setup
clc
clear all
eccentricity_old = 1;
radii_40 = 112;             % Value found impericaly
error_old = 0;

% Setup ros
rosshutdown
rpi_ip = "192.168.43.104";
host_ip = "192.168.43.31";
setenv('ROS_MASTER_URI','http://' + rpi_ip + ':11311')
setenv('ROS_IP',host_ip)
rosinit('http://'+rpi_ip+':11311','NodeHost',host_ip);


robotCmd = rospublisher("/cmd_vel", "DataFormat","struct");
velMsg = rosmessage(robotCmd);
velMsg.Linear.X = 0.0;
if ismember('/raspicam_node/image/compressed',rostopic('list'))
    image_sub = rossubscriber('/raspicam_node/image/compressed');
end
%%

while eccentricity_old > 0.2
    image_msg = receive(image_sub);
    image_msg.Format = "rgb8;";
    image = readImage(image_msg); 
    [center,minorAxis,majorAxis,radii,eccentricity] = find_circle(image);
    
    %plot the biggest circle on image
    figure(6)
    imshow(image)
    title("Image with circle")
    hold on
    viscircles(center,radii);
    hold off
    disp("Eccentricity: ")
    disp(eccentricity_old)
    
    ang_v = PID_controller(eccentricity,eccentricity_old,1,0,3);
    eccentricity_old = eccentricity;
    velMsg.Angular.Z = ang_v;
    send(robotCmd, velMsg); % Sending velocities to Turtlebot
end
disp("Found direction")
%Set rotation to 0
velMsg.Angular.Z = 0;
send(robotCmd, velMsg); % Sending velocities to Turtlebot

while abs(radii_40 - radii) > (0.2*radii_40)
    image_msg = receive(image_sub);
    image_msg.Format = "rgb8;";
    image = readImage(image_msg); 
    [center,minorAxis,majorAxis,radii,eccentricity] = find_circle(image);
    
    
    error = radii_40 - radii;   % Calculating error
    d = error - error_old;      % Calculating derivative of error
    lin_v = error*1 + 3*d;     % PD-controller
    error_old = error;
    disp("Radii: ")
    disp(radii)
    eccentricity_old = eccentricity;
    velMsg.Linear.X = lin_v;
    send(robotCmd, velMsg); % Sending velocities to Turtlebot
end
disp("Is 40cm from green circle")

