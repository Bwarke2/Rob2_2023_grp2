%% Setup
clc
clear all
eccentricity_old = 1;
radii_40 = 300;             % Get this value

% Setup ros
rosshutdown
rpi_ip = "192.168.43.172";
host_ip = "192.168.43.31";
setenv('ROS_MASTER_URI','http://' + rpi_ip + ':11311')
setenv('ROS_IP',host_ip)
rosinit('http://'+rpi_ip+':11311','NodeHost',host_ip);


%robotCmd = rospublisher("/cmd_vel", "DataFormat","struct");
%velMsg = rosmessage(robotCmd);
%velMsg.Linear.X = 0.0;
if ismember('/raspicam_node/image/compressed',rostopic('list'))
    image_sub = rossubscriber('/raspicam_node/image/compressed');
end
%%

while eccentricity_old > 0.1
    image_msg = receive(image_sub);
    image_msg.Format = "rgb8;";
    image = readImage(image_msg); 
    [center,minorAxis,majorAxis,radii,eccentricity] = find_circle(image);
    
    %plot the biggest circle on image
%     figure(6)
%     imshow(image)
%     title("Image with circle")
%     hold on
%     viscircles(center,radii);
%     hold off
    
    ang_v = PID_controller(eccentricity,eccentricity_old,1,0,30);
    eccentricity_old = eccentricity;
    velMsg.Angular.Z = ang_v;
    %send(robotCmd, velMsg); % Sending velocities to Turtlebot
end
print("Found direction")

figure(2)
imshow(image)
    title("Image with circle")
    hold on
    viscircles(center,radii);
    hold off

while abs(radii_40 - radii) > (0.2*radii_40)
    image = receive(image_sub);
    image.Format = "rgb8";
    [center,minorAxis,majorAxis,radii,eccentricity] = find_circle(image);
    
    
    error = radii_40 - radii;   % Calculating error
    d = error - error_old;      % Calculating derivative of error
    lin_v = error*1 + 30*d;     % PD-controller
    error_old = error;

    eccentricity_old = eccentricity;
    velMsg.Linear.x = lin_v;
    send(robotCmd, velMsg); % Sending velocities to Turtlebot
end
print("Is 40cm from green circle")

