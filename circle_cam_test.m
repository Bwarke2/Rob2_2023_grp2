%% Setup
clc
clear all
eccentricity_old = 1;
radii_40 = 300;             % Get this value
error_old = 1;
cam = webcam;
disp(eccentricity_old)

%%

while eccentricity_old > 0.3
    image = cam.snapshot; 
    [center,minorAxis,majorAxis,radii,eccentricity] = find_circle(image);
    
    %plot the biggest circle on image
    figure(6)
    imshow(image)
    title("Image with circle")
    subtitle(eccentricity_old)
    hold on
    viscircles(center,radii);
    hold off
    
    ang_v = PID_controller(eccentricity,eccentricity_old,1,0,30);
    eccentricity_old = eccentricity;
    disp("Eccentricity: ")
    disp(eccentricity_old)
    %velMsg.Angular.Z = ang_v;
    %send(robotCmd, velMsg); % Sending velocities to Turtlebot
end
disp("Found direction")

figure(2)
imshow(image)
    title("Image with circle")
    hold on
    viscircles(center,radii);
    hold off

while abs(radii_40 - radii) > (0.2*radii_40)
    image = cam.snapshot;
    [center,minorAxis,majorAxis,radii,eccentricity] = find_circle(image);
    
    
    error = radii_40 - radii;   % Calculating error
    d = error - error_old;      % Calculating derivative of error
    lin_v = error*1 + 30*d;     % PD-controller
    error_old = error;
    disp("Radii: ")
    disp(radii)
    %velMsg.Linear.x = lin_v;
    %send(robotCmd, velMsg); % Sending velocities to Turtlebot
end
disp("Is 40cm from green circle")

