clc
clear all
eccentricity_old = 1;

    image = imread("Check.png");
    [center,minorAxis,majorAxis,radii,eccentricity] = find_circle(image);
    
    %plot the biggest circle on image
    figure(6)
    imshow(image)
    title("Image with circle")
    hold on
    viscircles(center,radii);
    hold off
    
    ang_v = PID_controller(eccentricity,eccentricity_old,1,0,30);
    eccentricity_old = eccentricity;
    velMsg.Angular.Z = ang_v;