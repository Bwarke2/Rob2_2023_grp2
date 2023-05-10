%% Gray scale object detection example
clear
clc
image = imread('Check.png');
imshow(image)

%%
im_green = image(:,:,2);
im_red  = image(:,:,1);
im_blue  = image(:,:,3);

im_gray = rgb2gray(image);

%% Green filter

im_filt = im_green - (im_red/2 + im_blue/2);
im_filt2 = (im_filt > 10);

figure(8)
imshow(im_filt2)
title("Image filtered")


%%
% figure(1)
% imshow(im_blue)
% title("Image Blue")
% 
% figure(2)
% imshow(im_red)
% title("Image Red")
% 
% figure(3)
% imshow(im_green)
% title("Image Green")
% 
% figure(4)
% imshow(im_gray)
% title("Image 4")

%% Erode small parts of the picture
SE = strel('disk',10);
im_filt3 = imopen(im_filt2,SE);

figure(4)
imshow(im_filt3)
title("Image 4")

im_filt4 = imsharpen(double(im_filt3));

%%
stats = regionprops("table",im_filt2,"Centroid", ...
    "MajorAxisLength","MinorAxisLength","Eccentricity");
centers = stats.Centroid(:,:);  %Get centers
diameters = mean([stats.MajorAxisLength stats.MinorAxisLength],2);
radii = diameters/2;

%Get the biggest circle
circles = [centers,stats.MajorAxisLength,stats.MinorAxisLength,radii,stats.Eccentricity];
circles = sortrows(circles,4,"descend");
main_circle = circles(1,:);

%plot the biggest circle on image
figure(6)
imshow(image)
title("Image with circle")
hold on
viscircles(main_circle(1:2),main_circle(5));
hold off


%% Get the values for controller
% Get radii at 40 cm
% Eccentricity needs to be minimized 
    error_old = 0;
    k_p = 3; 
    k_d = 30;
    error = main_circle(6);         % Use essentricity as error
    d = error - error_old;          % Calculating derivative of error
    ang_v = error*k_p + d*k_d;      % PD-controller

    error_old = error;      % Updating the previous error

    velMsg.Linear.X = 0.1;  
    velMsg.Angular.Z = ang_v;
    %send(robotCmd, velMsg); % Sending velocities to Turtlebot

