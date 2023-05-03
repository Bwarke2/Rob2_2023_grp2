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
    "MajorAxisLength","MinorAxisLength");
centers = stats.Centroid;
diameters = mean([stats.MajorAxisLength stats.MinorAxisLength],2);
radii = diameters/2;

figure(6)
imshow(im_filt2)
title("Image filtered")
hold on
viscircles(centers,radii);
hold off


%%
% [centers, radii, metric] = imfindcircles(im_gray,[100 1000],"ObjectPolarity","dark","Sensitivity",0.95);
% figure(8)
% imshow(im_gray)
% title("Image filtered")
% hold on
% viscircles(centers,radii);
% hold off