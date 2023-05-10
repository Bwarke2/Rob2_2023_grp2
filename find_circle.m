function [center,majorAxis,minorAxis,radii,Eccentricity] = find_circle(image)
%   Find_circle: Finds the largest green circle in a picture
%   Finds the largest green circle in a picture
    im_green = image(:,:,2);
    im_red  = image(:,:,1);
    im_blue  = image(:,:,3);
    %im_gray = rgb2gray(image);
    
    % Green filter
    im_filt = im_green - (im_red/2 + im_blue/2);
    im_filt2 = (im_filt > 10);
    
    %Find circles
    stats = regionprops("table",im_filt2,"Centroid", ...
    "MajorAxisLength","MinorAxisLength","Eccentricity");
    if isempty(stats)
        center = [0,0];
        majorAxis = 0;
        minorAxis = 0;
        radii = 0;
        Eccentricity = 1;
        return
    end
    centers = stats.Centroid(:,:);  %Get centers
    diameters = mean([stats.MajorAxisLength stats.MinorAxisLength],2);
    radiis = diameters/2;
    
    %Get the biggest circle
    circles = [centers,stats.MajorAxisLength,stats.MinorAxisLength,radiis,stats.Eccentricity];
    circles = sortrows(circles,4,"descend");
    center = circles(1,1:2);
    majorAxis = circles(1,3);
    minorAxis = circles(1,4);
    radii = circles(1,5);
    Eccentricity = circles(1,6);
end
