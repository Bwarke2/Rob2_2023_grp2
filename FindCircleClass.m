classdef FindCircleClass
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        radii_40 = 112;             % Value found impericaly
        robCmd = rospublisher("/cmd_vel", "DataFormat","struct");
        velMsg = [];
        imageSub = rossubscriber('/raspicam_node/image/compressed');
    end

    methods
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
            radii = circles(1,5);
            if radii < 50
                center = [0,0];
                majorAxis = 0;
                minorAxis = 0;
                radii = 0;
                Eccentricity = 1;
                return
            end
        
            center = circles(1,1:2);
            majorAxis = circles(1,3);
            minorAxis = circles(1,4);
            
            Eccentricity = circles(1,6);
        end

        function setupRos(robotCmd)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            % Setup ros
            rosshutdown
            rpi_ip = "192.168.43.104";
            host_ip = "192.168.43.31";
            setenv('ROS_MASTER_URI','http://' + rpi_ip + ':11311')
            setenv('ROS_IP',host_ip)
            rosinit('http://'+rpi_ip+':11311','NodeHost',host_ip);
        end

        function driveToCircle(val)
            % Drives to circle
            eccentricity_old = 1;
            this.radii_40 = 112;             % Value found impericaly
            error_old = 0;
            
            robotCmd = rospublisher("/cmd_vel", "DataFormat","struct");
            this.velMsg = rosmessage(robotCmd);
            this.velMsg.Linear.X = 0.0;
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
%                 figure(6)
%                 imshow(image)
%                 title("Image with circle")
%                 hold on
%                 viscircles(center,radii);
%                 hold off
%                 disp("Eccentricity: ")
                disp("e_old:")
                disp(eccentricity_old)
                
                ang_v = PID_controller(eccentricity,0,0.2,0,1);
                %disp("ang_v:")
                %disp(ang_v)
                eccentricity_old = eccentricity;
                this.velMsg.Angular.Z = ang_v;
                send(robotCmd, this.velMsg); % Sending velocities to Turtlebot
            end
            disp("Found direction")
            %Set rotation to 0
            this.velMsg.Angular.Z = 0;
            send(robotCmd, this.velMsg); % Sending velocities to Turtlebot
            
            while abs(this.radii_40 - radii) > (0.2*this.radii_40)
                image_msg = receive(image_sub);
                image_msg.Format = "rgb8;";
                image = readImage(image_msg); 
                [center,minorAxis,majorAxis,radii,eccentricity] = find_circle(image);
                
                
                error = this.radii_40 - radii;   % Calculating error
                d = error - error_old;      % Calculating derivative of error
                lin_v = error*1 + 3*d;     % PD-controller
                error_old = error;
                disp("Radii: ")
                disp(radii)
                %eccentricity_old = eccentricity;
                this.velMsg.Linear.X = lin_v;
                send(robotCmd, this.velMsg); % Sending velocities to Turtlebot
            end
            disp("Is 40cm from green circle")
        end
    end
end