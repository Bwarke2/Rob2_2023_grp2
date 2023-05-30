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

FCC = FindCircleClass;
FCC.setupRos();
FCC.driveToCircle();