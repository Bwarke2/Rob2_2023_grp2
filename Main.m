%% Setup
clc
clear all
close all

rosshutdown
setenv('ROS_MASTER_URI','http://192.168.43.31:11311')
setenv('ROS_IP','192.168.43.143')
rosinit('http://192.168.43.31:11311','NodeHost','192.168.43.143');

map

%% Load map

%% Plan rute

%% Avoid obstacles

%% Check if at goal

%% Scan cirle