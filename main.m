% main.m: main script implemented for testing the performance of the
% developed general purpose plannar kinematics analayzer module.

%% Close any figures that are opend and clear workspace
close all; clear; clc;

%% Initialization
% Initialize the system by adding all files to the MATLAB PATH variable
Initialize;


%% Define the mechanism
% Get the mechanism for which kinematics analysis is to be performed
mechanism = defineMechanism;


%% Kinematics analysis - bodies
% solve the kinematic problem for the mechanism (this time only for bodies
% of the mechanism, next we will solve for markers as well)
bodiesKinematics = bodiesKinematics(mechanism);


%% Kinematics analysis - markers
% solve for kinematics of markers attached to the bodies of the mechanism
markersKinematics = markersKinematics(bodiesKinematics);


%% Visualization - bodies kinematic analysis result
% To generate plot of the kinematics analysis result for the entire bodies
% in the mechanism use: Visualizer(bodiesKinematics, 'b') by uncommenting
% the following

% Visualizer(bodiesKinematics, 'b')

% Or we can specify the body name(s) in a cell array for which the plot 
% will be generated
bodies = {'bodyC1', 'bodyC10'};
Visualizer(bodiesKinematics, 'b', bodies)


%% Visualization - markers kinematic analysis result
% To generate plot of kinematics result analysis for the entire 
% markers in the mechanism use: Visualizer(markersKinematics, 'm') by
% uncommenting the following

% Visualizer(markersKinematics, 'm')

% Or we can specify the body name that contains a marker of interest and
% the marker name(s) as follows
bodyName = {'bodyC8'};
markerName = {'K', 'I', 'KM'};
Visualizer(markersKinematics, 'm', bodyName, markerName)


%% Comparison with results from Adams Multibody system simulation sofware


