% main.m: main script implemented for testing the performance of the
% developed general purpose plannar mechanism kinematics analayzer module.

%% Close any figures that are opend and clear workspace
close all; clear; clc;

%% Initialization
% Initialize the system by adding all files to the MATLAB PATH variable
Initialize;

%% Define the mechanism
% Get the mechanism for which kinematics analysis is to be performed
mechanism = defineMechanism;


%% Kinematics analysis
% solve the kinematic problem for the mechanism bodies and their marker
% points.
endTime = 5;
steps = 100;
kinematics = Kinematics(mechanism, endTime, steps);


%% Visualization
% To generate plot of kinematics result analysis for the entire 
% bodies or markers in the mechanism use: 

% Visualizer(markersKinematics, 'b')
% Visualizer(markersKinematics, 'm')

% Or we can specify the body name of interest
bodyNames = {'bodyC1', 'bodyC10'};
Visualizer(kinematics, 'b', bodyNames)

% The same thing for markers as well
bodyNames = {'bodyC8'};
markerNames = {'K', 'I'};
Visualizer(kinematics, 'm', bodyNames, markerNames)


%% Comparison with results from Adams Multibody system simulation sofware
% Here the comparsion we will be done for bodyC10.

% Kinematics of bodyC10 obtained from the MATLAB implementation
% Position
bodyC10X = kinematics.bodyC10.q0(1, :);
bodyC10Y = kinematics.bodyC10.q0(2, :);
bodyC10Phi = kinematics.bodyC10.q0(3, :);

% Velocity
bodyC10VX = kinematics.bodyC10.dq(1, :);
bodyC10VY = kinematics.bodyC10.dq(2, :);
bodyC10dPhi = kinematics.bodyC10.dq(3, :);

% Acceleration
bodyC10AX = kinematics.bodyC10.ddq(1, :);
bodyC10AY = kinematics.bodyC10.ddq(2, :);
bodyC10ddPhi = kinematics.bodyC10.ddq(3, :);

% Kinematics of bodyC10 obtained from Adams
warningState = warning('off', 'MATLAB:table:ModifiedAndSavedVarnames');
bodyC10XAdams = readtable("testData/forMainMech/bodyC10CmX.txt");
bodyC10XAdams = table2array(bodyC10XAdams(:, 2));
bodyC10YAdams = readtable("testData/forMainMech/bodyC10CmY.txt");
bodyC10YAdams = table2array(bodyC10YAdams(:, 2));

bodyC10VxAdams = readtable("testData/forMainMech/bodyC10CmVx.txt");
bodyC10VxAdams = table2array(bodyC10VxAdams(:, 2));
bodyC10VyAdams = readtable("testData/forMainMech/bodyC10CmVy.txt");
bodyC10VyAdams = table2array(bodyC10VyAdams(:, 2));
bodyC10dPhiAdams = readtable("testData/forMainMech/bodyC10dPhi.txt");
bodyC10dPhiAdams = table2array(bodyC10dPhiAdams(:, 2));

bodyC10AxAdams = readtable("testData/forMainMech/bodyC10CmAx.txt");
bodyC10AxAdams = table2array(bodyC10AxAdams(:, 2));
bodyC10AyAdams = readtable("testData/forMainMech/bodyC10CmAy.txt");
bodyC10AyAdams = table2array(bodyC10AyAdams(:, 2));
bodyC10ddPhiAdams = readtable("testData/forMainMech/bodyC10ddPhi.txt");
bodyC10ddPhiAdams = table2array(bodyC10ddPhiAdams(:, 2));
warning(warningState);

%% Qualitative comparison with plots
% Position
% X
figure
plot(kinematics.time, bodyC10X, "k", "LineWidth", 2, "DisplayName", "MATLAB")
hold on
plot(kinematics.time, bodyC10XAdams, "--r", "LineWidth", 2, "DisplayName", "Imported from Adams")
hold off
title("Position along X of bodyC10")
ylabel("Postion X [m]", "FontSize",13, "FontWeight","bold")
xlabel("Time [s]", "FontSize",13, "FontWeight","bold")
legend("Location", "best", "FontSize", 13, "FontWeight", "bold")
grid on

% Y
figure
plot(kinematics.time, bodyC10Y, "k", "LineWidth", 2, "DisplayName", "MATLAB")
hold on
plot(kinematics.time, bodyC10YAdams, "--r", "LineWidth", 2, "DisplayName", "Imported from Adams")
hold off
title("Position along Y of bodyC10")
ylabel("Postion Y [m]", "FontSize",13, "FontWeight","bold")
xlabel("Time [s]", "FontSize",13, "FontWeight","bold")
legend("Location", "best", "FontSize", 13, "FontWeight", "bold")
grid on

% Velocity
% X
figure
plot(kinematics.time, bodyC10VX, "k", "LineWidth", 2, "DisplayName", "MATLAB")
hold on
plot(kinematics.time, bodyC10VxAdams, "--r", "LineWidth", 2, "DisplayName", "Imported from Adams")
hold off
title("Velocity along X of bodyC10")
ylabel("Velocity in X [m/s]", "FontSize",13, "FontWeight","bold")
xlabel("Time [s]", "FontSize",13, "FontWeight","bold")
legend("Location", "best", "FontSize", 13, "FontWeight", "bold")
grid on

% Y
figure
plot(kinematics.time, bodyC10VY, "k", "LineWidth", 2, "DisplayName", "MATLAB")
hold on
plot(kinematics.time, bodyC10VyAdams, "--r", "LineWidth", 2, "DisplayName", "Imported from Adams")
hold off
title("Velocity along Y of bodyC10")
ylabel("Velocity in Y [m/s]", "FontSize",13, "FontWeight","bold")
xlabel("Time [s]", "FontSize",13, "FontWeight","bold")
legend("Location", "best", "FontSize", 13, "FontWeight", "bold")
grid on

% About Z
figure
plot(kinematics.time, bodyC10dPhi, "k", "LineWidth", 2, "DisplayName", "MATLAB")
hold on
plot(kinematics.time, bodyC10dPhiAdams, "--r", "LineWidth", 2, "DisplayName", "Imported from Adams")
hold off
title("Angular velocity about Z of bodyC10")
ylabel("Angular velocity [rad/s]", "FontSize",13, "FontWeight","bold")
xlabel("Time [s]", "FontSize",13, "FontWeight","bold")
legend("Location", "best", "FontSize", 13, "FontWeight", "bold")
grid on


% Acceleration
% X
figure
plot(kinematics.time, bodyC10AX, "k", "LineWidth", 2, "DisplayName", "MATLAB")
hold on
plot(kinematics.time, bodyC10AxAdams, "--r", "LineWidth", 2, "DisplayName", "Imported from Adams")
hold off
title("Acceleration along X of bodyC10")
ylabel("Acceleration in X [m/s^2]", "FontSize",13, "FontWeight","bold")
xlabel("Time [s]", "FontSize",13, "FontWeight","bold")
legend("Location", "best", "FontSize", 13, "FontWeight", "bold")
grid on

% Y
figure
plot(kinematics.time, bodyC10AY, "k", "LineWidth", 2, "DisplayName", "MATLAB")
hold on
plot(kinematics.time, bodyC10AyAdams, "--r", "LineWidth", 2, "DisplayName", "Imported from Adams")
hold off
title("Acceleration along Y of bodyC10")
ylabel("Acceleration in Y [m/s^2]", "FontSize",13, "FontWeight","bold")
xlabel("Time [s]", "FontSize",13, "FontWeight","bold")
legend("Location", "best", "FontSize", 13, "FontWeight", "bold")
grid on

% About Z
figure
plot(kinematics.time, bodyC10ddPhi, "k", "LineWidth", 2, "DisplayName", "MATLAB")
hold on
plot(kinematics.time, bodyC10ddPhiAdams, "--r", "LineWidth", 2, "DisplayName", "Imported from Adams")
hold off
title("Angular acceleration about Z of bodyC10")
ylabel("Angular acceleration [rad/s^2]", "FontSize",13, "FontWeight","bold")
xlabel("Time [s]", "FontSize",13, "FontWeight","bold")
legend("Location", "best", "FontSize", 13, "FontWeight", "bold")
grid on


%% RMSE computation
bodyC10rmsX = MyRMS(bodyC10X, bodyC10XAdams');
bodyC10rmsY = MyRMS(bodyC10Y, bodyC10YAdams');

bodyC10rmsVx = MyRMS(bodyC10VX, bodyC10VxAdams');
bodyC10rmsVy = MyRMS(bodyC10VY, bodyC10VyAdams');
bodyC10rmsOmegaZ = MyRMS(bodyC10dPhi, bodyC10dPhiAdams');

bodyC10rmsAx = MyRMS(bodyC10AX, bodyC10AxAdams');
bodyC10rmsAy = MyRMS(bodyC10AY, bodyC10AyAdams');
bodyC10rmsAlphaZ = MyRMS(bodyC10ddPhi, bodyC10ddPhiAdams');

disp("#############   Error analysis for bodyC10 ###############")
fprintf("RMSE in position X: %e\n", bodyC10rmsX)
fprintf("RMSE in position Y: %e\n\n", bodyC10rmsY)

fprintf("RMSE in velocity VX: %e\n", bodyC10rmsVx)
fprintf("RMSE in velocity VY: %e\n", bodyC10rmsVy)
fprintf("RMSE in angular velocity about Z : %e\n\n", bodyC10rmsOmegaZ)

fprintf("RMSE in acceleration AX: %e\n", bodyC10rmsAx)
fprintf("RMSE in acceleration AY: %e\n", bodyC10rmsAy)
fprintf("RMSE in angular acceleration about Z: %e\n", bodyC10rmsAlphaZ)
disp("#############   Error analysis for bodyC10 END  ###############")
fprintf("\n")

%% Comparison for point K
% Position
bodyC8MarkerKX = kinematics.bodyC8.markers.K.q(1, :);
bodyC8MarkerKY = kinematics.bodyC8.markers.K.q(2, :);
bodyC8MarkerKPhi = kinematics.bodyC8.markers.K.q(3, :);

% Velocity
bodyC8MarkerKVx = kinematics.bodyC8.markers.K.dq(1, :);
bodyC8MarkerKVy = kinematics.bodyC8.markers.K.dq(2, :);
bodyC8MarkerKdPhi = kinematics.bodyC8.markers.K.dq(3, :);

% Acceleration
bodyC8MarkerKAx = kinematics.bodyC8.markers.K.ddq(1, :);
bodyC8MarkerKAy = kinematics.bodyC8.markers.K.ddq(2, :);
bodyC8MarkerKddPhi = kinematics.bodyC8.markers.K.ddq(3, :);

% Kinematics of marker K obtained from Adams
warningState = warning('off', 'MATLAB:table:ModifiedAndSavedVarnames');
bodyC8MarkerKXAdams = readtable("testData/forMainMech/bodyC8MarkerKX.txt");
bodyC8MarkerKXAdams = table2array(bodyC8MarkerKXAdams(:, 2));
bodyC8MarkerKYAdams = readtable("testData/forMainMech/bodyC8MarkerKY.txt");
bodyC8MarkerKYAdams = table2array(bodyC8MarkerKYAdams(:, 2));

bodyC8MarkerKVxAdams = readtable("testData/forMainMech/bodyC8MarkerKVx.txt");
bodyC8MarkerKVxAdams = table2array(bodyC8MarkerKVxAdams(:, 2));
bodyC8MarkerKVyAdams = readtable("testData/forMainMech/bodyC8MarkerKVy.txt");
bodyC8MarkerKVyAdams = table2array(bodyC8MarkerKVyAdams(:, 2));
bodyC8MarkerKdPhiAdams = readtable("testData/forMainMech/bodyC8MarkerKdPhi.txt");
bodyC8MarkerKdPhiAdams = table2array(bodyC8MarkerKdPhiAdams(:, 2));

bodyC8MarkerKAxAdams = readtable("testData/forMainMech/bodyC8MarkerKAx.txt");
bodyC8MarkerKAxAdams = table2array(bodyC8MarkerKAxAdams(:, 2));
bodyC8MarkerKAyAdams = readtable("testData/forMainMech/bodyC8MarkerKAy.txt");
bodyC8MarkerKAyAdams = table2array(bodyC8MarkerKAyAdams(:, 2));
bodyC8MarkerKddPhiAdams = readtable("testData/forMainMech/bodyC8MarkerKddPhi.txt");
bodyC8MarkerKddPhiAdams = table2array(bodyC8MarkerKddPhiAdams(:, 2));
warning(warningState);

%% Qualitative comparison with plots
% Position
% X
figure
plot(kinematics.time, bodyC8MarkerKX, "k", "LineWidth", 2, "DisplayName", "MATLAB")
hold on
plot(kinematics.time, bodyC8MarkerKXAdams, "--r", "LineWidth", 2, "DisplayName", "Imported from Adams")
hold off
title("Position along X of bodyC8 Marker K")
ylabel("Postion X [m]", "FontSize",13, "FontWeight","bold")
xlabel("Time [s]", "FontSize",13, "FontWeight","bold")
legend("Location", "best", "FontSize", 13, "FontWeight", "bold")
grid on

% Y
figure
plot(kinematics.time, bodyC8MarkerKY, "k", "LineWidth", 2, "DisplayName", "MATLAB")
hold on
plot(kinematics.time, bodyC8MarkerKYAdams, "--r", "LineWidth", 2, "DisplayName", "Imported from Adams")
hold off
title("Position along Y of bodyC8 Marker K")
ylabel("Postion Y [m]", "FontSize",13, "FontWeight","bold")
xlabel("Time [s]", "FontSize",13, "FontWeight","bold")
legend("Location", "best", "FontSize", 13, "FontWeight", "bold")
grid on

% Velocity
% X
figure
plot(kinematics.time, bodyC8MarkerKVx, "k", "LineWidth", 2, "DisplayName", "MATLAB")
hold on
plot(kinematics.time, bodyC8MarkerKVxAdams, "--r", "LineWidth", 2, "DisplayName", "Imported from Adams")
hold off
title("Velocity along X of bodyC8 Marker K")
ylabel("Velocity in X [m/s]", "FontSize",13, "FontWeight","bold")
xlabel("Time [s]", "FontSize",13, "FontWeight","bold")
legend("Location", "best", "FontSize", 13, "FontWeight", "bold")
grid on

% Y
figure
plot(kinematics.time, bodyC8MarkerKVy, "k", "LineWidth", 2, "DisplayName", "MATLAB")
hold on
plot(kinematics.time, bodyC8MarkerKVyAdams, "--r", "LineWidth", 2, "DisplayName", "Imported from Adams")
hold off
title("Velocity along Y of bodyC8 Marker K")
ylabel("Velocity in Y [m/s]", "FontSize",13, "FontWeight","bold")
xlabel("Time [s]", "FontSize",13, "FontWeight","bold")
legend("Location", "best", "FontSize", 13, "FontWeight", "bold")
grid on

% About Z
figure
plot(kinematics.time, bodyC8MarkerKdPhi, "k", "LineWidth", 2, "DisplayName", "MATLAB")
hold on
plot(kinematics.time, bodyC8MarkerKdPhiAdams, "--r", "LineWidth", 2, "DisplayName", "Imported from Adams")
hold off
title("Angular velocity about Z of bodyC8 Marker K")
ylabel("Angular velocity [rad/s]", "FontSize",13, "FontWeight","bold")
xlabel("Time [s]", "FontSize",13, "FontWeight","bold")
legend("Location", "best", "FontSize", 13, "FontWeight", "bold")
grid on


% Acceleration
% X
figure
plot(kinematics.time, bodyC8MarkerKAx, "k", "LineWidth", 2, "DisplayName", "MATLAB")
hold on
plot(kinematics.time, bodyC8MarkerKAxAdams, "--r", "LineWidth", 2, "DisplayName", "Imported from Adams")
hold off
title("Acceleration along X of bodyC8 Marker K")
ylabel("Acceleration in X [m/s^2]", "FontSize",13, "FontWeight","bold")
xlabel("Time [s]", "FontSize",13, "FontWeight","bold")
legend("Location", "best", "FontSize", 13, "FontWeight", "bold")
grid on

% Y
figure
plot(kinematics.time, bodyC8MarkerKAy, "k", "LineWidth", 2, "DisplayName", "MATLAB")
hold on
plot(kinematics.time, bodyC8MarkerKAyAdams, "--r", "LineWidth", 2, "DisplayName", "Imported from Adams")
hold off
title("Acceleration along Y of bodyC8 Marker K")
ylabel("Acceleration in Y [m/s^2]", "FontSize",13, "FontWeight","bold")
xlabel("Time [s]", "FontSize",13, "FontWeight","bold")
legend("Location", "best", "FontSize", 13, "FontWeight", "bold")
grid on

% About Z
figure
plot(kinematics.time, bodyC8MarkerKddPhi, "k", "LineWidth", 2, "DisplayName", "MATLAB")
hold on
plot(kinematics.time, bodyC8MarkerKddPhiAdams, "--r", "LineWidth", 2, "DisplayName", "Imported from Adams")
hold off
title("Angular acceleration about Z of bodyC8 Marker K")
ylabel("Angular acceleration [rad/s^2]", "FontSize",13, "FontWeight","bold")
xlabel("Time [s]", "FontSize",13, "FontWeight","bold")
legend("Location", "best", "FontSize", 13, "FontWeight", "bold")
grid on


%% RMSE computation
bodyC8MarkerKrmsX = MyRMS(bodyC8MarkerKX, bodyC8MarkerKXAdams');
bodyC8MarkerKrmsY = MyRMS(bodyC8MarkerKY, bodyC8MarkerKYAdams');

bodyC8MarkerKrmsVx = MyRMS(bodyC8MarkerKVx, bodyC8MarkerKVxAdams');
bodyC8MarkerKrmsVy = MyRMS(bodyC8MarkerKVy, bodyC8MarkerKVyAdams');
bodyC8MarkerKrmsdPhi = MyRMS(bodyC8MarkerKdPhi, bodyC8MarkerKdPhiAdams');

bodyC8MarkerKrmsAx = MyRMS(bodyC8MarkerKAx, bodyC8MarkerKAxAdams');
bodyC8MarkerKrmsAy = MyRMS(bodyC8MarkerKAy, bodyC8MarkerKAyAdams');
bodyC8MarkerKrmsddPhi = MyRMS(bodyC8MarkerKddPhi, bodyC8MarkerKddPhiAdams');

disp("#############   Error analysis for bodyC8 Marker K ###############")
fprintf("RMSE in position X: %e\n", bodyC8MarkerKrmsX)
fprintf("RMSE in position Y: %e\n\n", bodyC8MarkerKrmsY)

fprintf("RMSE in velocity VX: %e\n", bodyC8MarkerKrmsVx)
fprintf("RMSE in velocity VY: %e\n", bodyC8MarkerKrmsVy)
fprintf("RMSE in angular velocity about Z : %e\n\n", bodyC8MarkerKrmsdPhi)

fprintf("RMSE in acceleration AX: %e\n", bodyC8MarkerKrmsAx)
fprintf("RMSE in acceleration AY: %e\n", bodyC8MarkerKrmsAy)
fprintf("RMSE in angular acceleration about Z: %e\n", bodyC8MarkerKrmsddPhi)
disp("#############   Error analysis for bodyC8 Marker K END  ###############")
fprintf("\n")


%% For the test mechanism
testMech = testMechanism;

%% Kinematic analysis for text mechanism
testKinematics = Kinematics(testMech, 1.5, 30);

%% Comparison with results from Adams Multibody system simulation sofware
% Here the comparsion we will be done for bodyC2.

% Kinematics of bodyC2 obtained from the MATLAB implementation
% Possition
bodyC2X = testKinematics.bodyC2.q0(1, :);
% Velocity
bodyC2Vx = testKinematics.bodyC2.dq(1, :);
% Acceleration
bodyC2Ax = testKinematics.bodyC2.ddq(1, :);

% Results obtained from Adams
warningState = warning('off', 'MATLAB:table:ModifiedAndSavedVarnames');
bodyC2XAdams = readtable("testData/forTestMech/bodyC2X.txt");
bodyC2XAdams = table2array(bodyC2XAdams(:, 2));

bodyC2VxAdams = readtable("testData/forTestMech/bodyC2Vx.txt");
bodyC2VxAdams = table2array(bodyC2VxAdams(:, 2));

bodyC2AxAdams = readtable("testData/forTestMech/bodyC2Ax.txt");
bodyC2AxAdams = table2array(bodyC2AxAdams(:, 2));
warning(warningState);

%% plot the results for test Mechanism
% Position
figure
plot(testKinematics.time, bodyC2X, "k", "LineWidth",1.5, "DisplayName", "MATLAB")
hold on
plot(testKinematics.time, bodyC2XAdams, "--r", "LineWidth",1.5, "DisplayName", "Imported from Adams")
hold off
title("Position along X of body 2")
ylabel("Postion in X [m]", "FontSize",13, "FontWeight","bold")
xlabel("Time [s]", "FontSize",13, "FontWeight","bold")
legend("Location", "best", "FontSize", 13, "FontWeight", "bold")
grid on

% Velocity
figure
plot(testKinematics.time, bodyC2Vx, "k", "LineWidth",1.5, "DisplayName", "MATLAB")
hold on
plot(testKinematics.time, bodyC2VxAdams, "--r", "LineWidth",1.5, "DisplayName", "Imported from Adams")
hold off
title("Velocity along X of body 2")
ylabel("Velocity in X [m/s]", "FontSize",13, "FontWeight","bold")
xlabel("Time [s]", "FontSize",13, "FontWeight","bold")
legend("Location", "best", "FontSize", 13, "FontWeight", "bold")
grid on

% Acceleration
figure
plot(testKinematics.time, bodyC2Ax, "k", "LineWidth",1.5, "DisplayName", "MATLAB")
hold on
plot(testKinematics.time, bodyC2AxAdams, "--r", "LineWidth",1.5, "DisplayName", "Imported from Adams")
hold off
title("Acceleration along X of body 2")
ylabel("Acceleration in X [m/s^2]", "FontSize",13, "FontWeight","bold")
xlabel("Time [s]", "FontSize",13, "FontWeight","bold")
legend("Location", "best", "FontSize", 13, "FontWeight", "bold")
grid on

%% Performing RMS comparison with results obtained form Adams
bodyC2rmsX = MyRMS(bodyC2X, bodyC2XAdams');
bodyC2rmsVx = MyRMS(bodyC2Vx, bodyC2VxAdams');
bodyC2rmsAx = MyRMS(bodyC2Ax,bodyC2AxAdams');


disp("#############   Error analysis for test mechanism bodyC2 ###############")
fprintf("RMSE in position X: %e\n", bodyC2rmsX)
fprintf("RMSE in velocity VX: %e\n", bodyC2rmsVx)
fprintf("RMSE in acceleration AX: %e\n", bodyC2rmsAx)
disp("#############   Error analysis for bodyC10 END  ###############")
fprintf("\n")
