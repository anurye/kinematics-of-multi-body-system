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


%% Kinematics analysis - bodies
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
bodyNames = {'bodyC8', 'bodyC10'};
Visualizer(kinematics, 'b', bodyNames)

% The same thing for markers as well
bodyNames = {'bodyC8', 'bodyC10'};
markerNames = {'N', 'M';};
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
bodyC10XAdams = readtable("testData\forMainMech\bodyC10CmX.txt");
bodyC10XAdams = table2array(bodyC10XAdams(:, 2));
bodyC10YAdams = readtable("testData\forMainMech\bodyC10CmY.txt");
bodyC10YAdams = table2array(bodyC10YAdams(:, 2));

bodyC10VxAdams = readtable("testData\forMainMech\bodyC10CmVx.txt");
bodyC10VxAdams = table2array(bodyC10VxAdams(:, 2));
bodyC10VyAdams = readtable("testData\forMainMech\bodyC10CmVy.txt");
bodyC10VyAdams = table2array(bodyC10VyAdams(:, 2));
bodyC10dPhiAdams = readtable("testData\forMainMech\bodyC10dPhi.txt");
bodyC10dPhiAdams = 2*pi*table2array(bodyC10dPhiAdams(:, 2))/360;

bodyC10AxAdams = readtable("testData\forMainMech\bodyC10CmAx.txt");
bodyC10AxAdams = table2array(bodyC10AxAdams(:, 2));
bodyC10AyAdams = readtable("testData\forMainMech\bodyC10CmAy.txt");
bodyC10AyAdams = table2array(bodyC10AyAdams(:, 2));
bodyC10ddPhiAdams = readtable("testData\forMainMech\bodyC10ddPhi.txt");
bodyC10ddPhiAdams = 2*pi*table2array(bodyC10ddPhiAdams(:, 2))/360;
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

% Angular position
%{
figure
plot(kinematics.time, qBodyC10Phi, "r", "LineWidth",1.5)
title("Angular position about Z of bodyC10")
ylabel("Angular position [rad]", "FontSize",13, "FontWeight","bold")
xlabel("Time [s]", "FontSize",13, "FontWeight","bold")
grid on
%}

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
bodyC10rmsX = MyRMS(bodyC10X, qBodyC10XAdams');
bodyC10rmsY = MyRMS(bodyC10Y, qBodyC10YAdams');

bodyC10rmsVx = MyRMS(bodyC10VX, qBodyC10VxAdams');
bodyC10rmsVy = MyRMS(bodyC10VY, qBodyC10VyAdams');
bodyC10rmsOmegaZ = MyRMS(bodyC10dPhi, bodyC10dPhiAdams');

bodyC10rmsAx = MyRMS(bodyC10AX, qBodyC10AxAdams');
bodyC10rmsAy = MyRMS(bodyC10AY, qBodyC10AyAdams');
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




%% For the test mechanism
close all; clear; clc;
testMech = testMechanism;

%% Kinematic analysis for text mechanism
testKinematics = Kinematics(testMech, 5, 60);

%% Comparison with results from Adams Multibody system simulation sofware
% Here the comparsion we will be done for bodyC2.

% Kinematics of bodyC2 obtained from the MATLAB implementation
% Possition
bodyC2X = testKinematics.bodyC2.q0(1, :);
bodyC2Y = testKinematics.bodyC2.q0(2, :);

% Velocity
bodyC2Vx = testKinematics.bodyC2.dq(1, :);
bodyC2Vy = testKinematics.bodyC2.dq(2, :);
bodyC2dPhi = testKinematics.bodyC2.dq(3, :);

% Acceleration
bodyC2Ax = testKinematics.bodyC2.ddq(1, :);
bodyC2Ay = testKinematics.bodyC2.ddq(2, :);
bodyC2ddPhi = testKinematics.bodyC2.ddq(3, :);

% Results obtained from Adams
warningState = warning('off', 'MATLAB:table:ModifiedAndSavedVarnames');
bodyC2XAdams = readtable("testData\forTestMech\bodyC2X.txt");
bodyC2XAdams = table2array(bodyC2XAdams(:, 2));
bodyC2YAdams = readtable("testData\forTestMech\bodyC2Y.txt");
bodyC2YAdams = table2array(bodyC2YAdams(:, 2));

bodyC2VxAdams = readtable("testData\forTestMech\bodyC2Vx.txt");
bodyC2VxAdams = table2array(bodyC2VxAdams(:, 2));
bodyC2VyAdams = readtable("testData\forTestMech\bodyC2Vy.txt");
bodyC2VyAdams = table2array(bodyC2VyAdams(:, 2));
bodyC2dPhiAdams = readtable("testData\forTestMech\bodyC2dPhi.txt");
bodyC2dPhiAdams = 2*pi*table2array(bodyC2dPhiAdams(:, 2))/360;

bodyC2AxAdams = readtable("testData\forTestMech\bodyC2Ax.txt");
bodyC2AxAdams = table2array(bodyC2AxAdams(:, 2));
bodyC2AyAdams = readtable("testData\forTestMech\bodyC2Ay.txt");
bodyC2AyAdams = table2array(bodyC2AyAdams(:, 2));
bodyC2ddPhiAdams = readtable("testData\forTestMech\bodyC2ddPhi.txt");
bodyC2ddPhiAdams = 2*pi*table2array(bodyC2ddPhiAdams(:, 2))/360;
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

figure
plot(testKinematics.time, bodyC2Y, "k", "LineWidth",1.5, "DisplayName", "MATLAB")
hold on
plot(testKinematics.time, bodyC2YAdams, "--r", "LineWidth",1.5, "DisplayName", "Imported from Adams")
hold off
title("Position along Y of body 2")
ylabel("Postion in Y [m]", "FontSize",13, "FontWeight","bold")
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

figure
plot(testKinematics.time, bodyC2Vy, "k", "LineWidth",1.5, "DisplayName", "MATLAB")
hold on
plot(testKinematics.time, bodyC2VyAdams, "--r", "LineWidth",1.5, "DisplayName", "Imported from Adams")
hold off
title("Velocity along Y of body 2")
ylabel("Velocity in Y [m/s]", "FontSize",13, "FontWeight","bold")
xlabel("Time [s]", "FontSize",13, "FontWeight","bold")
legend("Location", "best", "FontSize", 13, "FontWeight", "bold")
grid on
% About Z
figure
plot(testKinematics.time, bodyC2dPhi, "k", "LineWidth", 2, "DisplayName", "MATLAB")
hold on
plot(testKinematics.time, bodyC2dPhiAdams, "--r", "LineWidth", 2, "DisplayName", "Imported from Adams")
hold off
title("Angular velocity about Z of bodyC2")
ylabel("Angular velocity [rad/s]", "FontSize",13, "FontWeight","bold")
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

figure
plot(testKinematics.time, bodyC2Ay, "k", "LineWidth",1.5, "DisplayName", "MATLAB")
hold on
plot(testKinematics.time, bodyC2AyAdams, "--r", "LineWidth",1.5, "DisplayName", "Imported from Adams")
hold off
title("Acceleration along Y of body 2")
ylabel("Acceleration in Y [m/s^2]", "FontSize",13, "FontWeight","bold")
xlabel("Time [s]", "FontSize",13, "FontWeight","bold")
legend("Location", "best", "FontSize", 13, "FontWeight", "bold")
grid on
% About Z
figure
plot(testKinematics.time, bodyC2ddPhi, "k", "LineWidth", 2, "DisplayName", "MATLAB")
hold on
plot(testKinematics.time, bodyC2ddPhiAdams, "--r", "LineWidth", 2, "DisplayName", "Imported from Adams")
hold off
title("Angular acceleration about Z of bodyC2")
ylabel("Angular acceleration [rad/s^2]", "FontSize",13, "FontWeight","bold")
xlabel("Time [s]", "FontSize",13, "FontWeight","bold")
legend("Location", "best", "FontSize", 13, "FontWeight", "bold")
grid on


%% Performing RMS comparison with results obtained form Adams
bodyC2rmsX = MyRMS(bodyC2X, bodyC2XAdams');
bodyC2rmsY = MyRMS(bodyC2Y, bodyC2YAdams');

bodyC2rmsVx = MyRMS(bodyC2Vx, bodyC2VxAdams');
bodyC2rmsVy = MyRMS(bodyC2Vy, bodyC2VyAdams');
bodyC2rmsOmegaZ = MyRMS(bodyC2dPhi, bodyC2dPhiAdams');

bodyC2rmsAx = MyRMS(bodyC2Ax,bodyC2AxAdams');
bodyC2rmsAy = MyRMS(bodyC2Ay, bodyC2AyAdams');
bodyC2rmsAlphaZ = MyRMS(bodyC2ddPhi, bodyC2ddPhiAdams');

disp("#############   Error analysis for test mechanism bodyC2 ###############")
fprintf("RMSE in position X: %e\n", bodyC2rmsX)
fprintf("RMSE in position Y: %e\n\n", bodyC2rmsY)

fprintf("RMSE in velocity VX: %e\n", bodyC2rmsVx)
fprintf("RMSE in velocity VY: %e\n", bodyC2rmsVy)
fprintf("RMSE in angular velocity about Z : %e\n\n", bodyC2rmsOmegaZ)

fprintf("RMSE in acceleration AX: %e\n", bodyC2rmsAx)
fprintf("RMSE in acceleration AY: %e\n", bodyC2rmsAy)
fprintf("RMSE in angular acceleration about Z: %e\n", bodyC2rmsAlphaZ)
disp("#############   Error analysis for bodyC10 END  ###############")
