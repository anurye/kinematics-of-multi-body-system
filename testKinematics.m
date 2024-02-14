function tests = testKinematics
tests = functiontests(localfunctions);
end

% File fixture
function setupOnce(testCase)
% Initialize the system by adding all files to the MATLAB PATH variable
Initialize;
% Get the mechanism for which kinematics analysis is to be performed
mechanism = defineMechanism;
% solve the kinematic problem for the mechanism bodies and their marker
% points.
endTime = 5;
steps = 100;
testCase.TestData.kinematics = Kinematics(mechanism, endTime, steps);
% Define tolerance
testCase.TestData.tol = 1e-6;
end


% Test functions
function testBodyKinematics(testCase)
% Read the actual and expected results
% Kinematics of bodyC10 obtained from the MATLAB implementation - actual

% Position
bodyC10X = testCase.TestData.kinematics.bodyC10.q0(1, :);
bodyC10Y = testCase.TestData.kinematics.bodyC10.q0(2, :);
% Velocity
bodyC10VX = testCase.TestData.kinematics.bodyC10.dq(1, :);
bodyC10VY = testCase.TestData.kinematics.bodyC10.dq(2, :);
bodyC10dPhi = testCase.TestData.kinematics.bodyC10.dq(3, :);
% Acceleration
bodyC10AX = testCase.TestData.kinematics.bodyC10.ddq(1, :);
bodyC10AY = testCase.TestData.kinematics.bodyC10.ddq(2, :);
bodyC10ddPhi = testCase.TestData.kinematics.bodyC10.ddq(3, :);

% Kinematics of bodyC10 obtained from Adams - expected
warningState = warning('off', 'MATLAB:table:ModifiedAndSavedVarnames');
bodyC10XAdams = readtable("testData/forMainMech/bodyC10CmX.txt");
bodyC10XAdams = table2array(bodyC10XAdams(:, 2))';
bodyC10YAdams = readtable("testData/forMainMech/bodyC10CmY.txt");
bodyC10YAdams = table2array(bodyC10YAdams(:, 2))';

bodyC10VxAdams = readtable("testData/forMainMech/bodyC10CmVx.txt");
bodyC10VxAdams = table2array(bodyC10VxAdams(:, 2))';
bodyC10VyAdams = readtable("testData/forMainMech/bodyC10CmVy.txt");
bodyC10VyAdams = table2array(bodyC10VyAdams(:, 2))';
bodyC10dPhiAdams = readtable("testData/forMainMech/bodyC10dPhi.txt");
bodyC10dPhiAdams = table2array(bodyC10dPhiAdams(:, 2))';

bodyC10AxAdams = readtable("testData/forMainMech/bodyC10CmAx.txt");
bodyC10AxAdams = table2array(bodyC10AxAdams(:, 2))';
bodyC10AyAdams = readtable("testData/forMainMech/bodyC10CmAy.txt");
bodyC10AyAdams = table2array(bodyC10AyAdams(:, 2))';
bodyC10ddPhiAdams = readtable("testData/forMainMech/bodyC10ddPhi.txt");
bodyC10ddPhiAdams = table2array(bodyC10ddPhiAdams(:, 2))';
warning(warningState);

% Test position
verifyLessThanOrEqual(testCase, MyRMS(bodyC10X, bodyC10XAdams), testCase.TestData.tol, ...
    'bodyC10 - X position faild')
verifyLessThanOrEqual(testCase, MyRMS(bodyC10Y, bodyC10YAdams), testCase.TestData.tol,...
    'bodyC10 - Y position faild')
% Test velocity
verifyLessThanOrEqual(testCase, MyRMS(bodyC10VX, bodyC10VxAdams), testCase.TestData.tol , ...
    'bodyC10 - X velocity faild')
verifyLessThanOrEqual(testCase, MyRMS(bodyC10VY, bodyC10VyAdams), testCase.TestData.tol, ...
    'bodyC10 - Y velocity faild')
verifyLessThanOrEqual(testCase, MyRMS(bodyC10dPhi, bodyC10dPhiAdams), testCase.TestData.tol, ...
    'bodyC10 - about Z velocity faild')
% Test acceleration
verifyLessThanOrEqual(testCase, MyRMS(bodyC10AX, bodyC10AxAdams), testCase.TestData.tol , ...
    'bodyC10 - X acceleration faild')
verifyLessThanOrEqual(testCase, MyRMS(bodyC10AY, bodyC10AyAdams), testCase.TestData.tol, ...
    'bodyC10 - Y acceleration faild')
verifyLessThanOrEqual(testCase, MyRMS(bodyC10ddPhi, bodyC10ddPhiAdams), testCase.TestData.tol, ...
    'bodyC10 - about Z acceleration faild')
end


function testMarkerKinematics(testCase)
% Read the actual and expected results
% Kinematics of bodyC10 obtained from the MATLAB implementation - actual

% Position
bodyC8MarkerKX = testCase.TestData.kinematics.bodyC8.markers.K.q(1, :);
bodyC8MarkerKY = testCase.TestData.kinematics.bodyC8.markers.K.q(2, :);

% Velocity
bodyC8MarkerKVx = testCase.TestData.kinematics.bodyC8.markers.K.dq(1, :);
bodyC8MarkerKVy = testCase.TestData.kinematics.bodyC8.markers.K.dq(2, :);
bodyC8MarkerKdPhi = testCase.TestData.kinematics.bodyC8.markers.K.dq(3, :);

% Acceleration
bodyC8MarkerKAx = testCase.TestData.kinematics.bodyC8.markers.K.ddq(1, :);
bodyC8MarkerKAy = testCase.TestData.kinematics.bodyC8.markers.K.ddq(2, :);
bodyC8MarkerKddPhi = testCase.TestData.kinematics.bodyC8.markers.K.ddq(3, :);

% Kinematics of marker K obtained from Adams
warningState = warning('off', 'MATLAB:table:ModifiedAndSavedVarnames');
bodyC8MarkerKXAdams = readtable("testData/forMainMech/bodyC8MarkerKX.txt");
bodyC8MarkerKXAdams = table2array(bodyC8MarkerKXAdams(:, 2))';
bodyC8MarkerKYAdams = readtable("testData/forMainMech/bodyC8MarkerKY.txt");
bodyC8MarkerKYAdams = table2array(bodyC8MarkerKYAdams(:, 2))';

bodyC8MarkerKVxAdams = readtable("testData/forMainMech/bodyC8MarkerKVx.txt");
bodyC8MarkerKVxAdams = table2array(bodyC8MarkerKVxAdams(:, 2))';
bodyC8MarkerKVyAdams = readtable("testData/forMainMech/bodyC8MarkerKVy.txt");
bodyC8MarkerKVyAdams = table2array(bodyC8MarkerKVyAdams(:, 2))';
bodyC8MarkerKdPhiAdams = readtable("testData/forMainMech/bodyC8MarkerKdPhi.txt");
bodyC8MarkerKdPhiAdams = table2array(bodyC8MarkerKdPhiAdams(:, 2))';

bodyC8MarkerKAxAdams = readtable("testData/forMainMech/bodyC8MarkerKAx.txt");
bodyC8MarkerKAxAdams = table2array(bodyC8MarkerKAxAdams(:, 2))';
bodyC8MarkerKAyAdams = readtable("testData/forMainMech/bodyC8MarkerKAy.txt");
bodyC8MarkerKAyAdams = table2array(bodyC8MarkerKAyAdams(:, 2))';
bodyC8MarkerKddPhiAdams = readtable("testData/forMainMech/bodyC8MarkerKddPhi.txt");
bodyC8MarkerKddPhiAdams = table2array(bodyC8MarkerKddPhiAdams(:, 2))';
warning(warningState);
% Test position
verifyLessThanOrEqual(testCase, MyRMS(bodyC8MarkerKX, bodyC8MarkerKXAdams), ...
    testCase.TestData.tol, 'bodyC8 marker K - X position faild')
verifyLessThanOrEqual(testCase, MyRMS(bodyC8MarkerKY, bodyC8MarkerKYAdams), ...
    testCase.TestData.tol, 'bodyC8 marker K - Y position faild')
% Test velocity
verifyLessThanOrEqual(testCase, MyRMS(bodyC8MarkerKVx, bodyC8MarkerKVxAdams), ...
    testCase.TestData.tol, 'bodyC8 marker K - X velocity faild')
verifyLessThanOrEqual(testCase, MyRMS(bodyC8MarkerKVy, bodyC8MarkerKVyAdams), ...
    testCase.TestData.tol, 'bodyC8 marker K - Y velocity faild')
verifyLessThanOrEqual(testCase, MyRMS(bodyC8MarkerKdPhi, bodyC8MarkerKdPhiAdams),...
    testCase.TestData.tol, 'bodyC8 marker K - about Z velocity faild')
% Test acceleration
verifyLessThanOrEqual(testCase, MyRMS(bodyC8MarkerKAx, bodyC8MarkerKAxAdams), ...
    testCase.TestData.tol, 'bodyC8 marker K - X acceleration faild')
verifyLessThanOrEqual(testCase, MyRMS(bodyC8MarkerKAy, bodyC8MarkerKAyAdams), ...
    testCase.TestData.tol, 'bodyC8 marker K - Y acceleration faild')
verifyLessThanOrEqual(testCase, MyRMS(bodyC8MarkerKddPhi, bodyC8MarkerKddPhiAdams), ...
    testCase.TestData.tol, 'bodyC10 marker K - about Z acceleration faild')
end
