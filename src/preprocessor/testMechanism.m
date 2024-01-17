function mechanism = testMechanism
% testMechanism.m: define a test mechanism for evaluating the performance
% of the system (whether it is applicable for general case planar
% mechanism or not)
%
%   Parameter:
%            None
%
%   Returns:
%          mechanism: a struct representing the defined mechanism
%

mechanism = struct();
% Ground: center of mass and initial values of the kinematic variables
% These are fixed since the ground doesn't move.
mechanism.ground.com = [0, 0]';
mechanism.ground.q0 = [0, 0, 0]';
mechanism.ground.dq = [0, 0, 0]';
mechanism.ground.ddq = [0, 0, 0]';

% Joints associated with the ground
mechanism.ground.joints.jointA.type = 'R';
mechanism.ground.joints.jointA.location = [0, 0]';
mechanism.ground.joints.jointA.driving = true;
mechanism.ground.joints.jointA.fAB = @(t) t^2 + pi/2;

mechanism.ground.joints.jointD.type = 'P';
mechanism.ground.joints.jointD.location = [NaN, NaN]';
mechanism.ground.joints.jointD.driving = false;
% Put a reference for defining the translational axis
mechanism.ground.joints.jointD.reference = [5, 4]';


% body C1: center of mass and initial values
mechanism.bodyC1.com = [0, 0]';
mechanism.bodyC1.q0 = [0, 0, 0]';
mechanism.bodyC1.dq = [0, 0, 0]';
mechanism.bodyC1.ddq = [0, 0, 0]';

% Joints associated with bodyC1
mechanism.bodyC1.joints.jointA.type = 'R';
mechanism.bodyC1.joints.jointA.location = [0, 0]';
mechanism.bodyC1.joints.jointA.driving = true;
mechanism.bodyC1.joints.jointA.fAB = @(t) t^2 + pi/2;

mechanism.bodyC1.joints.jointB.type = 'R';
mechanism.bodyC1.joints.jointB.location = [0, 1]';
mechanism.bodyC1.joints.jointB.driving = false;

% body C2: center of mass and initial values
mechanism.bodyC2.com = [0, 1]';
mechanism.bodyC2.q0 = [0, 1, 0]';
mechanism.bodyC2.dq = [0, 0, 0]';
mechanism.bodyC2.ddq = [0, 0, 0]';

% Joints associated with bodyC2
mechanism.bodyC2.joints.jointB.type = 'R';
mechanism.bodyC2.joints.jointB.location = [0, 1]';
mechanism.bodyC2.joints.jointB.driving = false;

mechanism.bodyC2.joints.jointC.type = 'R';
mechanism.bodyC2.joints.jointC.location = [4, 4]';
mechanism.bodyC2.joints.jointC.driving = false;


% body C3: center of mass and initial values
mechanism.bodyC3.com = [4, 4]';
mechanism.bodyC3.q0 = [4, 4, 0]';
mechanism.bodyC3.dq = [0, 0, 0]';
mechanism.bodyC3.ddq = [0, 0, 0]';

% Joints associated with bodyC3
mechanism.bodyC3.joints.jointC.type = 'R';
mechanism.bodyC3.joints.jointC.location = [4, 4]';
mechanism.bodyC3.joints.jointC.driving = false;

mechanism.bodyC3.joints.jointD.type = 'P';
mechanism.bodyC3.joints.jointD.location = [NaN, NaN]';
mechanism.bodyC3.joints.jointD.driving = false;
mechanism.bodyC3.joints.jointD.reference = [5, 4]';


end