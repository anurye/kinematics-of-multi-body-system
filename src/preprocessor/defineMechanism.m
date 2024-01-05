function mechanism = defineMechanism
% define_mechanism.m: defines the structure of the mechanism
%
% Returns:
%         mechanism: a struct that contains the description of the mechanisms
%         bodies. This includes, joints, center of mass locations, initial
%         coordinates, joint locations, joint type, and driving
%         constraint motion definitions.
%

% Initialize a struct to represent the mechanism
mechanism = struct();
% Ground: center of mass and initial values of the kinematic variables
% These are fixed since the ground doesn't move.
mechanism.ground.com = [0, 0]';
mechanism.ground.q0 = [0, 0, 0]';
mechanism.ground.dq = [0, 0, 0]';
mechanism.ground.ddq = [0, 0, 0]';

% Joints associated with the ground
mechanism.ground.joints.joint_A.type = 'R';
mechanism.ground.joints.joint_A.location = [0, 0]';
mechanism.ground.joints.joint_A.driving = false;

mechanism.ground.joints.joint_B.type = 'R';
mechanism.ground.joints.joint_B.location = [0.3, 0]';
mechanism.ground.joints.joint_B.driving = false;

mechanism.ground.joints.joint_E.type = 'R';
mechanism.ground.joints.joint_E.location = [-0.1, 0.7]';
mechanism.ground.joints.joint_E.driving = false;


% Plate C1: center of mass and initial values
mechanism.bodyC1.com = [0, 0.85]';
mechanism.bodyC1.q0 = [0, 0.85, 0]';
mechanism.bodyC1.dq = [0, 0, 0]';
mechanism.bodyC1.ddq = [0, 0, 0]';

% Joints associated with plate C1
mechanism.bodyC1.joints.joint_D.type = 'R';
mechanism.bodyC1.joints.joint_D.location = [0.2, 0.6]';
mechanism.bodyC1.joints.joint_D.driving = false;

mechanism.bodyC1.joints.joint_E.type = 'R';
mechanism.bodyC1.joints.joint_E.location = [-0.1, 0.7]';
mechanism.bodyC1.joints.joint_E.driving = false;

mechanism.bodyC1.joints.joint_F.type = 'R';
mechanism.bodyC1.joints.joint_F.location = [-0.2, 0.8]';
mechanism.bodyC1.joints.joint_F.driving = false;

mechanism.bodyC1.joints.joint_G.type = 'R';
mechanism.bodyC1.joints.joint_G.location = [0, 1.2]';
mechanism.bodyC1.joints.joint_G.driving = false;

% Markers associated with bodyC1
mechanism.bodyC1.markers.D.location = [0.2, 0.6]';
mechanism.bodyC1.markers.E.location = [-0.1, 0.7]';
mechanism.bodyC1.markers.G.location = [0, 1.2]';


% Plate C2: center of mass and initial values
mechanism.bodyC2.com = [0.4, 0.75]';
mechanism.bodyC2.q0 = [0.4, 0.75, 0]';
mechanism.bodyC2.dq = [0, 0, 0]';
mechanism.bodyC2.ddq = [0, 0, 0]';

% Joints associated with plate C2
mechanism.bodyC2.joints.joint_G.type = 'R';
mechanism.bodyC2.joints.joint_G.location = [0, 1.2]';
mechanism.bodyC2.joints.joint_G.driving = false;

mechanism.bodyC2.joints.joint_H.type = 'R';
mechanism.bodyC2.joints.joint_H.location = [0.5, 0.8]';
mechanism.bodyC2.joints.joint_H.driving = false;

mechanism.bodyC2.joints.joint_J.type = 'R';
mechanism.bodyC2.joints.joint_J.location = [0.7, 0.3]';
mechanism.bodyC2.joints.joint_J.driving = false;

% Markers associated with bodyC2
mechanism.bodyC2.markers.G.location = [0, 1.2]';
mechanism.bodyC2.markers.H.location = [0.5, 0.8]';
mechanism.bodyC2.markers.J.location = [0.7, 0.3]';


% Plate C3: center of mass and initial values
mechanism.bodyC3.com = [0.15, 0.45]';
mechanism.bodyC3.q0 = [0.15, 0.45, 0]';
mechanism.bodyC3.dq = [0, 0, 0]';
mechanism.bodyC3.ddq = [0, 0, 0]';

% Joints associated with plated C3
mechanism.bodyC3.joints.joint_D.type = 'R';
mechanism.bodyC3.joints.joint_D.location = [0.2, 0.6]';
mechanism.bodyC3.joints.joint_D.driving = false;

mechanism.bodyC3.joints.joint_AD.type = 'P';
mechanism.bodyC3.joints.joint_AD.location = [NaN, NaN]';
mechanism.bodyC3.joints.joint_AD.driving = true;
mechanism.bodyC3.joints.joint_AD.fAB = @(t) -0.1*sin(1.5*t+0);  % Specify motion

% Markers associated with bodyC3
mechanism.bodyC3.markers.D.location = [0.2, 0.6]';


% Plate C4: center of mass and initial values
mechanism.bodyC4.com = [0.05, 0.15]';
mechanism.bodyC4.q0 = [0.05, 0.15, 0]';
mechanism.bodyC4.dq = [0, 0, 0]';
mechanism.bodyC4.ddq = [0, 0, 0]';

% Joints associated with plate C3
mechanism.bodyC4.joints.joint_A.type = 'R';
mechanism.bodyC4.joints.joint_A.location = [0, 0]';
mechanism.bodyC4.joints.joint_A.driving = false;

mechanism.bodyC4.joints.joint_AD.type = 'P';
mechanism.bodyC4.joints.joint_AD.location = [NaN, NaN]';
mechanism.bodyC4.joints.joint_AD.driving = true;
mechanism.bodyC4.joints.joint_AD.fAB = @(t) -0.1*sin(1.5*t+0);  % Specify motion

% Markers associated with bodyC4
mechanism.bodyC4.markers.A.location = [0, 0]';


% Plate C5: center of mass and initial values
mechanism.bodyC5.com = [0.45, 0.6]';
mechanism.bodyC5.q0 = [0.45, 0.6, 0]';
mechanism.bodyC5.dq = [0, 0, 0]';
mechanism.bodyC5.ddq = [0, 0, 0]';

mechanism.bodyC5.joints.joint_H.type = 'R';
mechanism.bodyC5.joints.joint_H.location = [0.5, 0.8]';
mechanism.bodyC5.joints.joint_H.driving = false;

mechanism.bodyC5.joints.joint_BH.type = 'P';
mechanism.bodyC5.joints.joint_BH.location = [NaN, NaN]';
mechanism.bodyC5.joints.joint_BH.driving = true;
mechanism.bodyC5.joints.joint_BH.fAB = @(t) -0.1*sin(1.5*t+0);  % Specify motion

% Markers associated with bodyC5
mechanism.bodyC5.markers.H.location = [0.5, 0.8]';


% Plate C6: center of mass and initial values
mechanism.bodyC6.com = [0.35, 0.2]';
mechanism.bodyC6.q0 = [0.35, 0.2, 0]';
mechanism.bodyC6.dq = [0, 0, 0]';
mechanism.bodyC6.ddq = [0, 0, 0]';

% Joints associated with plate C6
mechanism.bodyC6.joints.joint_B.type = 'R';
mechanism.bodyC6.joints.joint_B.location = [0.3, 0]';
mechanism.bodyC6.joints.joint_B.driving = false;

mechanism.bodyC6.joints.joint_BH.type = 'P';
mechanism.bodyC6.joints.joint_BH.location = [NaN, NaN]';
mechanism.bodyC6.joints.joint_BH.driving = true;
mechanism.bodyC6.joints.joint_BH.fAB = @(t) -0.1*sin(1.5*t+0);  % Specify motion

% Markers associated with bodyC6
mechanism.bodyC6.markers.B.location = [0.3, 0]';


% Plate C7: center of mass and initial values
mechanism.bodyC7.com = [0.4, 0.1]';
mechanism.bodyC7.q0 = [0.4, 0.1, 0]';
mechanism.bodyC7.dq = [0, 0, 0]';
mechanism.bodyC7.ddq = [0, 0, 0]';

% Joints associated with plate C7
mechanism.bodyC7.joints.joint_F.type = 'R';
mechanism.bodyC7.joints.joint_F.location = [-0.2, 0.8]';
mechanism.bodyC7.joints.joint_F.driving = false;

mechanism.bodyC7.joints.joint_I.type = 'R';
mechanism.bodyC7.joints.joint_I.location = [0.5, -0.1]';
mechanism.bodyC7.joints.joint_I.driving  = false;

mechanism.bodyC7.joints.joint_L.type = 'R';
mechanism.bodyC7.joints.joint_L.location = [1, -0.5]';
mechanism.bodyC7.joints.joint_L.driving = false;

% Markers associated with bodyC7
mechanism.bodyC7.markers.F.location = [-0.2, 0.8]';
mechanism.bodyC7.markers.I.location = [0.5, -0.1]';
mechanism.bodyC7.markers.L.location = [1, -0.5]';


% Plate C8: center of mass and initial values
mechanism.bodyC8.com = [0.65, 0.25]';
mechanism.bodyC8.q0 = [0.65, 0.25, 0]';
mechanism.bodyC8.dq = [0, 0, 0]';
mechanism.bodyC8.ddq = [0, 0, 0]';

% Joints associated with plate C8
mechanism.bodyC8.joints.joint_I.type = 'R';
mechanism.bodyC8.joints.joint_I.location = [0.5, -0.1]';
mechanism.bodyC8.joints.joint_I.driving = false;

mechanism.bodyC8.joints.joint_J.type = 'R';
mechanism.bodyC8.joints.joint_J.location = [0.7, 0.3]';
mechanism.bodyC8.joints.joint_J.driving = false;

mechanism.bodyC8.joints.joint_K.type = 'R';
mechanism.bodyC8.joints.joint_K.location = [0.8, 0.6]';
mechanism.bodyC8.joints.joint_K.driving = false;

% Markers associated with bodyC8
mechanism.bodyC8.markers.I.location = [0.5, -0.1]';
mechanism.bodyC8.markers.J.location = [0.7, 0.3]';
mechanism.bodyC8.markers.K.location = [0.8, 0.6]';


% Plate C9: center of mass and initial values
mechanism.bodyC9.com = [1.05, 0.3]';
mechanism.bodyC9.q0 = [1.05, 0.3, 0]';
mechanism.bodyC9.dq = [0, 0, 0]';
mechanism.bodyC9.ddq = [0, 0, 0]';

% Joints associated with plate C9
mechanism.bodyC9.joints.joint_K.type = 'R';
mechanism.bodyC9.joints.joint_K.location = [0.8, 0.6]';
mechanism.bodyC9.joints.joint_K.driving = false;

mechanism.bodyC9.joints.joint_N.type = 'R';
mechanism.bodyC9.joints.joint_N.location = [1.3, 0]';
mechanism.bodyC9.joints.joint_N.driving = false;

% Markers associated with bodyC9
mechanism.bodyC9.markers.K.location = [0.8, 0.6]';
mechanism.bodyC9.markers.N.location = [1.3, 0]';


% Plate C10: center of mass and initial values
mechanism.bodyC10.com = [1.2, -0.25]';
mechanism.bodyC10.q0 = [1.2, -0.25, 0]';
mechanism.bodyC10.dq = [0, 0, 0]';
mechanism.bodyC10.ddq = [0, 0, 0]';

% Joints associated with plate C10
mechanism.bodyC10.joints.joint_L.type = 'R';
mechanism.bodyC10.joints.joint_L.location = [1, -0.5]';
mechanism.bodyC10.joints.joint_L.driving = false;

mechanism.bodyC10.joints.joint_M.type = 'R';
mechanism.bodyC10.joints.joint_M.location = [1.3, -0.3]';
mechanism.bodyC10.joints.joint_M.driving = false;

mechanism.bodyC10.joints.joint_N.type = 'R';
mechanism.bodyC10.joints.joint_N.location = [1.3, 0]';
mechanism.bodyC10.joints.joint_N.driving = false;

% Markers associated with bodyC10
mechanism.bodyC10.markers.L.location = [1, -0.5]';
mechanism.bodyC10.markers.M.location = [1.3, -0.3]';
mechanism.bodyC10.markers.N.location = [1.3, 0]';

end
