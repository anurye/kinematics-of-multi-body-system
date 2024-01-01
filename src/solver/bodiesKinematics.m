function [finalMechanism, time] = bodiesKinematics(mechanism, varargin)
% BodiesKinematics.m: Perfoms the overall kinematic analysis of the
% mechanism bodies (i.e., at CoM).
%
%  Parameter:
%            mechanism: a struct representing the mechanism and everything
%            it contains.
%
%            varargin: optional argument for specifying end time and
%            number of steps.
%                   - 1st argument should be endTime
%                   - 2nd argument should be steps
%
%  Returns:
%         finalMechanism: the mechanism with its calculated kinematic
%         parameters and time instants at which the parameters are
%         obtained.
%
%         time: list of time instants at which the kinematics problem is
%         solved.
%


% Check if the right amout of optional arguments is provided
if nargin > 3
    error("BodiesKinematics: more optional arguments than expected: %s", varargin);
end

% Parse varargin
if nargin == 3
    endTime = varargin{1};
    steps = varargin{2};
elseif nargin == 2
    endTime = varargin{1};
    steps = 100;
else
    endTime = 5;
    steps = 100;
end

% Define simulation start time, end time, and incrment
t0 = 0;
tk = endTime;
h = (tk - t0)/steps;

% Initialize size of solution matrix based on the number of bodies
numBodies = numel(fieldnames(mechanism)) - 1; % Ground doesn't count
numRows = 3*numBodies;
% The number of columns will be the length of the time vector.
numColumns = floor((tk - t0) / h) + 1;

% Initialization
T = zeros(1, numColumns);
Q = zeros(numRows, numColumns);
dQ = zeros(numRows, numColumns);
ddQ = zeros(numRows, numColumns);

% Initialize counter that will be used to determine indeces when updating
% parameters.
counter = 1;

% Get initial value of the coordinate vectors, velocity and acceleration.
% Velocity and acceleration are initially zero
[q, dq, ddq] = getParameters(mechanism);

% Iteratively solve the kinematic problem
for t = t0:h:tk
    % Perform initial approximate estimation for the postion problem
    q0 = q + dq*h + (1/2)*ddq*h^2;
    % update mechanism with the new position approximation
    mechanism = updateMechanism(mechanism, q, dq, ddq);

    % Solve the kinematic problem for the current time step
    q = NewtonRaphson(mechanism, t, 25);
    % update mechanism with the new position solution
    mechanism = updateMechanism(mechanism, q, dq, ddq);

    dq = Velocity(mechanism, t);
    % update mechanism with the new position and velocity solutions
    mechanism = updateMechanism(mechanism, q, dq, ddq);

    ddq = Acceleration(mechanism, t);
    % update mechanism with the new postion, velocity and acceleration
    % solutions
    mechanism = updateMechanism(mechanism, q0, dq, ddq);

    % Add current solution to the list
    Q(:, counter) = q;
    dQ(:, counter) = dq;
    ddQ(:, counter) = ddq;
    T(1, counter) = t;

    % Update counter
    counter = counter + 1;
end

% Associate the final results to the mechanism
updatedMechanism = updateMechanism(mechanism, Q, dQ, ddQ);

% Return output variables based on nargout the user required
if nargout == 2
    finalMechanism = updatedMechanism;
    time = T;
elseif nargout == 1
    finalMechanism = updatedMechanism;
    % Add the simulation time instants to the finalMechanism
    finalMechanism.time = T;
end

end