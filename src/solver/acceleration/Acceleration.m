function ddq = Acceleration(mechanism, t)
% Acceleration.m: Computes the accelerations of the mechanism bodies at time t
%
% Parameters:
%       mechanism: a struct representing the mechanism. this is from
%       where we extract the coordinate vector values, q, and velocities
%       dq.
%       t: time instant at which the acceleration problem is going to be
%       solved
%
% Returns:
%         ddq: acceleration of bodies at time t.
%

% Get the jacobian matrix
Fq = MyJacobian(mechanism);
% Get gamma vector
gamma = Gamma(mechanism, t);

% Solve for the acceleration problem
ddq = Fq\gamma;

end
