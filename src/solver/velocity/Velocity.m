function dq = Velocity(mechanism, t)
% Velocity.m: Computes the velocities of the mechanism bodies at time t
%
% Parameters:
%       mechanism: a struct representing the mechanism. this is from
%       where we extract the coordinate vector values, q.
%       t: time instant at which the velocity is going to be solved
%
% Returns:
%         dq: velocity of bodies at time t.
%

% Get the jacobian matrix
Fq = MyJacobian(mechanism);
% Check if singular confuguration is encountered
if abs(det(Fq)) <= Zero
    error("Velocity: Singularity encountered. det(Fq) = %d\n", det(Fq))
end
% Get the right hand side of the velocity equation
Ft = RightHandSide(mechanism, t);

% Solve for the velocity problem
dq = -Fq\Ft;

end
