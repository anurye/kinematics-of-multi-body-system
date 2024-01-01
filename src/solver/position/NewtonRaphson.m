function q = NewtonRaphson(mechanism, t, varargin)
% NewtonRaphson.m: NewtonRaphson algorithm routine for solving the position
% problem.
%
%   Parameters:
%       mechanism: a struct representing the mechanism. this is from
%       where we extract the initial coordinate values, q0.
%
%       t: time instant at which the position is going to be solved
%       varargin: Optional argument to specify the number of iteration and
%       accuracy.
%
%       varargin: optional argument to specify tolerance and maximum number
%       of iteration.
%                - 1st optional argument should be maxIteration
%                - 2nd optional argument should be tolerance
%
%   Returns:
%         q: coordinate vector at time t
%

% Parse varargin
if nargin > 4
    error("NewtonRaphson: Yor are allowed to specify a maximum of two " + ...
        "optional arguments(maxIter and epsi): %s", varargin)
end
if nargin == 3
    maxIter = varargin{1};
    epsi = Tolerance;
elseif nargin == 4
    maxIter = varargin{1};
    epsi = varargin{2};
else
    maxIter = 25;
    epsi = Tolerance;
end

% Extract current postion, q0, from the mechanism
[q0, ~, ~] = getParameters(mechanism);
% Initialize the solution with the initial value
q = q0;
% Get the constraint vector
F = Constraints(mechanism, t);
% Initialize counter for iteration
iter = 1;
while ((norm(F) > epsi) && (iter < maxIter) )
    F = Constraints(mechanism, t);
    Fq = MyJacobian(mechanism);
    q = q - Fq\F; %inv(Fq)*F for efficiency Fq\F
    mechanism = updateMechanism(mechanism, q);
    iter = iter + 1;
end

if iter >= maxIter
    warning('NewtonRaphson: algorithm did not converge. Returning q0')
    q=q0;
end

end
