function Ft = RightHandSide(mechanism, ti)
% RightHandSide.m: Computer the right hand side equation of the velocity problem.
%
%   Parameter:
%          mechanism: a struct representing the mechanism.
%          ti: time instant at which the right hand side equation will be evaluated.
%
%   Returns:
%          Ft: Right hand side equation of the velocity problem
%

% Define a symbolic variable, t, that will be used for differentiation
syms t

% determine the degree of freedom, this is the same as the number of
% driving constraints => number of rows of Ft which are non-zero.
commonJoints = findCommonJoints(mechanism);    % Get common joints between bodies
dof = degreeOfFreedom(mechanism, commonJoints);
% Initialize the number of driving constraints
count_drive = dof;
% Get the list of joints
jointNames = fieldnames(commonJoints);

% Initialize the size of Ft
Ft = zeros(3*(numel(fieldnames(mechanism)) - 1), 1);
% Fill elements of Ft iteratively
for i = 1:numel(jointNames)
    jointName = jointNames{i};
    isDriving = commonJoints.(jointName).driving;
    if isDriving
        if count_drive < 0
            error("RightHandSide.m: You can't specify more driving constraints than the DoF: %d", dof)
        end
        fAB = commonJoints.(jointName).fAB;
        % Differentiate fAB with respect to time
        fABt = diff(fAB, t);
        % Evaluate fABt at time instant ti
        fABt_ti = subs(fABt, ti);
        Ft(end-(count_drive - 1), 1) = -fABt_ti;
        count_drive = count_drive - 1;
    end
end

end