function dof = degreeOfFreedom(mechanism, commonJoints)
% degreeOfFreedom: calculates the degree of freedom of the mechanism.
%
% Parameters:
%            mechanism: struct containing all bodies in the mechanism
%            commonJoints: list of joints that connects any two bodies
% Returns: dof - theoretical degree of freedom of the mechanism
%

% Initialize the number of kinematic class
p4 = 0; p5 = 0;
% Get the list of joints
jointNames = fieldnames(commonJoints);

% Update p4 and P5 iteratively
for i = 1:numel(jointNames)
    jointName = jointNames{i};
    type = commonJoints.(jointName).type;
    if strcmp(type, 'R') || strcmp(type, 'P')
        p5 = p5 + 1;
    elseif strcmp(type, 'RP') || strcmp(type, 'PR')
        p4 = p4 + 1;
    else
        error("The joint type: %s is not valied for a plannar mechanism", type)
    end
end

% number of bodies excluding the ground
n = numel(fieldnames(mechanism)) - 1;

dof = 3*n - 2*p5 - p4;

end
