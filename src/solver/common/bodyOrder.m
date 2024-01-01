function order = bodyOrder(mechanism)
% bodyOrder.m: computes the order of bodies in the mechanism. This is
% important to associate each body with its respective position, velocity
% and acceleration.
%
%   Parameter:
%             mechanism: a struct representing the mechanism.
%
%   Returns:
%           order: cell array containing names of bodies in their
%           respective order
%

% Get common joints between bodies
commonJoints = findCommonJoints(mechanism);
% Get the list of joints
jointNames = fieldnames(commonJoints);

% Accumulate unique name of bodies in their order of apperance
% Initialize order
order = cell(0, 0);
for i = 1:numel(jointNames)
    % Get the current joint, from which we will get bodies connected by it
    jointName = jointNames{i};
    
    % Accumulate unique body names so that we can know the order of the
    % coordinate vectors from the order of the bodies.
    bodyAName = commonJoints.(jointName).bodyA.name;
    bodyBName = commonJoints.(jointName).bodyB.name;
    if ~ismember(bodyAName, order) && ~ismember(bodyBName, order)
        order{end + 1} = bodyAName;
        order{end + 1} = bodyBName;
    elseif ~ismember(bodyAName, order)
        order{end + 1} = bodyAName;
    elseif ~ismember(bodyBName, order)
        order{end + 1} = bodyBName;
    end
end

end
