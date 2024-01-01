function updatedMech = updateMechanism(mechanism, varargin)
% define_mechanism.m: defines the structure of the mechanism
%
%Parameter:
%         mechanism: mechanism: a struct representing the mechanism.
%         varargin: optional arguments for specifying the kinematics
%         (position, velocity, acceleration) of the mechanism
%
% Returns:
%         mechanism: a struct that contains the description of the mechanisms
%         bodies. This includes, joints, center of mass locations, initial
%         coordinates, joint locations, joint type, and driving
%         constraint motion definitions.
%


% Check number of inputs
if nargin > 4
    error("defineMechanism: no more than 3 parameters are expected: %s", varargin)
end

% Determine the appropriate length of the coordinate vector
numBodies = numel(fieldnames(mechanism)) - 1; % Ground doesn't count
len = 3*numBodies;

% Parse varargin
if nargin == 2
    % Check the correctness of the length of the time vector provided
    if ~length(varargin{1}) == len
        error("updateMechanism: the mechanism has %d bodies. Incompatible " + ...
            "coordinate vector length provided: %d", numBodies, length(varargin{1}))
    end
    q = varargin{1};
    dq = zeros(len, 1);
    ddq = zeros(len, 1);
elseif nargin == 3
    % Check the correctness of the length of the time vector provided
    if ~length(varargin{1}) == len || ~length(varargin{2}) == len
        error("updateMechanism: the mechanism has %d bodies. Incompatible " + ...
            "coordinate vector length provided: %d", numBodies, length(varargin{1}))
    end
    q = varargin{1};
    dq = varargin{2};
    ddq = zeros(len, 1);
elseif nargin == 4
     % Check the correctness of the length of the time vector provided
    if ~length(varargin{1}) == len || ~length(varargin{2}) == len || ~length(varargin{3}) == len
        error("updateMechanism: the mechanism has %d bodies. Incompatible " + ...
            "coordinate vector length provided: %d", numBodies, length(varargin{1}))
    end
    q = varargin{1};
    dq = varargin{2};
    ddq = varargin{3};
else
    warning("No additional kinematic parameters value provided. Mechanism not updated.")
    updatedMech = mechanism;
    return
end

% Get list of bodies in their proper order
order = bodyOrder(mechanism);
% Find the ground and remove it from the list of bodies.
indexGround = find(strcmp(order, 'ground'));
if ~isempty(indexGround)
    % Remove the element
    order(indexGround) = [];
else
    error("updateMechanism: specify the name of ground as '%s'", "ground")
end
% Initialize kinematic variables index counter
idxCount = 1;
% Iterate over the names in order and update parameters to each body
for i = 1:length(order)
    % Get the current body name so that its parameters can be updated
    bodyName = order{i};
    % Update the kinematic parameters of the current body
    mechanism.(bodyName).q0 = q(idxCount:idxCount + 2, :);
    mechanism.(bodyName).dq = dq(idxCount:idxCount + 2, :);
    mechanism.(bodyName).ddq = ddq(idxCount:idxCount + 2, :);

    % update the counter
    idxCount = idxCount + 3;
end
% Update the mechanism
updatedMech = mechanism;

end
