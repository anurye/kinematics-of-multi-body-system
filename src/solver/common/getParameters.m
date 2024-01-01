function [Q, dQ, ddQ] = getParameters(mechanism, varargin)
% getParameters.m: extracts kinematic parameters (position, velocity and
% acceleration) of bodies of the mechanism
%
%   Parameter:
%             mechanism: a struct representing the mechanism.
%
%   Returns:
%          q: position
%         dq: velocity
%        ddq: acceleration
%

% Parse varargin
if nargin > 2
    error("getParameters: only one optional argument can be specified: %s", varargin)
end

if nargin == 2
    numColumns = length(varargin{1});
else
    numColumns = 1;
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

% Initialize variables. Their size can be determined from the number of
% bodies and numColumn
% Determine the appropriate length of the coordinate vector
numBodies = numel(fieldnames(mechanism)) - 1; % Ground doesn't count
numRows = 3*numBodies;
Q = zeros(numRows, numColumns);
dQ = zeros(numRows, numColumns);
ddQ = zeros(numRows, numColumns);
% Initialize row counter
rowCounter = 1;
% Iterate over the names of bodies and get their kinematic parameters
for i = 1:length(order)
    % Get the current body name so that its parameters can be extracted
    bodyName = order{i};
    % Extract the kinematic parameters of the current body
    q = mechanism.(bodyName).q0;
    dq = mechanism.(bodyName).dq;
    ddq = mechanism.(bodyName).ddq;
    % Append them to the list
    Q(rowCounter:rowCounter + 2, :) = q;
    dQ(rowCounter:rowCounter + 2, :) = dq;
    ddQ(rowCounter:rowCounter + 2, :) = ddq;

    % Update rowCounter
    rowCounter = rowCounter + 3;
end

end
