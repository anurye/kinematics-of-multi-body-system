function updatedMechanism = markersKinematics(mechanism)
% MarkersKinematics: Perform kinematic analysis of markers located on the
% mechanism's bodies.
%
%   Parameter:
%            mechanism: a struct representing the mechanism and everything
%            it contains
%
%   Returns:
%           updatedMechanism: the mechanism that contains kinematics
%           information for markers.
%

% Iterate through each body
bodyNames = fieldnames(mechanism);
for i = 1:numel(bodyNames)
    % Get the current body name
    bodyName = bodyNames{i};
    % Access the markers connected to bodyI
    if isfield( mechanism.(bodyName), 'markers')
        % Get the marker name(s)
        markers = fieldnames( mechanism.(bodyName).markers);
        % Iterate through each markers
        for k = 1:numel(markers)
            % Get the current marker name
            markerName = markers{k};
            % Get the location of the local reference frame of bodyI
            piILocation =  mechanism.(bodyName).com;
            % Get the location of the marker w.r.t GRF
            markerLocation =  mechanism.(bodyName).markers.(markerName).location;
            % Calculate the vector SA w.r.t the LRF of bodyI
            SA = markerLocation - piILocation;

            % Get the kinematics of bodyI
            q =  mechanism.(bodyName).q0;
            dq =  mechanism.(bodyName).dq;
            ddq =  mechanism.(bodyName).ddq;

            % Compute the kinematics of the marker
            % Initialize the size of variables
            qM = zeros(size(q));
            dqM = zeros(size(dq));
            ddqM = zeros(size(ddq));

            % Compute the elements iteratively
            for j = 1:size(q, 2)
                % Get parameters of body necessary to compute marker
                % kinematics
                ri = q(1:2, j); fi_i = q(3, j);
                dri = dq(1:2, j); dfi_i = dq(3, j);
                ddri = ddq(1:2, j); ddfi_i = ddq(3, j);

                % Compute marker kinematics
                % Position
                qM(1:2, j) = ri + Rotation(fi_i)*SA;
                qM(3, j) = fi_i;
                % Velocity
                dqM(1:2, j) = dri + omega*Rotation(fi_i)*SA*dfi_i;
                dqM(3, j) = dfi_i;
                % Acceleration
                ddqM(1:2, j) = ddri + omega*Rotation(fi_i)*SA*ddfi_i - ...
                    Rotation(fi_i)*SA*dfi_i^2;
                ddqM(3, j) = ddfi_i;
            end
            % Add the current marker's kinematics to the body
            mechanism.(bodyName).markers.(markerName).q = qM;
            mechanism.(bodyName).markers.(markerName).dq = dqM;
            mechanism.(bodyName).markers.(markerName).ddq = ddqM;
        end
    else
        if strcmp(bodyName, 'time')
            continue
        end
        warning("There is no marker associated" + ...
            " with body: %s\n", bodyName);
    end
end

% Return the updated mechanism
updatedMechanism = mechanism;

end