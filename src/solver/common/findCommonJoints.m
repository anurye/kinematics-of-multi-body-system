function commonJoints = findCommonJoints(mechanism)
% findCommonJoints: Finds joints that are common between two bodies
%
%  Parameter:
%            mechanism: a struct representing the mechanism.
%
%  Returns:
%     commonJoints: a struct of joints, and its properties. 
%     This includes the bododies connected by the particular joint, the
%     joint type, whether the joint is connected to a drive and the joint
%     location.
%

commonJoints = struct();
% Iterate through each body
bodyNames = fieldnames(mechanism);
numBodies = numel(bodyNames);

% Iterate through pairs of bodies
for i = 1:numBodies
    for j = i+1:numBodies
        bodyAName = bodyNames{i};
        bodyBName = bodyNames{j};

        bodyA = mechanism.(bodyAName);
        bodyB = mechanism.(bodyBName);

        % Check if the bodies have joints connected to them
        if isfield(bodyA, 'joints') && isfield(bodyB, 'joints')
            % Find common joints
            commonJointNames = intersect(fieldnames(bodyA.joints), fieldnames(bodyB.joints));

            % Iterate through common joints
            for k = 1:numel(commonJointNames)
                jointName = commonJointNames{k};
                jointA = bodyA.joints.(jointName);
                jointB = bodyB.joints.(jointName);
                % Check if the joint is valied. To be valied: the joint
                % have to have the same type, location and driving
                % condition on both of the two bodies it is connecting.
                typeCheck = strcmp(jointA.type, jointB.type);
                locationCheck = all(abs(jointA.location - jointB.location)...
                    < Tolerance | (isnan(jointA.location) & isnan(jointB.location)));
                drivingCheck = jointA.driving == jointB.driving;
                if ~(typeCheck && locationCheck && drivingCheck)
                    error("findCommonJoints: One joint connecting two " + ...
                        "bodies must have the same properties on both bodies.")
                end
                % Store common joint information
                commonJoints.(jointName).bodyA.name = bodyAName;
                commonJoints.(jointName).bodyA.com = bodyA.com;
                commonJoints.(jointName).bodyA.q0 = bodyA.q0;
                commonJoints.(jointName).bodyA.dq = bodyA.dq;
                commonJoints.(jointName).bodyA.ddq = bodyA.ddq;
                commonJoints.(jointName).bodyA.joints = bodyA.joints;

                commonJoints.(jointName).bodyB.name = bodyBName;
                commonJoints.(jointName).bodyB.com = bodyB.com;
                commonJoints.(jointName).bodyB.q0 = bodyB.q0;
                commonJoints.(jointName).bodyB.dq = bodyB.dq;
                commonJoints.(jointName).bodyB.ddq = bodyB.ddq;
                commonJoints.(jointName).bodyB.joints = bodyB.joints;

                % Since we have already checked that jointA and jointB are
                % essentially the same joint we can return one of them.
                commonJoints.(jointName).type = jointA.type;
                commonJoints.(jointName).location = jointA.location;
                commonJoints.(jointName).driving = jointA.driving;
                if jointA.driving
                    commonJoints.(jointName).fAB = jointA.fAB;
                end
            end
        end
    end
end
end
