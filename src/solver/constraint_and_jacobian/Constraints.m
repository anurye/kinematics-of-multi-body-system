function F = Constraints(mechanism, t)
% Constraints.m: define and assembles the kinematic and driving
% constraints of the mechanism.
%
% Parameters:
%       mechanism: a struct representing the mechanism.
%       t: time instant at which the the constraint vector is going to be
%       evaluated.
%
% Returns:
%         F: constraint vector evaluated at time t.
%

% Initialize the size of the constraint vector, one body is the ground
% which doesn't count. and also initialize a counter that will be used to
% iteratively update F
F = zeros(3*(numel(fieldnames(mechanism)) - 1), 1);
counter = 1;

% Get common joints between bodies
commonJoints = findCommonJoints(mechanism);

% determine the degree of freedom
dof = degreeOfFreedom(mechanism, commonJoints);
% Initialize the number of driving constraints
count_drive = dof;

% Define the constraint for each joint based on its type
% Get the list of joints
jointNames = fieldnames(commonJoints);
for i = 1:numel(jointNames)
    jointName = jointNames{i};
    type = commonJoints.(jointName).type;
    isDriving = commonJoints.(jointName).driving;

    % Location of the LRF for body i and j
    piILocation = commonJoints.(jointName).bodyA.com; % LRF of body i
    piJLocation = commonJoints.(jointName).bodyB.com; % LRF of body j

    % Coordinate vectors for body i and j
    qi = commonJoints.(jointName).bodyA.q0; % coordinate vector qi
    qj = commonJoints.(jointName).bodyB.q0; % coordinate vector qj
    ri = qi(1:2); fi_i = qi(3);
    rj = qj(1:2); fi_j = qj(3);

    if strcmp(type, 'R')
        jointLocation = commonJoints.(jointName).location;
        SA = jointLocation - piILocation;
        SB = jointLocation - piJLocation;
        % Fill elements of the constraint matrix
        F(counter:counter+1, 1) = ri + Rotation(fi_i)*SA - (rj + Rotation(fi_j)*SB);

        % Driving constraint
        if isDriving
            if count_drive < 0
                error("Constraints: You can't specify more driving constraints than the DoF: %d", dof)
            end
            fAB = commonJoints.(jointName).fAB;
            F(end-(count_drive - 1), 1) = fi_i - fi_j - fAB(t);
            count_drive = count_drive - 1;
        end

    elseif strcmp(type, 'P')
        % Defining SA, SB and V vectors for translational joint
        if strcmp(commonJoints.(jointName).bodyA.name, 'ground')
            A = commonJoints.(jointName).reference;
            if isfield(commonJoints.(jointName).bodyB, 'joints')
                subJointNames = fieldnames(commonJoints.(jointName).bodyB.joints);
                if numel(subJointNames) ~=2
                    error("Constraints: Translational joint end is not conected to a body: %s", jointName)
                end
                % Iterate through all subJoints
                for j = 1:numel(subJointNames)
                    currentJointName = subJointNames{j};
                    if ~strcmp(currentJointName, jointName)
                        B = commonJoints.(jointName).bodyB.joints.(currentJointName).location;
                        break
                    end
                end
            end

        elseif strcmp(commonJoints.(jointName).bodyB.name, 'ground')
            B = commonJoints.(jointName).reference;
            if isfield(commonJoints.(jointName).bodyA, 'joints')
                subJointNames = fieldnames(commonJoints.(jointName).bodyA.joints);
                if numel(subJointNames) ~=2
                    error("Constraints: Translational joint end is not conected to a body: %s", jointName)
                end
                % Iterate through all subJoints
                for j = 1:numel(subJointNames)
                    currentJointName = subJointNames{j};
                    if ~strcmp(currentJointName, jointName)
                        % Define common refererence point to be able to
                        % calculate U
                        A = commonJoints.(jointName).bodyA.joints.(currentJointName).location;
                        break
                    end
                end
            end
            
        else
            if isfield(commonJoints.(jointName).bodyA, 'joints')
                subJointNames = fieldnames(commonJoints.(jointName).bodyA.joints);
                if numel(subJointNames) ~=2
                    error("Constraints: Translational joint end is not conected to a body: %s", jointName)
                end
                % Iterate through all subJoints
                for j = 1:numel(subJointNames)
                    currentJointName = subJointNames{j};
                    if ~strcmp(currentJointName, jointName)
                        % Define common refererence point to be able to
                        % calculate U
                        A = commonJoints.(jointName).bodyA.joints.(currentJointName).location;
                        break
                    end
                end
            end

            if isfield(commonJoints.(jointName).bodyB, 'joints')
                subJointNames = fieldnames(commonJoints.(jointName).bodyB.joints);
                if numel(subJointNames) ~=2
                    error("Constraints: Translational joint end is not conected to a body: %s", jointName)
                end
                % Iterate through all subJoints
                for j = 1:numel(subJointNames)
                    currentJointName = subJointNames{j};
                    if ~strcmp(currentJointName, jointName)
                        B = commonJoints.(jointName).bodyB.joints.(currentJointName).location;
                        break
                    end
                end
            end
        end

        % Vector SA and SB
        SA = A - piILocation;
        SB = A - piJLocation;

        % Define vector d or u that is along the axis of translation, l.
        d = (A - B)/norm(A - B);
        u = d;
        % Vector v is perpendicular to vector d => rotation by 90 degree
        v = Rotation(pi/2)*d;

        % Fill elements of the constraint matrix
        F(counter, 1) = fi_i - fi_j;
        F(counter + 1, 1) = (Rotation(fi_j)*v)'*(rj + Rotation(fi_j)*SB...
            - ri - Rotation(fi_i)*SA);

        if isDriving
            if count_drive < 0
                error("Constraints: You can't specify more driving constraints than the DoF: %d", dof)
            end
            fAB = commonJoints.(jointName).fAB;
            F(end-(count_drive - 1), 1) = (Rotation(fi_j)*u)'*(rj + Rotation(fi_j)*SB - ri - Rotation(fi_i)*SA) - fAB(t);
            count_drive = count_drive - 1;
        end

    else
        error("Constraints: Joint type: %s is invalied (Use either 'R' or 'P')", type)
    end

    % Update counter
    counter = counter + 2;

end

end

