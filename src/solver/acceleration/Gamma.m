function gamma = Gamma(mechanism, ti)
% Gamma.m: computes the gamma vector that is necessary for solving the
% acceleration problem
%
%   Parameter:
%          mechanism: a struct representing the mechanism.
%          ti: time instant at which the gamma vector will be evaluated
%
%   Returns:
%          gamma: Right hand side equation of the acceleration problem
%

% Define a symbolic variable, t, that will be used for differentiation
syms t

% determine the degree of freedom, this is the same as the number of
% driving constraints => number of rows of gamma which are non-zero.
commonJoints = findCommonJoints(mechanism);    % Get common joints between bodies
dof = degreeOfFreedom(mechanism, commonJoints);
% Initialize the number of driving constraints
count_drive = dof;

% Get mechanism bodies in their proper order
bodies = bodyOrder(mechanism);

% Initialize the size of gamma and row counter
gamma = zeros(3*(numel(fieldnames(mechanism)) - 1), 1);
rowCounter = 1;

% Fill the elements of gamma vector iteratively
% Get the list of joints
jointNames = fieldnames(commonJoints);
for i = 1:numel(jointNames)
    % Get joint properties
    jointName = jointNames{i};
    type = commonJoints.(jointName).type;
    isDriving = commonJoints.(jointName).driving;
    % Get location of Local Reference Frames (LRF)
    piILocation = commonJoints.(jointName).bodyA.com;
    piJLocation = commonJoints.(jointName).bodyB.com;
    % Extract position and velocity information
    qi = commonJoints.(jointName).bodyA.q0;   % Coordinate vector qi
    dqi = commonJoints.(jointName).bodyA.dq;  % Velocity of body i
    qj = commonJoints.(jointName).bodyB.q0;   % Coordinate vector qj
    dqj = commonJoints.(jointName).bodyB.dq;  % Velocity of body j
    ri = qi(1:2); fi_i = qi(3); dri = dqi(1:2); dfi_i = dqi(3);
    rj = qj(1:2); fi_j = qj(3); drj = dqj(1:2); dfi_j = dqj(3);

    % Detrmine the index of body i and body j in bodies. bodies contains
    % order of bodies in the mechanism.
    bodyAName = commonJoints.(jointName).bodyA.name;
    bodyBName = commonJoints.(jointName).bodyB.name;
    % The ground doesn't count; therefore, it is removed by subtracting 1.
    indexBodyA = find(strcmp(bodies, bodyAName)) - 1;
    indexBodyB = find(strcmp(bodies, bodyBName)) - 1;

    if strcmp(type, 'R')
        % Get joint location
        jointLocation = commonJoints.(jointName).location;
        % Compute SA and SB vectrors
        SA = jointLocation - piILocation;
        SB = jointLocation - piJLocation;
        if indexBodyA == 0
            % This meand bodA(body i) is the ground; therefore, consider
            % only body j.
            % Update the potentially non-zero entiries of gamma
            gamma(rowCounter:rowCounter + 1, 1) = -Rotation(fi_j)*SB*dfi_j^2;
        elseif indexBodyB == 0
            % This meand bodB(body j) is the ground; therefore, consider
            % only body i.
            % Update the potentially non-zero entiries of gamma
            gamma(rowCounter:rowCounter + 1, 1) = Rotation(fi_i)*SA*dfi_i^2;
        else
            % Update the potentially non-zero entiries of gamma
            gamma(rowCounter:rowCounter + 1, 1) = Rotation(fi_i)*SA*dfi_i^2 - Rotation(fi_j)*SB*dfi_j^2;
        end

        % Driving constraint
        if isDriving
            if count_drive < 0
                error("Gamma: You can't specify more driving constraints than the DoF: %d", dof)
            end
            fAB = commonJoints.(jointName).fAB;
            % Differentiate fAB twice to get (fAB)tt
            fABtt = diff(diff(fAB, t), t);
            % Evaluate fABtt at the current time instant ti
            fABtt_ti = subs(fABtt, ti);
            % Fill element of the gamma vector
            gamma(end - (count_drive - 1), 1) = 0 + fABtt_ti;

            % Update driver count
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
                    error("Gamma: Translational joint end is not conected to a body: %s", jointName)
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
                    error("Gamma: Translational joint end is not conected to a body: %s", jointName)
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

        % Element corresponding to K angle
        gamma(rowCounter, 1) = 0;
        % Element corresponding to K arrow
        gamma(rowCounter + 1, 1) = (Rotation(fi_j)*v)'*...
            (2*omega*(drj - dri)*dfi_j + (rj - ri)*dfi_j^2 - Rotation(fi_i)*SA*(dfi_j - dfi_i)^2);

        % Check if driving
        if isDriving
            if count_drive < 0
                error("Gamma: You can't specify more driving constraints than the DoF: %d", dof)
            end
            fAB = commonJoints.(jointName).fAB;
            % Differentiate fAB twice to get (fAB)tt
            fABtt = diff(diff(fAB, t), t);
            % Evaluate fABtt at the current time instant ti
            fABtt_ti = subs(fABtt, ti);
            % Fill element of the gamma vector
            gamma(end - (count_drive - 1), 1) = (Rotation(fi_j)*u)'*...
                (2*omega*(drj - dri)*dfi_j + (rj - ri)*dfi_j^2 - Rotation(fi_i)*SA*(dfi_j - dfi_i)^2) + fABtt_ti;

            % Update driver count
            count_drive = count_drive - 1;

        end
    end

    % Update rowCounter
    rowCounter = rowCounter + 2;

end

end
