function Fq = MyJacobian(mechanism)
% Jacobian.m: Computes the jacobian of the mechanism.
%
%  Parameter:
%            mechanism: a struct representing the mechanism.
%  Returns:
%           Fq: The jacobian matrix
%

% Initialize the size of the Jacobian matrix, one body is the ground
% which doesn't count. and also initialize a counter that will be used to
% iteratively update Fq
Fq = zeros(3*(numel(fieldnames(mechanism)) - 1), 3*(numel(fieldnames(mechanism)) - 1));
rowCounter = 1;

% Get common joints between bodies
commonJoints = findCommonJoints(mechanism);
% determine the degree of freedom
dof = degreeOfFreedom(mechanism, commonJoints);
% Initialize the number of driving constraints
count_drive = dof;

% Get mechanism bodies in their proper order
bodies = bodyOrder(mechanism);

% Fill the elements of the Jacobian matrix iteratively.
% Get the list of joints
jointNames = fieldnames(commonJoints);
for i = 1:numel(jointNames)
    jointName = jointNames{i};
    type = commonJoints.(jointName).type;
    isDriving = commonJoints.(jointName).driving;

    % Get the body names connected by the current joint. This will be used
    % to determine the column index of the jacobian (based on the index of
    % qi and qj)
    bodyAName = commonJoints.(jointName).bodyA.name;
    bodyBName = commonJoints.(jointName).bodyB.name;
    % Determine the indices of the columns of Fq that corresponds to qi
    % and qj
    columnIndexI = find(strcmp(bodies, bodyAName)) - 1;
    columnIndexI = 3*columnIndexI - 2;
    columnIndexJ = find(strcmp(bodies, bodyBName)) - 1;
    columnIndexJ = 3*columnIndexJ - 2;

    if strcmp(type, 'R')
        jointLocation = commonJoints.(jointName).location;
        piILocation = commonJoints.(jointName).bodyA.com; % Location of LRF of body i
        piJLocation = commonJoints.(jointName).bodyB.com; % Location of LRF of body j
        SA = jointLocation - piILocation;
        SB = jointLocation - piJLocation;
        qi = commonJoints.(jointName).bodyA.q0;           % coordinate vector qi
        qj = commonJoints.(jointName).bodyB.q0;           % coordinate vector qj
        ri = qi(1:2); fi_i = qi(3);
        rj = qj(1:2); fi_j = qj(3);

        % Check if index I is representing the ground object
        if columnIndexI < 0
            % qi is the ground; therefore, consider only qj
            Fq(rowCounter:rowCounter + 1, columnIndexJ:columnIndexJ + 1) = -eye(2);
            Fq(rowCounter:rowCounter + 1, columnIndexJ + 2) = -omega*Rotation(fi_j)*SB;
        elseif columnIndexJ < 0
            % qj is the ground; therefore, consider only qi
            Fq(rowCounter:rowCounter + 1, columnIndexI:columnIndexI + 1) = eye(2);
            Fq(rowCounter:rowCounter + 1, columnIndexI + 2) = omega*Rotation(fi_i)*SA;
        else
            % For coordinate vector qi
            Fq(rowCounter:rowCounter + 1, columnIndexI:columnIndexI + 1) = eye(2);
            Fq(rowCounter:rowCounter + 1, columnIndexI + 2) = omega*Rotation(fi_i)*SA;
            % For coordinate vector qj
            Fq(rowCounter:rowCounter + 1, columnIndexJ:columnIndexJ + 1) = -eye(2);
            Fq(rowCounter:rowCounter + 1, columnIndexJ + 2) = -omega*Rotation(fi_j)*SB;
        end

        % Driving constraint
        if isDriving
            if count_drive < 0
                error("Constraints: You can't specify more driving constraints than the DoF: %d", dof)
            end
            if columnIndexI < 0
                % Body i is ground consider only body j
                Fq(end-(count_drive - 1), columnIndexJ:columnIndexJ + 1) = zeros(1, 2);
                Fq(end-(count_drive - 1), columnIndexJ + 2) = -1;
            elseif columnIndexJ < 0
                % Body j is ground consider only body i
                Fq(end-(count_drive - 1), columnIndexI:columnIndexI + 1) = zeros(1, 2);
                Fq(end-(count_drive - 1), columnIndexI + 2) = 1;
            else
                % Corresponding to coordinate vector qi of body i
                Fq(end-(count_drive - 1), columnIndexI:columnIndexI + 1) = zeros(1, 2);
                Fq(end-(count_drive - 1), columnIndexI + 2) = 1;
                % Corresponding to coordinate vector qj of body j
                Fq(end-(count_drive - 1), columnIndexJ:columnIndexJ + 1) = zeros(1, 2);
                Fq(end-(count_drive - 1), columnIndexJ + 2) = -1;
            end
            % Update count drive
            count_drive = count_drive - 1;
        end

        % Update the row counter
        rowCounter = rowCounter + 2;

    elseif strcmp(type, 'P')
        piILocation = commonJoints.(jointName).bodyA.com; % Location of LRF of body i
        piJLocation = commonJoints.(jointName).bodyB.com; % Location of LRF of body j
        % Defining SA, SB and V vectors for translational joint
        if isfield(commonJoints.(jointName).bodyA, 'joints')
            subJointNames = fieldnames(commonJoints.(jointName).bodyA.joints);
            if numel(subJointNames) ~=2
                error("MyJacobian: Translational joint end is not conected to a body: %s", jointName)
            end
            % Iterate through all subJoints
            for j = 1:numel(subJointNames)
                currentJointName = subJointNames{j};
                if ~strcmp(currentJointName, jointName)
                    % Define common refererence point to be able to
                    % calculate vector u.
                    A = commonJoints.(jointName).bodyA.joints.(currentJointName).location;
                    SA = A - piILocation;
                    break
                end
            end
        end

        if isfield(commonJoints.(jointName).bodyB, 'joints')
            subJointNames = fieldnames(commonJoints.(jointName).bodyB.joints);
            if numel(subJointNames) ~=2
                error("MyJacobian: Translational joint end is not conected to a body: %s", jointName)
            end
            % Iterate through all subJoints
            for j = 1:numel(subJointNames)
                currentJointName = subJointNames{j};
                if ~strcmp(currentJointName, jointName)
                    B = commonJoints.(jointName).bodyB.joints.(currentJointName).location;
                    SB = A - piJLocation;
                    break
                end
            end
        end
        % Define vector d or u that is along the axis of translation, l.
        d = (A - B)/norm(A - B);
        u = d;
        % Vector v is perpendicular to vector d => rotation by 90 degree
        v = Rotation(pi/2)*d;
        % Coordinate vectors, qi and qj
        qi = commonJoints.(jointName).bodyA.q0;
        qj = commonJoints.(jointName).bodyB.q0;
        ri = qi(1:2); fi_i = qi(3);
        rj = qj(1:2); fi_j = qj(3);

        if columnIndexI < 0
            % Body i is ground; therefore, consider only body j
            % Elements corresponding to K angle constraint
            Fq(rowCounter, columnIndexJ:columnIndexJ + 1) = zeros(1, 2);
            Fq(rowCounter, columnIndexJ + 2) = -1;
            % Elements corresponding to K arrow constraints
            Fq(rowCounter + 1, columnIndexJ:columnIndexJ + 1) = (Rotation(fi_j)*v)';
            Fq(rowCounter + 1, columnIndexJ + 2) = -(Rotation(fi_j)*v)'*omega*(rj - ri - Rotation(fi_i)*SA);
        elseif columnIndexJ < 0
            % Body j is ground; therefore, consider only body i
            % Elements corresponding to K angle constraint
            Fq(rowCounter, columnIndexI:columnIndexI + 1) = zeros(1, 2);
            Fq(rowCounter, columnIndexI + 2) = 1;
            % Elements corresponding to K arrow constraints
            Fq(rowCounter + 1, columnIndexI:columnIndexI + 1) = -(Rotation(fi_j)*v)';
            Fq(rowCounter + 1, columnIndexI + 2) = -(Rotation(fi_j)*v)'*omega*Rotation(fi_i)*SA;
        else
            % Elements corresponding to K angle constraint
            Fq(rowCounter, columnIndexI:columnIndexI + 1) = zeros(1, 2);
            Fq(rowCounter, columnIndexI + 2) = 1;
            Fq(rowCounter, columnIndexJ:columnIndexJ + 1) = zeros(1, 2);
            Fq(rowCounter, columnIndexJ + 2) = -1;
            % Elements corresponding to K arrow constraints
            Fq(rowCounter + 1, columnIndexI:columnIndexI + 1) = -(Rotation(fi_j)*v)';
            Fq(rowCounter + 1, columnIndexI + 2) = -(Rotation(fi_j)*v)'*omega*Rotation(fi_i)*SA;
            Fq(rowCounter + 1, columnIndexJ:columnIndexJ + 1) = (Rotation(fi_j)*v)';
            Fq(rowCounter + 1, columnIndexJ + 2) = -(Rotation(fi_j)*v)'*omega*(rj - ri - Rotation(fi_i)*SA);
        end

        % Check if there is any driving constraint
        if isDriving
            if count_drive == 0
                error("MyJacobian: You can't specify more driving constraints than the DoF: %d", dof)
            end
            if columnIndexI < 0
                % Body i is ground; therefore, only fill elements that come
                % coordinate of body j, qj
                Fq(end-(count_drive - 1), columnIndexJ:columnIndexJ + 1) = (Rotation(fi_j)*u)';
                Fq(end-(count_drive - 1), columnIndexJ + 2) = -(Rotation(fi_j)*u)'*omega*(rj - ri - Rotation(fi_i)*SA);
                count_drive = count_drive - 1;
            elseif columnIndexJ < 0
                % Body j is ground; therefore, only fill elements that come
                % coordinate of body i, qi
                Fq(end-(count_drive - 1), columnIndexI:columnIndexI + 1) = -(Rotation(fi_j)*u)';
                Fq(end-(count_drive - 1), columnIndexI + 2) = -(Rotation(fi_j)*u)'*omega*(Rotation(fi_i)*SA);
                count_drive = count_drive - 1;
            else
                Fq(end-(count_drive - 1), columnIndexI:columnIndexI + 1) = -(Rotation(fi_j)*u)';
                Fq(end-(count_drive - 1), columnIndexI + 2) = -(Rotation(fi_j)*u)'*omega*(Rotation(fi_i)*SA);
                Fq(end-(count_drive - 1), columnIndexJ:columnIndexJ + 1) = (Rotation(fi_j)*u)';
                Fq(end-(count_drive - 1), columnIndexJ + 2) = -(Rotation(fi_j)*u)'*omega*(rj - ri - Rotation(fi_i)*SA);
                count_drive = count_drive - 1;
            end
        end

        % Update the row counter
        rowCounter = rowCounter + 2;

    else
        error("MyJacobian: Joint type - %s - is invalied (Use either 'R' or 'P')", type)
    end
end

end
