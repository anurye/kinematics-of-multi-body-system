function dispCommonJoints(commonJoints)
% dispCommonJoints.m: used to display joint properties. This includes the
% bodies that are connected by the joint, the joint type and its location
%
% Parameter:
%           commonJoints: struct representing set of joints that are common
%           between two bodies.
%
    jointNames = fieldnames(commonJoints);
    
    for i = 1:numel(jointNames)
        jointName = jointNames{i};
        disp(['Joint Name: ' jointName]);
        disp(['Body A: ' commonJoints.(jointName).bodyA.name]);
        disp(['Body B: ' commonJoints.(jointName).bodyB.name]);
        disp(['Type: ' commonJoints.(jointName).type]);
        disp(['Location: ' num2str(commonJoints.(jointName).location')]);
        disp('-----------------------------');
    end
end