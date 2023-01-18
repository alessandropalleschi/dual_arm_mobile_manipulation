function rosmsgOut = JointTrajectory(slBusIn, rosmsgOut)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    rosmsgOut.Header = bus_conv_fcns.ros.busToMsg.std_msgs.Header(slBusIn.Header,rosmsgOut.Header(1));
    for iter=1:slBusIn.JointNames_SL_Info.CurrentLength
        rosmsgOut.JointNames{iter} = char(slBusIn.JointNames(iter).Data).';
        maxlen = length(slBusIn.JointNames(iter).Data);
        if slBusIn.JointNames(iter).Data_SL_Info.CurrentLength < maxlen
        rosmsgOut.JointNames{iter}(slBusIn.JointNames(iter).Data_SL_Info.CurrentLength+1:maxlen) = [];
        end
    end
    if slBusIn.JointNames_SL_Info.CurrentLength < numel(rosmsgOut.JointNames)
        rosmsgOut.JointNames(slBusIn.JointNames_SL_Info.CurrentLength+1:numel(rosmsgOut.JointNames)) = [];
    end
    rosmsgOut.JointNames = rosmsgOut.JointNames.';
    rosmsgOut.Points = ros.slroscpp.internal.bus.Util.newMessageFromSimulinkMsgType('trajectory_msgs/JointTrajectoryPoint');
    for iter=1:slBusIn.Points_SL_Info.CurrentLength
        rosmsgOut.Points(iter) = bus_conv_fcns.ros.busToMsg.trajectory_msgs.JointTrajectoryPoint(slBusIn.Points(iter),rosmsgOut.Points(1));
    end
    if slBusIn.Points_SL_Info.CurrentLength < numel(rosmsgOut.Points)
    rosmsgOut.Points(slBusIn.Points_SL_Info.CurrentLength+1:numel(rosmsgOut.Points)) = [];
    end
end
