function rosmsgOut = JointTrajectoryPoint(slBusIn, rosmsgOut)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    rosmsgOut.Positions = double(slBusIn.Positions(1:slBusIn.Positions_SL_Info.CurrentLength));
    rosmsgOut.Velocities = double(slBusIn.Velocities(1:slBusIn.Velocities_SL_Info.CurrentLength));
    rosmsgOut.Accelerations = double(slBusIn.Accelerations(1:slBusIn.Accelerations_SL_Info.CurrentLength));
    rosmsgOut.Effort = double(slBusIn.Effort(1:slBusIn.Effort_SL_Info.CurrentLength));
    rosmsgOut.TimeFromStart = bus_conv_fcns.ros.busToMsg.ros.Duration(slBusIn.TimeFromStart,rosmsgOut.TimeFromStart(1));
end
