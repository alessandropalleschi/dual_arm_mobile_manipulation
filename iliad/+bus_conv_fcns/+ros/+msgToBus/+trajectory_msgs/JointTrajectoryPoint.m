function slBusOut = JointTrajectoryPoint(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    maxlength = length(slBusOut.Positions);
    recvdlength = length(msgIn.Positions);
    currentlength = min(maxlength, recvdlength);
    if (max(recvdlength) > maxlength) && ...
            isequal(varargin{1}{1},ros.slros.internal.bus.VarLenArrayTruncationAction.EmitWarning)
        diag = MSLDiagnostic([], ...
                             message('ros:slros:busconvert:TruncatedArray', ...
                                     'Positions', msgIn.MessageType, maxlength, max(recvdlength), maxlength, varargin{2}));
        reportAsWarning(diag);
    end
    slBusOut.Positions_SL_Info.ReceivedLength = uint32(recvdlength);
    slBusOut.Positions_SL_Info.CurrentLength = uint32(currentlength);
    slBusOut.Positions = double(msgIn.Positions(1:slBusOut.Positions_SL_Info.CurrentLength));
    if recvdlength < maxlength
    slBusOut.Positions(recvdlength+1:maxlength) = 0;
    end
    maxlength = length(slBusOut.Velocities);
    recvdlength = length(msgIn.Velocities);
    currentlength = min(maxlength, recvdlength);
    if (max(recvdlength) > maxlength) && ...
            isequal(varargin{1}{1},ros.slros.internal.bus.VarLenArrayTruncationAction.EmitWarning)
        diag = MSLDiagnostic([], ...
                             message('ros:slros:busconvert:TruncatedArray', ...
                                     'Velocities', msgIn.MessageType, maxlength, max(recvdlength), maxlength, varargin{2}));
        reportAsWarning(diag);
    end
    slBusOut.Velocities_SL_Info.ReceivedLength = uint32(recvdlength);
    slBusOut.Velocities_SL_Info.CurrentLength = uint32(currentlength);
    slBusOut.Velocities = double(msgIn.Velocities(1:slBusOut.Velocities_SL_Info.CurrentLength));
    if recvdlength < maxlength
    slBusOut.Velocities(recvdlength+1:maxlength) = 0;
    end
    maxlength = length(slBusOut.Accelerations);
    recvdlength = length(msgIn.Accelerations);
    currentlength = min(maxlength, recvdlength);
    if (max(recvdlength) > maxlength) && ...
            isequal(varargin{1}{1},ros.slros.internal.bus.VarLenArrayTruncationAction.EmitWarning)
        diag = MSLDiagnostic([], ...
                             message('ros:slros:busconvert:TruncatedArray', ...
                                     'Accelerations', msgIn.MessageType, maxlength, max(recvdlength), maxlength, varargin{2}));
        reportAsWarning(diag);
    end
    slBusOut.Accelerations_SL_Info.ReceivedLength = uint32(recvdlength);
    slBusOut.Accelerations_SL_Info.CurrentLength = uint32(currentlength);
    slBusOut.Accelerations = double(msgIn.Accelerations(1:slBusOut.Accelerations_SL_Info.CurrentLength));
    if recvdlength < maxlength
    slBusOut.Accelerations(recvdlength+1:maxlength) = 0;
    end
    maxlength = length(slBusOut.Effort);
    recvdlength = length(msgIn.Effort);
    currentlength = min(maxlength, recvdlength);
    if (max(recvdlength) > maxlength) && ...
            isequal(varargin{1}{1},ros.slros.internal.bus.VarLenArrayTruncationAction.EmitWarning)
        diag = MSLDiagnostic([], ...
                             message('ros:slros:busconvert:TruncatedArray', ...
                                     'Effort', msgIn.MessageType, maxlength, max(recvdlength), maxlength, varargin{2}));
        reportAsWarning(diag);
    end
    slBusOut.Effort_SL_Info.ReceivedLength = uint32(recvdlength);
    slBusOut.Effort_SL_Info.CurrentLength = uint32(currentlength);
    slBusOut.Effort = double(msgIn.Effort(1:slBusOut.Effort_SL_Info.CurrentLength));
    if recvdlength < maxlength
    slBusOut.Effort(recvdlength+1:maxlength) = 0;
    end
    currentlength = length(slBusOut.TimeFromStart);
    for iter=1:currentlength
        slBusOut.TimeFromStart(iter) = bus_conv_fcns.ros.msgToBus.ros.Duration(msgIn.TimeFromStart(iter),slBusOut(1).TimeFromStart(iter),varargin{:});
    end
    slBusOut.TimeFromStart = bus_conv_fcns.ros.msgToBus.ros.Duration(msgIn.TimeFromStart,slBusOut(1).TimeFromStart,varargin{:});
end
