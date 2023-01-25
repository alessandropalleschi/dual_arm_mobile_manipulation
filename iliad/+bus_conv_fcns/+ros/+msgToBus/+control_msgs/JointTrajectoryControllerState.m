function slBusOut = JointTrajectoryControllerState(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    currentlength = length(slBusOut.Header);
    for iter=1:currentlength
        slBusOut.Header(iter) = bus_conv_fcns.ros.msgToBus.std_msgs.Header(msgIn.Header(iter),slBusOut(1).Header(iter),varargin{:});
    end
    slBusOut.Header = bus_conv_fcns.ros.msgToBus.std_msgs.Header(msgIn.Header,slBusOut(1).Header,varargin{:});
    maxlength = length(slBusOut.JointNames);
    recvdlength = length(msgIn.JointNames);
    currentlength = min(maxlength, recvdlength);
    if (max(recvdlength) > maxlength) && ...
            isequal(varargin{1}{1},ros.slros.internal.bus.VarLenArrayTruncationAction.EmitWarning)
        diag = MSLDiagnostic([], ...
                             message('ros:slros:busconvert:TruncatedArray', ...
                                     'JointNames', msgIn.MessageType, maxlength, max(recvdlength), maxlength, varargin{2}));
        reportAsWarning(diag);
    end
    slBusOut.JointNames_SL_Info.ReceivedLength = uint32(recvdlength);
    slBusOut.JointNames_SL_Info.CurrentLength = uint32(currentlength);
    for iter=1:currentlength
        recvlen = strlength(msgIn.JointNames(iter));
        maxlen = length(slBusOut.JointNames(iter).Data);
        curlen = min(recvlen, maxlen);
        if (max(recvlen) > maxlen) && ...
                isequal(varargin{1}{1},ros.slros.internal.bus.VarLenArrayTruncationAction.EmitWarning)
            diag = MSLDiagnostic([], ...
                                 message('ros:slros:busconvert:TruncatedArray', ...
                                         'JointNames', msgIn.MessageType, maxlen, max(recvdlength), maxlength, varargin{2}));
            reportAsWarning(diag);
        end
        slBusOut.JointNames(iter).Data_SL_Info.CurrentLength = uint32(curlen);
        slBusOut.JointNames(iter).Data_SL_Info.ReceivedLength = uint32(recvlen);
        slBusOut.JointNames(iter).Data(1:curlen) = uint8(char(msgIn.JointNames(iter)));
    end
    currentlength = length(slBusOut.Desired);
    for iter=1:currentlength
        slBusOut.Desired(iter) = bus_conv_fcns.ros.msgToBus.trajectory_msgs.JointTrajectoryPoint(msgIn.Desired(iter),slBusOut(1).Desired(iter),varargin{:});
    end
    slBusOut.Desired = bus_conv_fcns.ros.msgToBus.trajectory_msgs.JointTrajectoryPoint(msgIn.Desired,slBusOut(1).Desired,varargin{:});
    currentlength = length(slBusOut.Actual);
    for iter=1:currentlength
        slBusOut.Actual(iter) = bus_conv_fcns.ros.msgToBus.trajectory_msgs.JointTrajectoryPoint(msgIn.Actual(iter),slBusOut(1).Actual(iter),varargin{:});
    end
    slBusOut.Actual = bus_conv_fcns.ros.msgToBus.trajectory_msgs.JointTrajectoryPoint(msgIn.Actual,slBusOut(1).Actual,varargin{:});
    currentlength = length(slBusOut.Error);
    for iter=1:currentlength
        slBusOut.Error(iter) = bus_conv_fcns.ros.msgToBus.trajectory_msgs.JointTrajectoryPoint(msgIn.Error(iter),slBusOut(1).Error(iter),varargin{:});
    end
    slBusOut.Error = bus_conv_fcns.ros.msgToBus.trajectory_msgs.JointTrajectoryPoint(msgIn.Error,slBusOut(1).Error,varargin{:});
end
