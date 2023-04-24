function rosmsgOut = Duration(slBusIn, rosmsgOut)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    rosmsgOut.Sec = int32(slBusIn.Sec);
    rosmsgOut.Nsec = int32(slBusIn.Nsec);
end
