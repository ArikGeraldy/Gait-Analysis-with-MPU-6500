function SyncroniseData(varargin)
%SYNCRONISEDATA Syncronises time series data between klasifikasidata objects
%
%   SyncroniseData(xIMUdataStruct, StartEventTimes)
%   SyncroniseData(xIMUdataStruct, StartEventTimes, EndEventTimes)
%   SyncroniseData(xIMUdataStruct, 'UseAX0fallingEdge')
%
%   This fuction syncronises time series data between klasifikasidata objects
%   by adjusting the StartTime and SampleRate properties of each data
%   class.
%
%   StartEventTimes is vector of values representing the time at which a
%   'start synchronisation event' common to all klasifikasidata objects is
%   known to occur within each klasifikasidata object's time series data. The
%   length of this vector must equal the number of klasifikasidata objects
%   in xIMUdataStruct.
%
%   EndEventTimes is vector of values representing the time at which a
%   'end synchronisation event' common to all klasifikasidata objects is
%   known to occur within each klasifikasidata object's time series data. The
%   length of this vector must equal the number of klasifikasidata objects
%   in xIMUdataStruct.
%
%   'UseAX0fallingEdge' should be specified if a falling edge of the
%   auxilary port (configured in digital I/O mode) channel AX0 represents
%   the 'start synchronisation event' or 'start synchronisation event' and
%   'end synchronisation event'.

    %% Apply arguments
    xIMUdata = varargin{1};
    xIMUdataObjs = struct2cell(xIMUdata);
    StartEventTimes = [];
    EndEventTimes = [];
    UseAX0fallingEdge = false;
    if(ischar(varargin{2}))
        if strcmp(varargin{2}, 'UseAX0fallingEdge'), UseAX0fallingEdge = true;
        else error('Invalid argument.');
        end
    else
        StartEventTimes = varargin{2};
        if(nargin == 3)
            EndEventTimes = varargin{3};
        end
    end

    %% Use AX0 falling edge of auxiliary port in Digital I/O mode
    if(UseAX0fallingEdge)
        for i = 1:numel(xIMUdataObjs)
            fallingEdgeIndexes = [0; diff(xIMUdataObjs{i}.DigitalIOdata.State.AX0)] == -1;
            fallingEdgeTimes = xIMUdataObjs{i}.DigitalIOdata.Time(fallingEdgeIndexes);
            StartEventTimes = [StartEventTimes; fallingEdgeTimes(1)];
            if(numel(fallingEdgeTimes) > 1)
                EndEventTimes = [EndEventTimes; fallingEdgeTimes(end)];
            end
        end
    end

    %% Modify start times to synchronise start of window
    if(numel(StartEventTimes) ~= numel(xIMUdataObjs))
        error('Length of StartEventTimes vector must equal number of klasifikasidata objects');
    end
    for i = 1:numel(xIMUdataObjs)
        
        try h = xIMUdataObjs{i}.CalInertialAndMagneticData; h.StartTime = -StartEventTimes(i); catch e, end
        
    end

    %% Modify sample rate to synchronise end of window
    if(numel(EndEventTimes) == 0)
        return;
    end
    if(numel(EndEventTimes) ~= numel(xIMUdataObjs))
        error('Length of EndEventTimes vector must equal number of klasifikasidata objects');
    end
    scalers = (EndEventTimes - StartEventTimes) * (1/((EndEventTimes(1)-StartEventTimes(1))));
    for i = 2:numel(xIMUdataObjs)
        
        try h = xIMUdataObjs{i}.CalInertialAndMagneticData; h.SampleRate = scalers(i)*h.SampleRate; h.StartTime = -StartEventTimes(i)/scalers(i); catch e, end
       
    end
end
