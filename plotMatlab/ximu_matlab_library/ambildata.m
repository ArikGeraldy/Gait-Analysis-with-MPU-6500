classdef ambildata < InertialAndMagneticDataBaseClass

    %% Public 'read-only' properties
    properties (SetAccess = private)
        FileNameAppendage = '_imu.csv';
    end

    %% Public methods
    methods (Access = public)
        function obj = ambildata(varargin)
            fileNamePrefix = varargin{1};
            for i = 2:2:nargin
                if  strcmp(varargin{i}, 'SampleRate'), obj.SampleRate = varargin{i+1};
                else error('Invalid argument.');
                end
            end
            obj.Import(fileNamePrefix);

            % Set protected parent class variables
           % obj.DelayUnits = 'ms';
            obj.GyroscopeUnits = '^\circ/s';
            obj.AccelerometerUnits = 'g';
            obj.MagnetometerUnits = 'G';
        end
    end
end