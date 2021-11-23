classdef klasifikasidata < handle

    %% Public properties
    properties (SetAccess = private)
        FileNamePrefix = '';
        CalInertialAndMagneticData = [];
       
    end

    %% Public methods
    methods (Access = public)
        function obj = klasifikasidata(varargin)
            % Create data objects from files
            obj.FileNamePrefix = varargin{1};
            dataImported = false;
            try obj.CalInertialAndMagneticData = ambildata(obj.FileNamePrefix); dataImported = true; catch e, end
               if(~dataImported)
                error('No data was imported.');
            end

            % Apply SampleRate from register data
            try h = obj.CalInertialAndMagneticData; h.SampleRate = obj.SampleRateFromRegValue(obj.RegisterData.GetValueAtAddress(69)); catch e, end
           
            % Apply SampleRate if specified as argument
            for i = 2:2:(nargin)
                if  strcmp(varargin{i}, 'InertialMagneticSampleRate')
                    try h = obj.CalInertialAndMagneticData; h.SampleRate = varargin{i+1}; catch e, end
                  else 
                    error('Invalid argument.');
                end
            end
        end
        function obj = Plot(obj)
            try obj.CalInertialAndMagneticData.Plot(); catch e, end
        end
    end

    %% Private methods
    methods (Access = private)
        function sampleRate = SampleRateFromRegValue(obj, value)
            sampleRate = floor(2^(value-1));
        end
    end
end