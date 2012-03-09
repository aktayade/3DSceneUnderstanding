classdef ExperimentBase < handle
    %EXPERIMENTBASE Base class to specify options for an experiment
    %   Detailed explanation goes here
    
    
    methods (Static)
        function [] = check_matlabpool()
            %{
            if matlabpool('size') == 0
                a = input('NOTE MATLABPOOL IS NOT RUNNING ARE YOU SURE?\n', 's');
                if lower(a(1)) ~= 'y'
                    error('Matlab pool was not running');
                end
            end
            %}
        end
        
        function [path] = generate_write_file_path(experiment, tag, dataset, file, varargin)
        %GENERATE_WRITE_FILE_PATH Standardizes the file system layout
        % experiment, tag, dataset, file
        % This function will create the directory if it doesn't exist
        % Extra variable arguments will be appended to the filename with -
        % for example myfile-arg1-arg2.mat
            globalpaths;
            finalFileName = file;
            for varg = varargin
                if ~isempty(cell2mat(varg))
                    finalFileName = [finalFileName '-' cell2mat(varg)];
                end
            end
            fpath = sprintf('%s\\%s\\%s\\%s', outputFolder, experiment, tag, dataset);
            if ~exist(fpath,'dir')
                mkdir(fpath);
            end
            path = [fpath '\' finalFileName];
        end
        
        function [path] = generate_read_file_path(experiment, tag, dataset, file, varargin)
        %GENERATE_FILE_PATH Standardizes the file system layout
        % experiment, tag, dataset, file
        % This function does not create a directory if it doesn't exist
        % Extra variable arguments will be appended to the filename with -
        % for example myfile-arg1-arg2.mat
            globalpaths;
            finalFileName = file;
            for varg = varargin
                if ~isempty(cell2mat(varg))
                    finalFileName = [finalFileName '-' cell2mat(varg)];
                end
            end
            path = sprintf('%s\\%s\\%s\\%s\\%s', outputFolder, experiment, ...
                tag, dataset, finalFileName);
        end
        
        function [results] = collect_results(resultsPath, reader)
            globalpaths;
            pathStart = [outputFolder resultsPath];
            [results] = ExperimentBase.collect_results_sub(pathStart, '', reader);
        end
    end
    
    methods(Access = private, Static)
        function [results] = collect_results_sub(path, experimentName, reader)
            results = cell(0);
            directories = dir(path);
            for ii = 1:length(directories)
                d = directories(ii);
                if ~d.isdir && d.name(1) ~= '.'
                    expName = strrep(lower([experimentName d.name]), '.mat', '');
                    result = reader([path '\' d.name], expName);
                    results = [results; result];
                end
            end
            for ii = 1:length(directories)
                d = directories(ii);
                if d.isdir && d.name(1) ~= '.'
                    subResults = ExperimentBase.collect_results_sub(...
                        [path '\' d.name], [experimentName d.name '-'], reader);
                    results = [results; subResults];
                end
            end
        end
    end
        
end

