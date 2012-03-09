classdef ExperimentHyperParameterSearch < ExperimentBase
    %EXPERIMENTHYPERPARAMETERSEARCH Model selection code
    %   This code will take a dataset and find the optimal hyper parameters
    %   for a given classifier.
    
    properties (Constant)
        experimentName = 'HyperParameterSearch';
        kFolds = 4;
    end
    
    methods (Static)
        function [] = run(data, searchType, performanceType, generateTestAccuracy)
        % data = a Dataset class instance with the data to use
        % searchType = 'grid' or 'line'
        % performanceType = 'linearsvm', 'gaussiansvm'
            if (nargin < 4)
                generateTestAccuracy = false;
            end
            
            % Ask user if matlab pool should be running
            ExperimentBase.check_matlabpool();
            
            [perfFunc, paramSpace, startPoint, searchInfo] = ...
                ExperimentHyperParameterSearch.set_performance_function(performanceType, searchType);
            
            % If applicable, compute start point heuristic for gaussianSVM
            if strcmpi(performanceType, 'gaussiansvm') && ...
                    (strcmpi(searchType, 'line') || strcmpi(searchType, 'jaak'))
                startPoint = [1, Jaakkola(data.xtrain)];
            end
            
            % Values between 1 and KFolds labeling which fold a datapoint is in
            cvIndices = crossvalind('Kfold', data.ytrain, ExperimentHyperParameterSearch.kFolds);
            
            switch lower(searchType)
                case 'grid'
                    [hyperParameters, ~, shownWork, MP_grid, MR_grid] = ...
                        GridSearch(data.xtrain, data.ytrain, paramSpace, ...
                            cvIndices, perfFunc, 0.5);
                case 'line'
                    [hyperParameters, ~, shownWork, MP_grid, MR_grid] = ...
                        LineSearch(startPoint, data.xtrain, data.ytrain, ...
                            paramSpace, cvIndices, perfFunc, 0.5);
                case 'jaak'
                    hyperParameters = startPoint;
                otherwise
                    error('Unknown search type: %s', searchType);
            end
            
            path = ExperimentBase.generate_write_file_path('hypersearch', ...
                'results', data.name, performanceType, searchType);
            
            if generateTestAccuracy
                
                [ risk, accuracy, classifier, MP, MR] = ...
                    perfFunc(data.xtrain, data.ytrain, data.xtest, data.ytest, hyperParameters, 0.5);
                save(path,'searchInfo','hyperParameters','accuracy',...
                    'risk', 'classifier', 'falsePos', 'falseNeg', 'shownWork', 'MP_grid', 'MR_grid', 'MP', 'MR');
            else
                save(path,'searchInfo','hyperParameters','shownWork', 'MP_grid', 'MR_grid');
            end
        end
        
        function [hyperParameters] = retrieve_hyperparameters(data, performanceMeasure)
            searchTypes = [{'grid'}, {'line'}, {'jaak'}];
            found = 0;
            for type = searchTypes
                path = ExperimentBase.generate_read_file_path('hypersearch', ...
                    'results', data.name, performanceMeasure, cell2mat(type));
                if exist([path '.mat'],'file')
                    load(path, 'hyperParameters');
                    found = 1;
                    break;
                end
            end
            if ~found
                error('Parameters have not been trained for %s with %s',...
                    data.name, performanceMeasure);
            end
        end
        
        function [accuracy, falsePos, falseNeg] = retrieve_performance(data, performanceMeasure)
            searchTypes = [{'grid'}, {'line'}, {'jaak'}];
            found = 0;
            for type = searchTypes
                path = ExperimentBase.generate_read_file_path('hypersearch', ...
                    'results', data.name, performanceMeasure, cell2mat(type));
                if exist([path '.mat'],'file')
                    load(path);
                    if exist('accuracy') && exist('falsePos') && exist('falseNeg')
                        found = 1;
                    end
                    break;
                end
            end
            if ~found
                error('Performance has been measured for %s with %s',...
                    data.name, performanceMeasure);
            end
        end
        
        function [results] = collect_results()
            [results] = ExperimentBase.collect_results(...
                '\hypersearch\results', @ExperimentHyperParameterSearch.read_result);
        end
        
        function [result] = read_result(path, experimentId)
            r = load(path);
            if ~isfield(r, 'accuracy')
                r.accuracy = '';
                r.falsePos = '';
                r.falseNeg = '';
            end
            result = [{experimentId}, r.hyperParameters, {r.accuracy},...
                {r.falsePos}, {r.falseNeg}, r.searchInfo];
        end
        
        function [] = visualize(obj)
            addpath('.\Utility');
            visualize(obj.shownWork);
        end
        
        function [] = manual(data, performanceType, hyperParameters)
            [perfFunc, paramSpace, startPoint, searchInfo] = ...
                ExperimentHyperParameterSearch.set_performance_function(performanceType, 'manual');

            searchname = 'manual';
            for ii = 1:length(hyperParameters)
                searchname = sprintf('%s-%f', searchname, hyperParameters(ii));
            end
            % Replace period with comma to not confuse file system
            searchname = strrep(searchname, '.',',');
            path = ExperimentBase.generate_write_file_path('hypersearch', ...
                'results', data.name, performanceType, searchname);
            
            [ risk, accuracy, classifier, falsePos, falseNeg] = ...
                perfFunc(data.xtrain, data.ytrain, data.xtest, data.ytest, hyperParameters, 0.5);
            save(path,'searchInfo','hyperParameters',...
                'accuracy', 'risk', 'classifier', 'falsePos', 'falseNeg' );
        end
    end
    
    methods (Access = private, Static)
        function [perfFunc, paramSpace, startPoint, searchInfo] = ...
                set_performance_function(performanceType, searchType)
        % set_performance_function helps keep all performance function
        % specific options in one place
            switch lower(performanceType)
                case 'linearsvm'
                    lambdaN = 20;
                    lambdaBounds = [-3 3];
                    perfFunc = @LinearPerformance;
                    paramSpace = {logspace(lambdaBounds(1),lambdaBounds(2),lambdaN)};
                    startPoint = 1;
                    searchInfo = ...
                        sprintf('%s, Lambda: %d x [%d, %d], using %s',...
                        performanceType, lambdaN, lambdaBounds(1),lambdaBounds(2), searchType);
                case 'gaussiansvm'
                    lambdaN = 20;
                    sigmaN = 20;
                    lambdaBounds = [-3 3];
                    sigmaBounds = [-2 2];
                    perfFunc = @Performance;
                    paramSpace = [...
                        {logspace(lambdaBounds(1),lambdaBounds(2),lambdaN)};
                        {logspace(sigmaBounds(1), sigmaBounds(2), sigmaN)}];
                    % Note this is updated later with the Jaakola heuristic
                    startPoint = mean(cell2mat(paramSpace),2);
                    searchInfo = ...
                        sprintf('%s, Lambda: %d x [%d, %d], Sigma %d x [%d, %d], using %s',...
                        performanceType, lambdaN, lambdaBounds(1),...
                        lambdaBounds(2), sigmaN, sigmaBounds(1), sigmaBounds(2), searchType);
                case 'multiclasssvm'
                    lambdaN = 10;
                    lambdaBounds = [-5 -3];
                    perfFunc = @MultiClassPerformance;
                    paramSpace = {logspace(lambdaBounds(1),lambdaBounds(2),lambdaN)};
                    startPoint = 1;
                    searchInfo = ...
                        sprintf('%s, Lambda: %d x [%d, %d], using %s',...
                        performanceType, lambdaN, lambdaBounds(1),lambdaBounds(2), searchType);
                otherwise
                    error('Unknown performance measure: %s', performanceType);
            end
        end
    end
end

