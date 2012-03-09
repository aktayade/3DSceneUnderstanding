classdef Dataset < handle
    %EXPERIMENTBASE Base class to hold loaded data from a dataset
    
    properties
        dataset
        dataSubset
        dataSetarg
        name
        truePi
        xtrain
        ytrain
        xtest
        ytest
    end
    
    methods
        function obj = Dataset( dataset, subset, setarg, standardize, setPi )
            obj.dataset = dataset;
            obj.dataSubset = subset;
            obj.dataSetarg = setarg;

            if nargin < 4
                standardize = true;
            end
            
            globalpaths;

            if strcmpi('scene', dataset)
                if nargin < 3
                    setarg = 0;
                end
                [obj.xtrain, obj.ytrain, obj.xtest, obj.ytest, obj.name] = ...
                    load_scene_dataset(0, 0, subset, setarg);
            end
            
            if nargin >= 5
                [obj.ytrain ~] = Dataset.contaminate_positives(obj.ytrain, setPi);
                [obj.ytest obj.truePi] = Dataset.contaminate_positives(obj.ytest, setPi);
                obj.name = strrep(sprintf('%s-pi%f', obj.name, obj.truePi), '.', ',');
            else
                obj.truePi = -1;
            end
            
            if standardize
                [obj.xtrain, dataMean, dataStd] = Dataset.standardize_data(obj.xtrain);
                [obj.xtest] = Dataset.standardize_data(obj.xtest, dataMean, dataStd);
            end
        end
    end
    
    methods (Static)
        function [data,dataMean,dataStd] = standardize_data(data,dataMean,dataStd)
        % STANDARDIZE_DATA Will normalize and center a dataset.
        %   Optionally, give it a mean and std to use to standardize the
        %   data.
            if (nargin < 2)
                dataMean = mean(data);
                dataStd = std(data);
            end
            [n,~] = size(data);
            data = (data - repmat(dataMean, n, 1)) ./ repmat(dataStd, n, 1);
            data(isnan(data)) = 0;
        end
        
        function [labels truePi] = contaminate_positives(labels, piVal)
        % CONTAMINATE_POSITIVES Will mix negative examples into positives
        %   After mixing pos will contain piVal percent true positives and 
        %   1-piVal percent false positives.  Since mixing is discretized
        %   the actual truePi value is also calculated and returned
            oldPosTotal = sum(labels == 1);
            newPosTotal = floor(oldPosTotal/piVal);
            truePi = oldPosTotal / newPosTotal;
            negMoveCount = newPosTotal - oldPosTotal;

            neg = find(labels ~= 1);
            perm = randperm(length(neg));
            labels(neg(perm(1:negMoveCount))) = 1;
            
        end
    end
   
        
end
