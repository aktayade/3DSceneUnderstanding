classdef MlpParameters
    %MLPPARAMETERS Parameters for creating and testing an mlp
    %   This aids the process of passing parameters to a function which
    %   creates the mlp
    
    properties
        mlpName
        binaryInput
        useRBMs
        hiddenLayers
        numClasses
        quick
        automated
        msUseGreedy
        msUseLineSearch
        msSearchDensity
        useModelSelection
        sparsityParameters
        lambdaParameters
        betaParameters
    end
    
    methods
        function obj = MlpParameters(mlpName, binaryInput, numClasses, useRBMs, hiddenLayers, quick, automated)
            obj.mlpName = mlpName;
            obj.binaryInput = binaryInput;
            obj.useRBMs = useRBMs;
            obj.hiddenLayers = hiddenLayers;
            obj.numClasses = numClasses;
            if ~exist('quick', 'var')
                quick = false;
            end
            obj.quick = quick;
            if ~exist('automated', 'var')
                automated = false;
            end
            obj.automated = automated;
            obj.useModelSelection = true;
            obj.msUseGreedy = true;
            obj.msUseLineSearch = true;
            obj.msSearchDensity = 10;
        end
        
        function obj = SetModelSelectionParameters(obj, useGreedy, useLineSearch, searchDensity)
            % Greedy model selection will model select one layer at a time
            % Non-Greedy will select for all layers at a time
            % LineSearch is a heuristic search process (faster)
            % searchDensity is the granularity of the search
            obj.msUseGreedy = useGreedy;
            obj.msUseLineSearch = useLineSearch;
            obj.msSearchDensity = searchDensity;
        end
        
        function obj = OverrideModelSelection(obj, sparsityParameters, lambdaParameters, betaParameters)
            % sparsityParameters - Controls hidden unit activity
            % lambdaParameters - L2 regularization term
            % betaParameters - Weight of the sparsity parameter
            if ~exist('sparsityParameters','var')
                obj.sparsityParameters = 10./obj.hiddenLayers;
            else
                obj.sparsityParameters = sparsityParameters;
            end
            if ~exist('lambdaParameters','var')
                obj.lambdaParameters = 3e-3*ones(size(obj.hiddenLayers));
            else
                obj.lambdaParameters = lambdaParameters;
            end
            if ~exist('betaParameters','var')
                if obj.useRBMs
                    obj.betaParameters = 10*ones(size(obj.hiddenLayers));
                else
                    obj.betaParameters = 3*ones(size(obj.hiddenLayers));
                end
            else
                obj.betaParameters = betaParameters;
            end
            obj.useModelSelection = false;
        end
        
    end
    
end

