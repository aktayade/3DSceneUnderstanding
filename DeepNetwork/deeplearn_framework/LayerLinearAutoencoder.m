classdef LayerLinearAutoencoder < LayerBase
    %LAYERLINEARAUTOENCODER Represents a layer of deterministic linear
    %   units in the network whose input is continuous and output is binary
    
    properties (Constant)
        defaultSparsity = 0.035;
        defaultLambda = 3e-3;
        defaultBeta = 5;
        quickPretrainMaxIter = 200;
        fullPretrainMaxIter = 400;
    end
    
    methods
        function obj = LayerLinearAutoencoder(hiddenSize)
            obj.hiddenSize = hiddenSize;
            obj.W = [];
            obj.b = [];
            obj.pretrained = false;
            obj.optimalParameters = [];
            obj.modelSelectionWork = [];
            obj.quickModelSelect = false;
        end
        
        function obj = Pretrain(obj, data, sparsityParam, lambda, beta, quick)
            maxIter = obj.quickPretrainMaxIter*quick + obj.fullPretrainMaxIter*~quick;
            [obj.W, obj.b] = pretrainLinearSparseAutoencoder(data, obj.hiddenSize, sparsityParam, lambda, beta, maxIter);
            obj.pretrained = true;
        end
        
        function transformed = FeedForward(obj, data)
            transformed = sigmoid(obj.W * data + repmat(obj.b,1,size(data,2)));
        end
        
        function da = BackpropDerivitive(~, activations)
            da = activations .* (1-activations);
        end
        
        function [paramSpace startPoint] = GetModelSelectionSearchSpace(obj, N)
            paramSpace = [... % Sparsity, Lambda, Beta
                {linspace(1/obj.hiddenSize, 30/obj.hiddenSize, N)};
                {obj.defaultLambda};
                {obj.defaultBeta}];
            startPoint = zeros(size(paramSpace));
            for d = 1:length(paramSpace)
                startPoint(d) = mean(paramSpace{d},2);
            end
        end
    end
end