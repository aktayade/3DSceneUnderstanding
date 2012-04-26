classdef LayerPoissonRBM < LayerBase
    %LAYERPOISSONRBM Represents a layer of stochastic poisson
    %   units in the network whose input is continuous and output is binary
    
    properties (Constant)
        defaultSparsity = 0.02;
        defaultLambda = 0.001;
        defaultBeta = 10;
        quickPretrainMaxIter = 250;
        fullPretrainMaxIter = 250;
    end
    
    methods
        function obj = LayerPoissonRBM(hiddenSize)
            obj.hiddenSize = hiddenSize;
            obj.W = [];
            obj.b = [];
            obj.pretrained = false;
            obj.optimalParameters = [];
            obj.modelSelectionWork = [];
        end
        
        function new = Clone(obj)
            new = LayerPoissonRBM(obj.hiddenSize);
            new.W = obj.W;
            new.b = obj.b;
            new.pretrained = obj.pretrained;
            new.optimalParameters = obj.optimalParameters;
            new.modelSelectionWork = obj.modelSelectionWork;
        end
        
        function obj = Pretrain(obj, data, sparsityParam, lambda, beta, quick)
            maxIter = obj.quickPretrainMaxIter*quick + obj.fullPretrainMaxIter*~quick;
            [obj.W, obj.b] = rbm_train_PB_hinton(data, obj.hiddenSize, sparsityParam, lambda, beta, maxIter);
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
            startPoint = zeros(size(paramSpace))';
            for d = 1:length(paramSpace)
                startPoint(d) = mean(paramSpace{d},2);
            end
        end
    end
end