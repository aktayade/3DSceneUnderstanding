classdef LayerBinaryAutoencoder < LayerBase
    %LAYERBINARYAUTOENCODER Represents a layer of deterministic binary
    %   units in the network whose input is [0,1] and output is [0,1]
    
    properties (Constant)
        defaultSparsity = 0.01;
        defaultLambda = 0.0001;
        defaultBeta = 3;
        quickPretrainMaxIter = 200;
        fullPretrainMaxIter = 400;
    end
    
    methods
        function obj = LayerBinaryAutoencoder(hiddenSize)
            obj.hiddenSize = hiddenSize;
            obj.W = [];
            obj.b = [];
            obj.pretrained = false;
            obj.optimalParameters = [];
            obj.modelSelectionWork = [];
        end
        
        function new = Clone(obj)
            new = LayerBinaryAutoencoder(obj.hiddenSize);
            new.W = obj.W;
            new.b = obj.b;
            new.pretrained = obj.pretrained;
            new.optimalParameters = obj.optimalParameters;
            new.modelSelectionWork = obj.modelSelectionWork;
        end
        
        function obj = Pretrain(obj, data, sparsityParam, lambda, beta, quick)
            maxIter = obj.quickPretrainMaxIter*quick + obj.fullPretrainMaxIter*~quick;
            [obj.W, obj.b] = pretrainBinarySparseAutoencoder(data, obj.hiddenSize, sparsityParam, lambda, beta, maxIter);
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