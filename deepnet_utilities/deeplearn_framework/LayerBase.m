classdef LayerBase < handle
    %LAYERBASE Base class representing a layer of the network
    
    properties
        hiddenSize
        W
        b
        pretrained
        optimalParameters
        modelSelectionWork
    end
    
    methods (Abstract)
        obj = Pretrain(obj, data, sparsityParam, lambda, beta, quick)
        transformed = FeedForward(obj, data)
        da = BackpropDerivitive(obj, activations)
        searchSpace = GetModelSelectionSearchSpace(N)
        new = Clone(obj)
    end
    
end