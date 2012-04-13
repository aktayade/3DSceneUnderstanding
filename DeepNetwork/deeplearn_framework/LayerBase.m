classdef LayerBase < handle
    %LAYERBASE Base class representing a layer of the network
    
    properties
        hiddenSize
        W
        b
        pretrained
        optimalParameters
        modelSelectionWork
        quickModelSelect
    end
    
    methods (Abstract)
        obj = Pretrain(obj, data, sparsityParam, lambda, beta, quick)
        transformed = FeedForward(obj, data)
        da = BackpropDerivitive(obj, activations)
        searchSpace = GetModelSelectionSearchSpace(N)
    end
    
end