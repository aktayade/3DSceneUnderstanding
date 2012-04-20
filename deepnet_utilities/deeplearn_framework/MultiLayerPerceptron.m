classdef MultiLayerPerceptron < handle
    %MULTILAYERPERCEPTRON Represents a MLP
    %   Capabilities:
    %       -Pretrain with RBM or autoencoders
    %       -Transform data for use with any classifier
    %       -Classify data with softmax classifier
    %       -Model selected building of each layer
    
    properties        
        % 0 = start, 1 = pretrained, 2 = softmax, 3 = finetuned
        networkTrainingStage
        layers
        numClasses
        softmaxClassifier
        networkName
        allowSaving
    end
    
    properties (Constant)
        quickFineTuneMaxIter = 100;
        fullFineTuneMaxIter = 400;
    end
    
    methods
        function obj = MultiLayerPerceptron(name, hiddenLayers, numClasses, binaryInput, useRBMs, noLoadExisting)
            % name is the name of the network (for saving/loading)
            % hiddenLayers is a vector with each hiddenLayer size
            % numClasses is the total number of labels in the test data
            % binaryInput is true if the first layer is [0,1], else false
            % useRBMs is true if RBMs should be used for pretraining
            %   instead of deterministic autoencoders
            % noLoadExisting is false to allow loading existing net
            if strcmp('__clone__', name)
                obj.networkTrainingStage = 0;
                obj.softmaxClassifier = struct;
                return;
            end
            
            if ~exist('noLoadExisting', 'var')
                noLoadExisting = false;
            end
            filename = ['SavedNetworks/' name '.mat'];
            if exist(filename, 'file') && ~noLoadExisting
                a = input('Load existing network?\n', 's');
                if lower(a(1)) == 'y'
                    load(filename);
                    return;
                end
            end
            
            % Build the network
            for l = 1:length(hiddenLayers)
                if l == 1 && ~binaryInput && useRBMs
                    obj.layers{l} = LayerGaussianRBM(hiddenLayers(l));
                elseif l == 1 && ~binaryInput && ~useRBMs
                    obj.layers{l} = LayerLinearAutoencoder(hiddenLayers(l));
                elseif useRBMs
                    obj.layers{l} = LayerBernoulliRBM(hiddenLayers(l));
                else
                    obj.layers{l} = LayerBinaryAutoencoder(hiddenLayers(l));
                end
            end
            obj.numClasses = numClasses;
            obj.networkTrainingStage = 0;
            obj.networkName = name;
            obj.softmaxClassifier = struct;
            obj.allowSaving = true;
        end
        
        function new = Clone(obj)
            new = MultiLayerPerceptron('__clone__');
            for l = 1:length(obj.layers)
                new.layers{l} = obj.layers{l}.Clone();
            end
            new.numClasses = obj.numClasses;
            new.networkTrainingStage = obj.networkTrainingStage;
            new.networkName = obj.networkName;
            new.softmaxClassifier = obj.softmaxClassifier;
            new.allowSaving = obj.allowSaving;
        end
        
        function obj = Pretrain(obj, pretrainData, quick, sparsityParameters, lambdaParameters, betaParameters)
            % Pretrains the layers given specific hyperparameters
            % or using defaults if not specified
            %
            % Set default parameters
            if ~exist('sparsityParameters', 'var')
                sparsityParameters = zeros(length(obj.layers),1);
                for l = 1:length(obj.layers)
                    sparsityParameters(l) = obj.layers{l}.defaultSparsity;
                end
            end
            if ~exist('lambdaParameters', 'var')
                lambdaParameters = zeros(length(obj.layers),1);
                for l = 1:length(obj.layers)
                    lambdaParameters(l) = obj.layers{l}.defaultLambda;
                end
            end
            if ~exist('betaParameters', 'var')
                betaParameters = zeros(length(obj.layers),1);
                for l = 1:length(obj.layers)
                    betaParameters(l) = obj.layers{l}.defaultBeta;
                end
            end
            
            % Train each layer with given parameters
            transformedPretrainData = pretrainData;
            for l = 1:length(obj.layers)
                if ~obj.layers{l}.pretrained
                    obj.layers{l}.Pretrain(transformedPretrainData, sparsityParameters(l), lambdaParameters(l), betaParameters(l), quick);
                    obj.layers{l}.pretrained = true;
                    obj.SaveNetwork();
                end
                transformedPretrainData = obj.layers{l}.FeedForward(transformedPretrainData);
            end
            
            % Save the network
            obj.networkTrainingStage = 1;
            obj.SaveNetwork();
        end
        
        function obj = TrainSoftmax(obj, trainingData, trainingLabels, lambda)
            % After pretraining this trains the softmax classifier
            assert(obj.networkTrainingStage >= 1, 'Must pretrain first');
            
            if ~exist('lambda', 'var')
                lambda = 1e-4;
            end
            
            transformedData = obj.TransformData(trainingData);
            
            maxIter = 400;
            obj.softmaxClassifier = softmaxClassifierTrain(obj.numClasses, lambda, transformedData, trainingLabels, maxIter);
            obj.networkTrainingStage = 2;
            obj.SaveNetwork();
        end
        
        function obj = FineTune(obj, trainingData, trainingLabels, lambda, quick)
            % After pretraining, uses backpropagation to fine-tune weights
            assert(obj.networkTrainingStage >= 1, 'Must pretrain first');
            if obj.networkTrainingStage == 1
                obj.TrainSoftmax(trainingData, trainingLabels);
            end
            maxIter = obj.quickFineTuneMaxIter*quick + obj.fullFineTuneMaxIter*~quick;
            
            [obj.layers, obj.softmaxClassifier] = fineTuneStackedAutoencoder(...
                obj.layers, obj.softmaxClassifier, obj.numClasses, trainingData, trainingLabels, lambda, maxIter);
            
            obj.networkTrainingStage = 3;
            obj.SaveNetwork();
        end
        
        function transformed = TransformData(obj, data)
            % When used with a trained network transforms data with the net
            assert(obj.networkTrainingStage >= 1, 'Must at least pretrain');
            transformed = data;
            for l = 1:length(obj.layers)
                transformed = obj.layers{l}.FeedForward(transformed);
            end
        end
        
        function predictions = Predict(obj, data)
            % When used with a trained network predicts labels with softmax
            assert(obj.networkTrainingStage >= 2, 'Softmax not trained yet');
            transformedData = obj.TransformData(data);
            predictions = softmaxClassifierPredict(obj.softmaxClassifier, transformedData);
        end
        
        function [] = SaveNetwork(obj)
            if ~obj.allowSaving
                return;
            end
            fpath = 'SavedNetworks';
            if ~exist(fpath,'dir')
                mkdir(fpath);
            end
            save([fpath '/' obj.networkName '.mat'], 'obj');
        end
    end
    
    methods (Static)
        function mlp = CreateFromStruct(mlpParams)
            mlp = MultiLayerPerceptron(mlpParams.mlpName,...
                mlpParams.hiddenLayers, mlpParams.numClasses,...
                mlpParams.binaryInput, mlpParams.useRBMs, mlpParams.automated);
        end
    end
end

