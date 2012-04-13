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
    end
    
    properties (Constant)
        quickFineTuneMaxIter = 100;
        fullFineTuneMaxIter = 400;
    end
    
    methods
        function obj = MultiLayerPerceptron(name, hiddenLayers, numClasses, binaryInput, useRBMs)
            % name is the name of the network (for saving/loading)
            % hiddenLayers is a vector with each hiddenLayer size
            % numClasses is the total number of labels in the test data
            % binaryInput is true if the first layer is [0,1], else false
            % useRBMs is true if RBMs should be used for pretraining
            %   instead of deterministic autoencoders
            filename = ['SavedNetworks/' name '.mat'];
            if exist(filename, 'file')
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
        end
        
        function obj = DoEverything(obj, trainingData, trainingLabels, quick)
            % Automatically pretrains with model selection, trains softmax,
            % and finetunes the network.
            obj.PretrainModelSelection(trainingData, trainingLabels, quick);
            obj.TrainSoftmax(trainingData, trainingLabels);
            obj.FineTune(trainingData, trainingLabels, quick);
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
        
        function obj = PretrainModelSelection(obj, pretrainData, pretrainLabels, quick)
            % Performs model selection on each layer to choose
            % hyperparameters and pretrains the network simultaniously
            % Model selection is based on softmax classifier performance
            % NOTE: If pretrainLabels is shorter than
            %   validationPercentage*length(pretrainData) then all
            %   examples which do have labels are used for validation
            %   (holdout cross validation) otherwise validationPercentage
            %   percent of the pretrainData is used for validation.
            %   It is expected that the data with labels are the first
            %   examples in pretrainData.
            % Ultimately the layer is pretrained using all pretrainData
            
            %{
            % Unused code, but would support unlabeled data, if labels were
            % shorter than the data matrix
            if ~exist('validationPercentage', 'var')
                validationPercentage = .3;
            end
            validationCount = floor(size(pretrainData, 2)*validationPercentage);
            holdoutPercentage = validationCount / length(pretrainLabels);
            if holdoutPercentage < 1
                valInd = crossvalind('HoldOut', pretrainLabels, 1-holdoutPercentage);
            else
                valInd = true(length(pretrainLabels),1);
            end
            validationLabels = pretrainLabels(valInd);
            valInd = [valInd;false(size(pretrainData,2)-length(valInd),1)];
            validationData = pretrainData(:,valInd);
            trainData = pretrainData(:,~valInd);
            %}
            
            % For each layer, model select and pretrain
            K = 5;
            if quick
                SearchN = 3;
            else
                SearchN = 15;
            end
            transformedData = pretrainData;
            for l = 1:length(obj.layers)
                if ~obj.layers{l}.pretrained
                    % Model selection:
                    obj.layers{l}.quickModelSelect = quick;
                    cvIndices = crossvalind('Kfold', pretrainLabels, K);
                    [searchSpace, startPoint] = obj.layers{l}.GetModelSelectionSearchSpace(SearchN);
                    [optimalParameters, work] = lineSearch( startPoint, transformedData,...
                        pretrainLabels, searchSpace, cvIndices,...
                        @MultiLayerPerceptron.ModelSelectionSoftmaxPerformance, obj.layers{l});
                    obj.layers{l}.optimalParameters = optimalParameters;
                    obj.layers{l}.modelSelectionWork = work;
                    
                    % Train the layer with optimal parameters and all data
                    obj.layers{l}.Pretrain(transformedData, optimalParameters(1), optimalParameters(2), optimalParameters(3), false);
                    obj.layers{l}.pretrained = true;
                    obj.SaveNetwork();
                end
                transformedData = obj.layers{l}.FeedForward(transformedData);
            end
            
            % Save the network
            obj.networkTrainingStage = 1;
            obj.SaveNetwork();
        end
        
        function obj = TrainSoftmax(obj, trainingData, trainingLabels, lambda)
            % After pretraining this trains the softmax classifier
            assert(obj.networkTrainingStage >= 1, 'Must pretrain first');
            
            if obj.networkTrainingStage >= 2
                return;
            end
            
            if ~exist('lambda', 'var')
                lambda = 1e-4;
            end
            
            transformedData = obj.TransformData(trainingData);
            
            maxIter = 400;
            obj.softmaxClassifier = softmaxClassifierTrain(obj.numClasses, lambda, transformedData, trainingLabels, maxIter);
            obj.networkTrainingStage = 2;
            obj.SaveNetwork();
        end
        
        function obj = FineTune(obj, trainingData, trainingLabels, quick)
            % After pretraining, uses backpropagation to fine-tune weights
            assert(obj.networkTrainingStage >= 1, 'Must pretrain first');
            if obj.networkTrainingStage == 1
                obj.TrainSoftmax(trainingData, trainingLabels);
            end
            maxIter = obj.quickFineTuneMaxIter*quick + obj.fullFineTuneMaxIter*~quick;
            
            [obj.layers, obj.softmaxClassifier] = fineTuneStackedAutoencoder(...
                obj.layers, obj.softmaxClassifier, obj.numClasses, trainingData, trainingLabels, maxIter);
            
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
            fpath = 'SavedNetworks';
            if ~exist(fpath,'dir')
                mkdir(fpath);
            end
            save([fpath '/' obj.networkName '.mat'], 'obj');
        end
    end
    
    methods (Static)
        function [error] = ModelSelectionSoftmaxPerformance(xtrain, ytrain, xtest, ytest, paramsLocal, layerObj)
            % This function assumes the data given has been transformed up
            % to the layer that needs selection.  It trains the layer given
            % the test parameters then trains a softmax classifier and
            % tests performance, returning this performance as a result
            %
            % Pretrain and transform the data
            sparsityParam = paramsLocal(1);
            lambda = paramsLocal(2);
            beta = paramsLocal(3);
            layerObj.Pretrain(xtrain, sparsityParam, lambda, beta, layerObj.quickModelSelect);
            transformed_xtrain = layerObj.FeedForward(xtrain);
            transformed_xtest = layerObj.FeedForward(xtest);
            
            % Train a softmax classifier to test performance
            lambda = 1e-4;
            maxIter = 100;
            nclasses = max(max(ytrain),max(ytest));
            softmaxObj = softmaxClassifierTrain(nclasses, lambda, transformed_xtrain, ytrain, maxIter);
            predictions = softmaxClassifierPredict(softmaxObj, transformed_xtest);
            error = 1-mean(ytest(:) == predictions(:));
        end
    end
end

