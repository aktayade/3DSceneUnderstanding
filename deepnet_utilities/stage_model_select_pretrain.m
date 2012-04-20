function [ ] = stage_model_select_pretrain( taskName, stayActive, target )
%STAGE_MODEL_SELECT_PRETRAIN Pretrains a MLP trying various values
%  stayActive indicates if this module should continue looking for work.
%  When target is the name of the data file to load then this function
%  pretrains  only runs for that target then exits
% Stage task parameters:
%   hiddenLayers - matrix representing the hidden layer counts in the MLP
%   binaryInput - true if data is in the range [0,1]. False otherwise.
%   useRBMs - true to use RBMs instead of autoencoder networks
%   modelSelectDensity - [opt] determines how many search values are tried
%   modelSelectKFolds - [opt] how many folds to use in model selection CV
%   
    global_paths;
    
    if ~exist('taskName', 'var')
        taskName = '';
    end
    if ~exist('stayActive','var')
        stayActive = false;
    end
    if ~exist('target', 'var')
        target = '';
    end
    
    inputFrom = {'tasks', 'output', 'taskData'};
    outputTo = {'tasks', 'output', 'pretrained'};
    intermediate = {'tasks', 'intermediate', 'pretrained'};
    
    find_work(@work, taskName, stayActive, target, inputFrom, {}, {intermediate, outputTo});
end

function work(paths, ~)
    progress = 0;
    save(paths{2}, 'progress');
    load(paths{1});

    [mlp] = pretrain(xtrain, ytrain, unlabeled, masterTaskName);
    save(paths{3}, 'mlp', 'masterTaskName');
end

function [mlp] = pretrain(xtrain, ytrain, unlabeled, taskName)
    task = read_task_parameters(taskName);
    if ~isfield(task, 'modelSelectDensity')
        task.modelSelectDensity = 10;
    end
    if ~isfield(task, 'modelSelectKFolds')
        task.modelSelectKFolds = 4;
    end
    if ~isfield(task, 'hiddenLayers')
        error('Must define task parameter hiddenLayers');
    end
    if ~isfield(task, 'binaryInput')
        error('Must define task parameter binaryInput');
    end
    if ~isfield(task, 'useRBMs')
        error('Must define task parameter useRBMs');
    end

    mlp = MultiLayerPerceptron(taskName, task.hiddenLayers, task.numClasses, task.binaryInput, task.useRBMs, true);
    
    for l = 1:length(mlp.layers)
        if ~mlp.layers{l}.pretrained
            % Model selection:
            cvIndices = crossvalind('Kfold', ytrain, task.modelSelectKFolds);
            [searchSpace, startPoint] = mlp.layers{l}.GetModelSelectionSearchSpace(task.modelSelectDensity);

            % Speed up model selection of gaussian RBM by only doing kmeans
            % once per fold instead of for each parameter also
            kmeans_label = cell(task.modelSelectKFolds,1);
            kmeans_center = cell(task.modelSelectKFolds,1);
            if isa(mlp.layers{l}, 'LayerGaussianRBM')
                parfor k = 1:task.modelSelectKFolds
                    [kmeans_label{k},kmeans_center{k}] = kmeans([xtrain(:,cvIndices ~= k) unlabeled]', mlp.layers{l}.hiddenSize, 'Replicates', 1, 'EmptyAction', 'singleton');
                end
            end

            [optimalParameters, work] = line_search( startPoint, searchSpace, @mlp_performance, '', false, xtrain, ytrain, unlabeled, cvIndices, mlp.layers{l}, kmeans_label, kmeans_center);
            mlp.layers{l}.optimalParameters = optimalParameters;
            mlp.layers{l}.modelSelectionWork = work;

            % Train the layer with optimal parameters and all data
            mlp.layers{l}.Pretrain([xtrain unlabeled], optimalParameters(1), optimalParameters(2), optimalParameters(3), false);
            mlp.layers{l}.pretrained = true;
            mlp.SaveNetwork();
        end
        xtrain = mlp.layers{l}.FeedForward(xtrain);
        unlabeled = mlp.layers{l}.FeedForward(unlabeled);
    end

    % Save the network
    mlp.networkTrainingStage = 1;
    mlp.SaveNetwork();
end

function [result] = mlp_performance(params, xtrain, ytrain, unlabeled, cvIndices, masterLayerObj, kmeans_label, kmeans_center)
% This function assumes the data given has been transformed up
% to the layer that needs selection.  It trains the layer given
% the test parameters then trains a softmax classifier and
% tests performance, returning this performance as a result
%
    layerObj = masterLayerObj.Clone();
    kagg = zeros(max(cvIndices),1);
    for k = 1:max(cvIndices)
        test = (k == cvIndices);
        train = ~test;
        
        % Use existing kmeans result to speed up Gaussian RBM
        if isa(layerObj, 'LayerGaussianRBM')
            layerObj.SetKMeansClusters(kmeans_label{k}, kmeans_center{k});
            layerObj.Pretrain([xtrain(:,train) unlabeled], params(1), params(2), params(3), true);
            layerObj.SetKMeansClusters();
        else
            layerObj.Pretrain([xtrain(:,train) unlabeled], params(1), params(2), params(3), true);
        end

        transformed_xtrain = layerObj.FeedForward(xtrain);

        % Train a softmax classifier to test performance
        lambda = 1e-3;
        maxIter = 200;
        nclasses = max(ytrain);
        softmaxObj = softmaxClassifierTrain(nclasses, lambda, transformed_xtrain(:,train), ytrain(train), maxIter);
        predictions = softmaxClassifierPredict(softmaxObj, transformed_xtrain(:,test));
        ytest = ytrain(test);
        kagg(k) = mean(ytest(:) == predictions(:));
    end
    result.score = mean(kagg);
end


