function [ ] = stage_finetune( taskName, stayActive, target )
%STAGE_FINETUNE Finetunes a MLP
%  stayActive indicates if this module should continue looking for work.
%  When target is the name of the data file to load then this function
%  finetunes  only runs for that target then exits
% Stage task parameters:
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
    
    inputFrom = {'tasks', 'output', 'pretrained'};
    outputTo = {'tasks', 'output', 'finetuned'};
    intermediate = {'tasks', 'intermediate', 'finetuned'};
    rawData = {'tasks', 'output', 'taskData'};
    
    find_work(@work, taskName, stayActive, target, inputFrom, {rawData}, {intermediate, outputTo});
end

function work(paths, ~)
    lambdaN = 5;
    lambdaSpace = logspace(-4, 1, lambdaN);
    progress = 0;
    save(paths{3}, 'progress');
    
    load(paths{1});
    load(paths{2});
    task = read_task_parameters(masterTaskName);

    [~, work] = grid_search({1:length(mlp.layers); lambdaSpace}, @finetune_performance, '', false, xtrain, ytrain, xtest, ytest, mlp, task.numClasses);
        
    bestMlps = cell(length(mlp.layers),1);
    scores = retrieve_cellstruct_field(work, 'score');
    for lcount = 1:length(mlp.layers)
        [~, c] = max(scores(lcount, :));
        bestMlps{lcount} = work{lcount, c}.mlp;
    end

    save(paths{4}, 'work', 'bestMlps', 'masterTaskName');
end

function [result] = finetune_performance(params, xtrain, ytrain, xtest, ytest, masterMlp, numClasses)
% Performs finetuning and returns cross validated performance
%       
    mlp = masterMlp.Clone();
    mlp.allowSaving = false;
    mlp.layers = mlp.layers(1:params(1));

    mlp.TrainSoftmax(xtrain, ytrain, params(2));
    pred = mlp.Predict(xtest);
    [result.pftAcc, result.pftMP, result.pftMR] = perf_stats(ytest, pred, numClasses);

    mlp.FineTune(xtrain, ytrain, params(2), false);
    pred = mlp.Predict(xtest);
    [result.ftAcc, result.ftMP, result.ftMR] = perf_stats(ytest, pred, numClasses);
    
    result.mlp = mlp;
    result.score = result.ftAcc;
end


