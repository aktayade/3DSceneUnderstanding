function [ ] = stage_external_classify( taskName, stayActive, target )
%STAGE_EXTERNAL_CLASSIFY Classifies raw data and transformed data
%  using various classifiers.
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
    
    while true
        outputTo = {'tasks', 'output', 'externalraw'};
        intermediate = {'tasks', 'intermediate', 'externalraw'};
        rawData = {'tasks', 'output', 'taskData'};
        find_work(@work_raw, taskName, false, target, rawData, {}, {outputTo, intermediate});
        
        outputTo = {'tasks', 'output', 'externalfinetuned'};
        intermediate = {'tasks', 'intermediate', 'externalfinetuned'};
        inputFrom = {'tasks', 'output', 'finetuned'};
        inputFrom2 = {'tasks', 'output', 'pretrained'};
        rawData = {'tasks', 'output', 'taskData'};
        find_work(@work_finetuned, taskName, false, target, rawData, {inputFrom, inputFrom2}, {outputTo, intermediate});
        
        if ~stayActive
            break;
        end
        
        pause(30);
    end
end

function work_raw(paths, ~)
    progress = 0;
    save(paths{3}, 'progress');
    load(paths{1});
    task = read_task_parameters(masterTaskName);
    [gsvm, lsvm, msvm, sm, parameters] = external_classifiers_performance(xtrain, ytrain, xtest, ytest, task.numClasses);
    save(paths{2}, 'gsvm', 'lsvm', 'msvm', 'sm', 'parameters', 'masterTaskName');
end

function work_finetuned(paths, ~)
    progress = 0;
    save(paths{5}, 'progress');
    load(paths{1});
    task = read_task_parameters(masterTaskName);
    
    load(paths{2});
    ftgsvm = cell(length(bestMlps),1);
    ftlsvm = cell(length(bestMlps),1);
    ftmsvm = cell(length(bestMlps),1);
    ftsm = cell(length(bestMlps),1);
    ftparameters = cell(length(bestMlps),1);
    for ii = 1:length(bestMlps)
        t_xtrain = bestMlps{ii}.TransformData(xtrain);
        t_xtest = bestMlps{ii}.TransformData(xtest);
        [ftgsvm{ii}, ftlsvm{ii}, ftmsvm{ii}, ftsm{ii}, ftparameters{ii}] = external_classifiers_performance(t_xtrain, ytrain, t_xtest, ytest, task.numClasses);
    end
    
    load(paths{3});
    pftgsvm = cell(length(bestMlps),1);
    pftlsvm = cell(length(bestMlps),1);
    pftmsvm = cell(length(bestMlps),1);
    pftsm = cell(length(bestMlps),1);
    pftparameters = cell(length(bestMlps),1);
    
    totalLayers = length(mlp.layers);
    for ii = totalLayers:-1:1
        mlp.layers = mlp.layers(1:ii);
        t_xtrain = mlp.TransformData(xtrain);
        t_xtest = mlp.TransformData(xtest);
        [pftgsvm{ii}, pftlsvm{ii}, pftmsvm{ii}, pftsm{ii}, pftparameters{ii}] = external_classifiers_performance(t_xtrain, ytrain, t_xtest, ytest, task.numClasses);
    end
    save(paths{4}, 'ftgsvm', 'ftlsvm', 'ftmsvm', 'ftsm', 'ftparameters','pftgsvm', 'pftlsvm', 'pftmsvm', 'pftsm', 'pftparameters', 'masterTaskName');
end


