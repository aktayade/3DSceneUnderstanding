function [ ] = stage_external_classify_raw( taskName, stayActive, target )
%STAGE_EXTERNAL_CLASSIFY Classifies raw data using various classifiers
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
    
    outputTo = {'tasks', 'output', 'externalraw'};
    intermediate = {'tasks', 'intermediate', 'externalraw'};
    rawData = {'tasks', 'output', 'taskData'};
    
    find_work(@work, taskName, stayActive, target, rawData, {}, {outputTo, intermediate});
end

function work(paths, ~)
    progress = 0;
    save(paths{3}, 'progress');
    load(paths{1});
    task = read_task_parameters(masterTaskName);
    [gsvm, lsvm, msvm, sm, parameters] = external_classifiers_performance(xtrain, ytrain, xtest, ytest, task.numClasses);
    save(paths{2}, 'gsvm', 'lsvm', 'msvm', 'sm', 'parameters', 'masterTaskName');
end




