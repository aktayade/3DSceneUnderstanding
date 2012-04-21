function [ ] = create_cross_validated_task( taskName, data, labels, foldCount, unlabeledData, standardize )
%CREATE_CROSS_VALIDATED_TASK Sets up a CV task
%   This function creates a classification task from raw data that will
%   ultimately be judged based on cross validation accuracy
% taskName - Base name of this task to be used throughout the processing
% data - [dxn] matrix. d = # of dimensions, n = # of training examples
% labels - [1xn] matrix with labels taking values 1:k
% foldCount - number of folds of cross validation to be done
% unlabeled - [dxn] matrix with data that is unlabeled. Or leave empty.
% If standardize is true then the data will be set to have 0 mean unit var
%
    global_paths;
    
    if ~exist('unlabeledData', 'var') || isempty(unlabeledData)
        unlabeledData = zeros(size(data,1),0);
    end
    
    assert(nargin >= 4, 'Not enough arguments');
    assert(min(labels) == 1 && length(unique(labels)) == max(labels), 'Labels must take values 1:K');
    assert(size(labels,1) == 1, 'Labels must be 1xN, data must be dxN');
    assert(size(labels,2) == size(data,2), 'Data count does not match label count');
    assert(size(unlabeledData,1) == size(data,1), 'Unlabeled dimension does not match data dimension');

    update_task_parameters(taskName, 'dataType', 'kfold', 'kfolds', foldCount, 'numClasses', max(labels));
    
    unlabeled = unlabeledData;
    
    cvIndices = crossvalind('Kfold', labels, foldCount);
    for k = 1:foldCount
        test = (cvIndices == k);
        train = ~test;
        xtrain = data(:, train);
        xtest = data(:, test);
        ytrain = labels(train);
        ytest = labels(test);
        
        if standardize
            [xtrain, dm, dstd] = standardize_data(xtrain);
            [xtest] = standardize_data(xtest, dm, dstd);
            [unlabeled] = standardize_data(unlabeledData, dm, dstd);
        end
        
        fpath = generate_write_file_path({'tasks', 'output', 'taskData'}, taskName, 'fold', k);
        masterTaskName = taskName;
        save(fpath, 'xtrain', 'ytrain', 'xtest', 'ytest', 'unlabeled', 'masterTaskName');
    end
    fprintf('Success\n');
end

