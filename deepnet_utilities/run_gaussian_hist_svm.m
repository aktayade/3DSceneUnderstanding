function [ ] = run_gaussian_hist_svm( data, labels, gaussianSize, histSize, dataName )
% Runs model selection on gaussian and histogram data.  Data will be
% standardized for the gaussian part and normalized for the histogram part
%   
    if nargin < 5
        error('Not enough arguments');
    end
    global_paths;
    foldCount = 4;
    
    cvIndices = crossvalind('Kfold', labels, foldCount);
    
    parameters.lambdaSpace = logspace(-4, 1, 10);
    parameters.sigmaSpace = logspace(-1, 4, 10);
    parameters.alphaSpace = linspace(.1, .9, 9);
    [optParams, csvm] = grid_search({parameters.lambdaSpace; parameters.sigmaSpace, parameters.alphaSpace}, @combined_performance, '', false, data', labels', numClasses, gaussianSize, histSize, cvIndices);
    save([dataName '-combinedSVM.mat'], 'csvm', 'optParams');
end

function [result] = combined_performance(data, labels, numClasses, gaussianSize, histSize, cvIndices)
    fprintf('Training Gaussian SVM with lambda(%f) sigma(%f) alpha(%f)\n', params(1), params(2), params(3));
    c = 1/params(1);
    g = 1/(2*params(2)^2);
    
    assert(size(labels,1) > 1, 'training vector dimensions swapped');
    assert(gaussianSize + histSize == size(data,1), 'Split sizes dont add to total');
    
    acc_k = zeros(foldCount,1);
    MP_k = zeros(foldCount,1);
    MR_k = zeros(foldCount,1);
    
    for k = 1:foldCount
        test = (cvIndices == k);
        train = ~test;
        xtrain = data(:, train);
        xtest = data(:, test);
        ytrain = labels(train);
        ytest = labels(test);
        
        gind = 1:gaussianSize;
        hind = (gaussianSize+1):(gaussianSize+histSize);
        
        % Standardize gaussian data
        [xtrain(gind,:), dm, dstd] = standardize_data(xtrain(gind,:));
        [xtest(gind,:)] = standardize_data(xtest(gind,:), dm, dstd);
        
        % Normalize histogram data
        xtrain(hind,:) = xtrain(hind,:) ./ repmat(sum(xtrain(hind,:),1), [length(hind) 1]);
        xtest(hind,:) = xtest(hind,:) ./ repmat(sum(xtest(hind,:),1), [length(hind) 1]);

        DATA = xtrain';
        DATA_1 = repmat(permute(DATA,[2 3 1]),[1 size(DATA,2) 1]);
        DATA_2 = repmat(permute(DATA,[3 2 1]),[size(DATA,2) 1 1]);
        DIST = sum((DATA_1 - DATA_2).^2,3);
        K_rbf = exp(-g*DIST);
        K_intersection = hist_isect(xtrain, xtrain);
        K_combined = params(3)*K_rbf + (1-params(3))*K_intersection;
        model = svmtrain(ytrain, K_combined, ['-c ' num2str(c) ' -g ' num2str(g) ' -m 1000 -t 4']);

        DATA = xtest';
        DATA_1 = repmat(permute(DATA,[2 3 1]),[1 size(DATA,2) 1]);
        DATA_2 = repmat(permute(DATA,[3 2 1]),[size(DATA,2) 1 1]);
        DIST = sum((DATA_1 - DATA_2).^2,3);
        K_rbf = exp(-g*DIST);
        K_intersection = hist_isect(xtrain, xtrain);
        K_combined = params(3)*K_rbf + (1-params(3))*K_intersection;
        [class] = svmpredict(ytest, K_combined, model);

        [acc_k(k) MP_k(k) MR_k(k)] = perf_stats(ytest, class, numClasses);
    end
    result.acc = mean(acc_k);
    result.MP = mean(MP_k);
    result.MR = mean(MR_k);
    result.score = result.acc;
end

function K = hist_isect(x1, x2)

% Evaluate a histogram intersection kernel, for example
%
%    K = hist_isect(x1, x2);
%
% where x1 and x2 are matrices containing input vectors, where 
% each row represents a single vector.
% If x1 is a matrix of size m x o and x2 is of size n x o,
% the output K is a matrix of size m x n.

n = size(x2,1);
m = size(x1,1);
K = zeros(m,n);

if (m <= n)
   for p = 1:m
       nonzero_ind = find(x1(p,:)>0);
       tmp_x1 = repmat(x1(p,nonzero_ind), [n 1]); 
       K(p,:) = sum(min(tmp_x1,x2(:,nonzero_ind)),2)';
   end
else
   for p = 1:n
       nonzero_ind = find(x2(p,:)>0);
       tmp_x2 = repmat(x2(p,nonzero_ind), [m 1]);
       K(:,p) = sum(min(x1(:,nonzero_ind),tmp_x2),2);
   end
end
end
