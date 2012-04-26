function [pred] = softmaxClassifierPredict(softmaxModel, data, threshold)
% softmaxModel - model trained using softmaxTrain
% data - the N x M input matrix, where each column data(:, i) corresponds to
%        a single test set
 
% Unroll the parameters from theta
theta = softmaxModel.optTheta;  % this provides a numClasses x inputSize matrix

% M = theta'*x
M = theta*data;
% For numerical purposes shrink M before taking exponential
M = bsxfun(@minus, M, max(M, [], 1));
% Take exponential
M = exp(M);
% Compute the normalization term and normalize.  M now is the probabilities
M = bsxfun(@rdivide, M, sum(M));

[v, pred] = max(M);
pred(v < threshold) = 0;
end

