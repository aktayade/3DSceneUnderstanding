function [softmaxModel] = softmaxClassifierTrain(numClasses, lambda, inputData, labels, maxIter)
%TRAINSOFTMAXCLASSIFIER Train a softmax model with the given parameters on the given
% data. Returns softmaxOptTheta, a vector containing the trained parameters
% for the model.
%
% numClasses: the number of classes 
% lambda: weight decay parameter
% inputData: an N by M matrix containing the input data, such that
%            inputData(:, c) is the cth input
% labels: M by 1 matrix containing the class labels for the
%            corresponding inputs. labels(c) is the class label for
%            the cth input
% options (optional): options
%   options.maxIter: number of iterations to train for

inputSize = size(inputData,1);

if ~exist('maxIter', 'var')
    maxIter = 400;
end

% initialize parameters
theta = 0.005 * randn(numClasses * inputSize, 1);

% Use minFunc to minimize the function
deeplearnGlobalPaths;
options.Method = 'lbfgs';
options.display = 'on';
options.maxIter = maxIter;

[softmaxOptTheta, ~] = minFunc( @(p) softmaxCost(p, ...
                                   numClasses, inputSize, lambda, ...
                                   inputData, labels), ...                                   
                              theta, options);

% Fold softmaxOptTheta into a nicer format
softmaxModel.optTheta = reshape(softmaxOptTheta, numClasses, inputSize);
softmaxModel.inputSize = inputSize;
softmaxModel.numClasses = numClasses;
end

function [cost, grad] = softmaxCost(theta, numClasses, inputSize, lambda, data, labels)
% numClasses - the number of classes 
% inputSize - the size N of the input vector
% lambda - weight decay parameter
% data - the N x M input matrix, where each column data(:, i) corresponds to
%        a single test set
% labels - an M x 1 matrix containing the labels corresponding for the input data

% Unroll the parameters from theta
theta = reshape(theta, numClasses, inputSize);

numCases = size(data, 2);
groundTruth = full(sparse(labels, 1:numCases, 1));

% M = theta'*x
M = theta*data;
% For numerical purposes shrink M before taking exponential
M = bsxfun(@minus, M, max(M, [], 1));
% Take exponential
M = exp(M);
% Compute the normalization term and normalize.  M now is the probabilities
M = bsxfun(@rdivide, M, sum(M));

cost = -sum(sum(groundTruth .* log(M)))/numCases + (lambda/2)*sum(sum(theta.^2));
thetagrad = -(1/numCases)*(groundTruth - M)*data' + lambda*theta;
grad = [thetagrad(:)];
end



