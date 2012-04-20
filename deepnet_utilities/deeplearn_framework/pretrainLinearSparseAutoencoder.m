function [ W,b ] = pretrainLinearSparseAutoencoder...
    (data, hiddenSize, sparsityParam, lambda, beta, maxIter)                     
%PRETRAINBINARYSPARSEAUTOENCODER Pretrains binary autoencoder
% data - Pretraining data (DxN)
% hiddenSize - Number of hidden units
% sparsityParam - desired average activation of the hidden units
% lambda - weight decay parameter (weight regularization)
% beta - weight of sparsity penalty term

if nargin < 1 error('Need data parameter'); end
if nargin < 2 error('Need hiddenSize parameter'); end
if nargin < 3 sparsityParam = 0.035; end
if nargin < 4 lambda = 3e-3; end
if nargin < 5 beta = 5; end
if nargin < 6 maxIter = 400; end

visibleSize = size(data,1);
theta = initializeParameters(hiddenSize, visibleSize);

%  Use minFunc to minimize the function
deeplearnGlobalPaths;
options.Method = 'lbfgs';
options.maxIter = maxIter;
options.display = 'on';

[opttheta, ~] = minFunc( @(p) sparseAutoencoderLinearCost(p, ...
                                   visibleSize, hiddenSize, ...
                                   lambda, sparsityParam, ...
                                   beta, data), ...
                              theta, options);

W = reshape(opttheta(1:hiddenSize*visibleSize), hiddenSize, visibleSize);
b = opttheta(2*hiddenSize*visibleSize+1:2*hiddenSize*visibleSize+hiddenSize);
end

function [cost,grad] = sparseAutoencoderLinearCost(theta, visibleSize, hiddenSize, ...
                                             lambda, sparsityParam, beta, data)
% visibleSize: the number of input units
% hiddenSize: the number of hidden units
% lambda: weight decay parameter
% sparsityParam: The desired average activation for the hidden units
% beta: weight of sparsity penalty term
% data: Our training data.  data(:,i) is the i-th training example. 

W1 = reshape(theta(1:hiddenSize*visibleSize), hiddenSize, visibleSize);
W2 = reshape(theta(hiddenSize*visibleSize+1:2*hiddenSize*visibleSize), visibleSize, hiddenSize);
b1 = theta(2*hiddenSize*visibleSize+1:2*hiddenSize*visibleSize+hiddenSize);
b2 = theta(2*hiddenSize*visibleSize+hiddenSize+1:end);

exampleCount = size(data,2);
roe = sparsityParam;

% Forward pass
hiddenZ = W1 * data + repmat(b1,1,exampleCount);
hiddenAct = sigmoid(hiddenZ);
outZ = W2 * hiddenAct + repmat(b2,1,exampleCount);
outAct = outZ;

roeavg = mean(hiddenAct,2);

cost = .5*mean(sum((outAct - data).^2));

deltaOut = -(data - outAct);
deltaHidden = ( (W2'*deltaOut) + repmat(beta*(-roe./roeavg + (1-roe)./(1-roeavg)), 1, exampleCount) ) .*((hiddenAct .* (1-hiddenAct)));
W2grad = (deltaOut * hiddenAct');
b2grad = mean(deltaOut,2);
W1grad = (deltaHidden * data');
b1grad = mean(deltaHidden,2);

% Average over all examples
W2grad = W2grad / exampleCount;
W1grad = W1grad / exampleCount;

% Augment with regularization penalty:
cost = cost + (lambda/2 * (sum(sum(W1.^2)) + sum(sum(W2.^2))));
W1grad = W1grad + lambda * W1;
W2grad = W2grad + lambda * W2;

% Augment cost with sparsity penalty:
cost = cost+beta*sum(roe*log(roe./roeavg)+(1-roe)*log((1-roe)./(1-roeavg)));

grad = [W1grad(:) ; W2grad(:) ; b1grad(:) ; b2grad(:)];

end
