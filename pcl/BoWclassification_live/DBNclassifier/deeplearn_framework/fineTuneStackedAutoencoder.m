function [ tunedStack, tunedSoftmax ] = fineTuneStackedAutoencoder(stack, softmaxModel, numClasses, data, labels, lambda, maxIter)
%FINETUNESTACKEDAUTOENCODER Takes a pretrained stacked autoencoder and runs
% backpropagation to fine tune it.

    if ~exist('maxIter', 'var')
        maxIter = 400;
    end
    if ~exist('lambda', 'var')
        lambda = 1e-3;  % For the softmax classifier regularization only
    end

    stackedAETheta = compressParameters(stack, softmaxModel.optTheta);
    
    deeplearnGlobalPaths;
    options.Method = 'lbfgs';
    options.maxIter = maxIter;
    options.display = 'on';
    [theta, ~] = minFunc( @(p) stackedAECost(p, stack, numClasses, lambda, data, labels),...
                                    stackedAETheta, options);

    tunedSoftmax = softmaxModel;
    [tunedStack, tunedSoftmax.optTheta] = uncompressParameters(stack, theta, numClasses, size(data,1));
end

function [ cost, grad ] = stackedAECost(theta, stack, numClasses, lambda, data, labels)
% stackedAECost: Takes a trained softmaxTheta and a training data set with labels,
% and returns cost and gradient using a stacked autoencoder model. Used for
% finetuning.

[stack, softmaxTheta] = uncompressParameters(stack, theta, numClasses, size(data,1));

stackgrad = cell(size(stack));
for d = 1:numel(stack)
    stackgrad{d}.W = zeros(size(stack{d}.W));
    stackgrad{d}.b = zeros(size(stack{d}.b));
end

numCases = size(data, 2);
groundTruth = full(sparse(labels, 1:numCases, 1));

% Forward prop
activation = cell(length(stack)+1,1);
activation{1} = data;
for l = 1:numel(stack)
    activation{l+1} = stack{l}.FeedForward(activation{l});
end

% M = theta'*x
M = softmaxTheta*activation{numel(stack)+1};
% For numerical purposes shrink M before taking exponential
M = bsxfun(@minus, M, max(M, [], 1));
% Take exponential
M = exp(M);
% Compute the normalization term and normalize.  M now is the probabilities
M = bsxfun(@rdivide, M, sum(M));
cost = -sum(sum(groundTruth .* log(M)))/numCases + (lambda/2)*sum(sum(softmaxTheta.^2));
softmaxThetaGrad = -(1/numCases)*(groundTruth - M)*activation{numel(stack)+1}' + lambda*softmaxTheta;

% Back propagate
delta = -softmaxTheta' * (groundTruth - M) .* stack{numel(stack)}.BackpropDerivitive(activation{numel(stack)+1});
for l = numel(stack):-1:1
    stackgrad{l}.W = delta * activation{l}' / numCases;
    stackgrad{l}.b = mean(delta,2);
    if l > 1
        delta = (stack{l}.W'*delta) .* stack{l-1}.BackpropDerivitive(activation{l});
    end
end

grad = compressParameters(stackgrad, softmaxThetaGrad);
end

function theta = compressParameters(stack, softmaxWeights)
    % Compress weights and biases into a vector
    params = [];
    for d = 1:numel(stack)
        params = [params; stack{d}.W(:); stack{d}.b(:)];

        % Check that stack is of the correct form
        assert(size(stack{d}.W, 1) == size(stack{d}.b, 1), ...
            ['The bias should be a *column* vector of ' ...
             int2str(size(stack{d}.W, 1)) 'x1']);
        if d < numel(stack)
            assert(size(stack{d}.W, 1) == size(stack{d+1}.W, 2), ...
                ['The adjacent layers L' int2str(d) ' and L' int2str(d+1) ...
                 ' should have matching sizes.']);
        end
    end
    theta = [softmaxWeights(:) ; params];
end

function [stack, softmaxWeights] = uncompressParameters(stack, theta, numClasses, inputSize)
    % Uncompress the params into weights and biases
    
    % Reshape the softmax and stack
    softmaxWeights = reshape(...
        theta(1:stack{numel(stack)}.hiddenSize*numClasses),...
        numClasses, stack{numel(stack)}.hiddenSize);
    params = theta((stack{numel(stack)}.hiddenSize*numClasses+1):end);
    
    prevLayerSize = inputSize;      % the size of the previous layer
    curPos = double(1);             % mark current position in parameter vector
    for d = 1:numel(stack)
        % Extract weights
        wlen = double(stack{d}.hiddenSize * prevLayerSize);
        stack{d}.W = reshape(params(curPos:curPos+wlen-1), stack{d}.hiddenSize, prevLayerSize);
        curPos = curPos+wlen;

        % Extract bias
        blen = double(stack{d}.hiddenSize);
        stack{d}.b = reshape(params(curPos:curPos+blen-1), stack{d}.hiddenSize, 1);
        curPos = curPos+blen;

        % Set previous layer size
        prevLayerSize = stack{d}.hiddenSize;
    end
end