% Expected performance
% Before Finetuning Test Accuracy: 87.7%
% After Finetuning Test Accuracy:  97.6%

addpath mnist/

quick = false;
model_select = false;
useRBMs = true;

name = 'mnist';
if model_select
    name = [name '_model_selected'];
end
if useRBMs
    name = [name '_RBM'];
end
if quick
    name = [name '_quick'];
end
mlp = MultiLayerPerceptron(name, [200 200], 10, true, useRBMs);

% Load MNIST database files
trainData = loadMNISTImages('train-images-idx3-ubyte');
trainLabels = loadMNISTLabels('train-labels-idx1-ubyte');
testData = loadMNISTImages('t10k-images-idx3-ubyte');
testLabels = loadMNISTLabels('t10k-labels-idx1-ubyte');
trainLabels(trainLabels == 0) = 10; % Remap 0 to 10 since our labels need to start from 1
testLabels(testLabels == 0) = 10; % Remap 0 to 10

%trainData = trainData(:, 1:100);
%testData = testData(:, 1:100);
%trainLabels = trainLabels(1:100);
%testLabels = testLabels(1:100);

% Train the network
if model_select
    mlp.DoEverything(trainData, trainLabels, quick);
else
    mlp.Pretrain(trainData, quick, [.1, .1], [3e-3 3e-3], [3 3]);
    mlp.TrainSoftmax(trainData, trainLabels);
    preFineTunePred = mlp.Predict(testData);
    acc = mean(testLabels(:) == preFineTunePred(:));
    mlp.FineTune(trainData, trainLabels, quick);
    fprintf('Before Finetuning Test Accuracy: %0.3f%%\n', acc * 100);
end

% Visualize weights
figure;
display_network(mlp.layers{1}.W');

% Run prediction
fineTunePred = mlp.Predict(testData);
acc = mean(testLabels(:) == fineTunePred(:));
fprintf('After Finetuning Test Accuracy: %0.3f%%\n', acc * 100);
