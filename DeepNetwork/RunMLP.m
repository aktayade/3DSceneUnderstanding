% This code will train a deep network using RBMs or stacked autoencoders
% Data is expected to be DxN (Features x Examples)
% Training labels are expected to be 1:K
addpath(genpath('deeplearn_framework'));
outputFolder = 'results/';

load('home.mat');
data = data';
trainInd = crossvalind('holdout', data(3,:), 0.3); % 30 percent testing
trainingData = data(4:end, trainInd);
trainingLabels = data(3, trainInd);
testingData = data(4:end, ~trainInd);
testingLabels = data(3, ~trainInd);

trialName = 'OriginalFeatures';     % Base file name for saving output
binaryInput = false;                % True if features are between [0,1]
numClasses = 17;                    % Number of classes (labels values)
useRBMs = true;                     % If false autoencoders used instead
hiddenLayers = [200 200];           % The architecture of the network

% If model selection is true then the parameters given are ignored and a
% range of parameters are tried.  This will take significantly longer.
modelSelection = false;
sparsityParameters = [.1, .1];      % Controls hidden unit activity
lambdaParameters = [3e-3 3e-3];     % L2 regularization term
betaParameters = [3 3];             % Weight of the sparsity parameter

% Standardize the data to have zero mean unit variance
[trainingData, dataMean, dataStd] = StandardizeData(trainingData);
testingData = StandardizeData(testingData, dataMean, dataStd);

if useRBMs
    pretrainType = 'RBM';
else
    pretrainType = 'Autoencoder';
end
layerName = '';
for ii = 1:length(hiddenLayers)
    layerName = [layerName sprintf('_%i', hiddenLayers(ii))];
end

mlp = MultiLayerPerceptron(sprintf('%s_%s%s', trialName, pretrainType, layerName), hiddenLayers, numClasses, binaryInput, useRBMs);

if modelSelection
    mlp.PretrainModelSelection(trainingData, trainingLabels, false);
else
    mlp.Pretrain(trainingData, false, sparsityParameters, lambdaParameters, betaParameters);
end
mlp.TrainSoftmax(trainingData, trainingLabels);
preFineTunePred = mlp.Predict(testingData);
preFineTuneAccuracy = mean(testingLabels(:) == preFineTunePred(:));

mlp.FineTune(trainingData, trainingLabels, false);
fineTunePred = mlp.Predict(testingData);
fineTuneAccuracy = mean(testingLabels(:) == fineTunePred(:));

fprintf('Before Finetuning Test Accuracy: %0.3f%%\n', preFineTuneAccuracy * 100);
fprintf('After Finetuning Test Accuracy: %0.3f%%\n', fineTuneAccuracy * 100);

transformedTrainingData = mlp.TransformData(trainingData);
transformedTestingData = mlp.TransformData(testingData);

% Save the result
if ~exist(outputFolder,'dir')
    mkdir(outputFolder);
end
save(sprintf('%s%s_%s.mat', outputFolder, mlp.networkName), 'preFineTuneAccuracy', 'fineTuneAccuracy',...
    'transformedTrainingData', 'transformedTestingData', 'trainingLabels', 'testingLabels');



