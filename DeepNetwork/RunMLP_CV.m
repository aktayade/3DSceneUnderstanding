% This code will train a deep network using RBMs or stacked autoencoders
% Data is expected to be DxN (Features x Examples)
% Training labels are expected to be 1:J
% Splits the data into K fold cross validation and reports this accuracy
% Does model selection on the whole framework adjusting sparsity
addpath(genpath('deeplearn_framework'));
outputFolder = 'results/';

load('home.mat');
data = data';
K = 4;
cvIndices = crossvalind('Kfold', data(3,:), K);
trainingData = data(4:end, :);
trainingLabels = data(3, :);

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

if useRBMs
    pretrainType = 'RBM';
else
    pretrainType = 'Autoencoder';
end
layerName = '';
for ii = 1:length(hiddenLayers)
    layerName = [layerName sprintf('_%i', hiddenLayers(ii))];
end

mlp = MultiLayerPerceptron(sprintf('%s_%s%s_CrossVal', trialName, pretrainType, layerName), hiddenLayers, numClasses, binaryInput, useRBMs);

mlp.PretrainModelSelection(trainingData, trainingLabels, false);

models = cell(K,1);
preFineTuneAccuracyBuf = zeros(K,1);
fineTuneAccuracyBuf = zeros(K,1);
for k = 1:K
    models{k} = mlp.Clone(sprintf('_%i', k));
end
parfor k = 1:K
    test = (cvIndices == k);
    train = ~test;
    models{k}.TrainSoftmax(trainingData(:,train), trainingLabels(train));
    preFineTunePred = models{k}.Predict(trainingData(:,test));
    tempTestLabels = trainingLabels(test);
    preFineTuneAccuracyBuf(k) = mean(tempTestLabels(:) == preFineTunePred(:));

    models{k}.FineTune(trainingData(:,train), trainingLabels(train), false);
    fineTunePred = models{k}.Predict(trainingData(:,test));
    tempTestLabels = trainingLabels(test);
    fineTuneAccuracyBuf(k) = mean(tempTestLabels(:) == fineTunePred(:));
end

preFineTuneAccuracy = mean(preFineTuneAccuracyBuf);
fineTuneAccuracy = mean(fineTuneAccuracy);

fprintf('Before Finetuning Test Accuracy: %0.3f%%\n', preFineTuneAccuracy * 100);
fprintf('After Finetuning Test Accuracy: %0.3f%%\n', fineTuneAccuracy * 100);

transformedTrainingData = mlp.TransformData(trainingData);

% Save the result
if ~exist(outputFolder,'dir')
    mkdir(outputFolder);
end
save(sprintf('%s%s_%s.mat', outputFolder, mlp.networkName), 'preFineTuneAccuracy', 'fineTuneAccuracy',...
    'transformedTrainingData', 'trainingLabels');



