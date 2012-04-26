function [ pftAcc, ftAcc, t_trainingData, t_testingData, mlp ] = ...
    mlpPerformance( trainingData, trainingLabels, testingData, testingLabels, unlabeledData, mlpParams, nameExtension )
%MLPPERFORMANCE Runs the whole pipeline of an MLP on the given data
% This code will train a deep network using RBMs or stacked autoencoders
% Data is expected to be DxN (Features x Examples)
% Training labels are expected to be 1:K
% mlpParams should be a MlpParameters class
% nameExtension is appended to the name of the mlp created in this function

% Build the mlp name
if mlpParams.useRBMs
     mlpParams.mlpName = [mlpParams.mlpName '_RBM'];
else
     mlpParams.mlpName = [mlpParams.mlpName '_Autoencoder'];
end
for ii = 1:length(mlpParams.hiddenLayers)
     mlpParams.mlpName = [mlpParams.mlpName sprintf('_%i', mlpParams.hiddenLayers(ii))];
end
if exist('nameExtension', 'var')
    mlpParams.mlpName = [mlpParams.mlpName nameExtension];
end

% Create the mlp
mlp = MultiLayerPerceptron.CreateFromStruct(mlpParams);

% Pretrain
if mlpParams.useModelSelection
    mlp.PretrainModelSelection(trainingData, trainingLabels, mlpParams.quick);
else
    mlp.Pretrain(trainingData, mlpParams.quick, mlpParams.sparsityParameters, mlpParams.lambdaParameters, mlpParams.betaParameters);
end

% Train softmax
mlp.TrainSoftmax(trainingData, trainingLabels);
preFineTunePred = mlp.Predict(testingData);
pftAcc = mean(testingLabels(:) == preFineTunePred(:));

% Finetune
mlp.FineTune(trainingData, trainingLabels, false);
fineTunePred = mlp.Predict(testingData);
ftAcc = mean(testingLabels(:) == fineTunePred(:));

% Transform the data
t_trainingData = mlp.TransformData(trainingData);
t_testingData = mlp.TransformData(testingData);
end

