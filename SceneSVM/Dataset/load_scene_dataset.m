function [ xtrain, ytrain, xtest, ytest, datasetName ] =...
    load_scene_dataset( training, testing, subset, num_bases )

if nargin < 3
    subset = 'home';
end

rbmUsed = 0;

if num_bases == 0 || nargin < 4
    baseName = 'noRbm';
    load(sprintf('%s.mat', subset));
else
    baseName = sprintf('rbm%d', num_bases);
    load(sprintf('%s_%s.mat', subset, baseName));
    rbmUsed = 1;
end

datasetName = sprintf('scene-%s-%d-%d-%s', subset, floor(training*100), floor(testing*100), baseName);



%{
exampleCount = size(data, 1);

%% Convert percentages to counts
if training < 1.0 && training > 0.0
   assert(testing == 0 || testing > 1.0 || training + testing <= 1.0,...
       'Training and testing percentages must sum to <= 1.0');
   training = floor(exampleCount * training);
end
if testing < 1.0 && testing > 0.0
   testing = floor(exampleCount * testing);
end

%% Set default values
if training == 0 && testing > 0.0
   training = exampleCount - testing;
elseif training > 0.0 && testing == 0
   testing = exampleCount - training;
elseif training == 0 && testing == 0
   error('Cannot default both training and testing values');
end
%}

%% Generate data
%trainInd = 1:training;
%testInd = (training+1):(training+testing);
if rbmUsed
    xtrain = double(data(:,2:end));
    ytrain = double(data(:,1));
    xtest = zeros(0,size(xtrain,2));
    ytest = zeros(0,size(ytrain,2));
else
    xtrain = double(data(:,4:end));
    ytrain = double(data(:,3));
    xtest = zeros(0,size(xtrain,2));
    ytest = zeros(0,size(ytrain,2));
end
%xtrain = double(data(trainInd,4:end));
%ytrain = double(data(trainInd,3) == subset);
%ytrain = double(data(trainInd,3));
%xtest = double(data(testInd,4:end));
%ytest = double(data(testInd,3) == subset);
%ytest = double(data(testInd,3));
end