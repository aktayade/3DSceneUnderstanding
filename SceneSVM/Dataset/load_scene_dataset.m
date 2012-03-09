function [ xtrain, ytrain, xtest, ytest, datasetName ] =...
    load_scene_dataset( training, testing, subset, num_bases )

if nargin < 3
    subset = 'home';
end

if num_bases == 0 || nargin < 4
    baseName = 'noRbm';
else
    baseName = sprintf('%d', num_bases);
    w = scenerbm(0,num_bases,.01,'hgrad',100,100);
end

datasetName = sprintf('scene-%s-%d-%d-%s', subset, floor(training*100), floor(testing*100), baseName);

load(sprintf('%s.mat', subset));

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
xtrain = double(data(:,4:end));
ytrain = double(data(:,3));
xtest = zeros(0,size(xtrain,2));
ytest = zeros(0,size(ytrain,2));
%xtrain = double(data(trainInd,4:end));
%ytrain = double(data(trainInd,3) == subset);
%ytrain = double(data(trainInd,3));
%xtest = double(data(testInd,4:end));
%ytest = double(data(testInd,3) == subset);
%ytest = double(data(testInd,3));
end