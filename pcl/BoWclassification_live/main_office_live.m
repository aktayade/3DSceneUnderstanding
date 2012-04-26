close all; clear; clc;

tic;

addpath('SpatialPyramid/');
addpath('libsvm-3.11/matlab/');
addpath('DBNclassifier/');

rawdata_name = 'featdata/features_live.pcd.txt';
data_dir = 'cache/office_final/';

%% Set parameters
seg_size_thres = 10;

% ISRBMDL = false;
% training_data = 'training/train_kmeans_office.mat';
% classifier = 'training/classifier_kmeans_office.mat';
% kmeansparams.dictionarySize = 400;

ISRBMDL = true;
training_data = 'training/train_rbm_office.mat';
classifier = 'training/classifier_rbm_office.mat';
rbmparams.dictionarySize = 700;
rbmparams.sparsityParam = 30/rbmparams.dictionarySize;
rbmparams.lambda = 0.001;
rbmparams.beta = 0.1;
rbmparams.quick = false;

%% Load spin images and cornell features
unix('rm -r cache/office_final/spinImage');
spinimage_dir = fullfile(data_dir, 'spinImage');
[NUMseg segind] = InputFeature(rawdata_name, spinimage_dir,seg_size_thres);

%% Calculate dictoinary
if ISRBMDL
    dicLearn_dir = fullfile(data_dir, 'dicLearnRBM');    
else
    dicLearn_dir = fullfile(data_dir, 'dicLearn');
end

dicBaseDir = dicLearn_dir;

%% Build Histograms
unix('rm -r cache/office_final/hist');
unix('rm -r cache/office_final/histRBM');
fprintf('\n');

if ISRBMDL
    hist_dir = fullfile(data_dir, 'histRBM');
else
    hist_dir = fullfile(data_dir, 'hist');
end

BASEhist_temp = fullfile(hist_dir);
if ~exist(BASEhist_temp,'dir')
    mkdir(BASEhist_temp);
end

filenames = [];

for f = 1:NUMseg
    ts = num2str(f+10000);
    tmp_name = ['seg_' ts(2:end) '.mat'];
    filenames{f} = tmp_name;
end

segFileList = filenames;
spinimageBaseDir = fullfile(spinimage_dir);
histBaseDir = BASEhist_temp;

if ISRBMDL
    RBMBuildHistograms(segFileList, spinimageBaseDir, dicBaseDir, histBaseDir, '.mat' ,rbmparams);
else
    myBuildHistograms(segFileList, spinimageBaseDir, dicBaseDir, histBaseDir, '.mat' ,kmeansparams);
end

%% Split data into training and validation set
DATA = [];
LABEL = [];

for cls = 1:NUMseg

    hist_all = [];
    histBaseDir = fullfile(hist_dir);
    
    filenames = [];
    
    ts = num2str(cls+10000);
    imageFName = ['seg_' ts(2:end) '.mat'];
    [dirN base] = fileparts(imageFName);
    baseFName = fullfile(dirN, base);
    if ISRBMDL
        outFName2 = fullfile(histBaseDir, sprintf('%s_hist_%d_%.2e_%.2e.mat', baseFName, ...
            rbmparams.dictionarySize,rbmparams.sparsityParam,rbmparams.beta));
    else
        outFName2 = fullfile(histBaseDir, sprintf('%s_hist_%d.mat', baseFName, kmeansparams.dictionarySize));
    end
    
    load(outFName2);
    DATA = [DATA;H];
        
end

DATA_unm = DATA;
DATA = DATA ./ repmat(sum(DATA,2),[1 size(DATA,2)]);  % Normalize data

NUMnan = sum(isnan(DATA(:,1)));
fprintf('Number of NaN segments:  %d\n\n',NUMnan);

%% Compute histogram intersection kernel
fprintf('Computing histogram intersection kernel ...\n');
load(training_data);
K = hist_isect(DATA, DATA_TRAIN);

%% Classify data using SVM
load(classifier);
K = [(1:NUMseg)' K];
[predict_label, accuracy, dec_values] = svmpredict(zeros(NUMseg,1), K, model);

fid = fopen('results/result_live_rbmsvm.txt','w');
for INDseg = 1:NUMseg
    fprintf(fid,'%d %d\n',segind(INDseg),predict_label(INDseg));
end
fclose(fid);

%% Classify data using DBN
addpath('DBNclassifier/deeplearn_framework/');
threshold = 0;
load('OfficeSpinRbm400PRBMClassifier.mat');
data = DATA_unm' * scaleFactor;
dbn_labels = mlp.Predict(data, threshold);

fid = fopen('results/result_live_dbn.txt','w');
for INDseg = 1:NUMseg
    fprintf(fid,'%d %d\n',segind(INDseg),dbn_labels(INDseg));
end
fclose(fid);

fprintf('Classification DONE in %f seconds\n',toc);