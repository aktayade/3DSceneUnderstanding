close all; clear; clc;

addpath('SpatialPyramid/');
addpath('libsvm-3.11/matlab/');

rawdata_dir = '../featdata/rawfeat_office_usam0418/'; % rawfeat_office_th10_int016
mapping_file = '../featdata/mapping_office_bkp.mat';
data_dir = 'cache/office/';

%% Set parameters
SIZEdictionary = 400;           % Set size of dictionary
CalDicSegNUM = 5;               % Set number of segments per class for dictionary learning
NUMexp = 10;                    % Set number of experiments
NUMfold = 4;                    % Set number of folds
C_cand = (10.^(10:-1:-5))';     % Set candidate C value
clean_all_cache = false;        % Remove cache folder if true

%% Preprocess
if clean_all_cache
    unix('rm -r cache/office/');
end

params.dictionarySize = SIZEdictionary;

%% Load spin images
spinimage_dir = fullfile(data_dir, 'spinImage');  

LISTclass = InputFeature(rawdata_dir, spinimage_dir, mapping_file);
NUMclass = length(LISTclass);

%% Calculate dictoinary
dicLearn_dir = fullfile(data_dir, 'dicLearn');
BASESEGsel = fullfile(data_dir, 'dicLearn', 'selectedSEG');

if ~exist(BASESEGsel,'dir')
   mkdir(BASESEGsel);
end

fprintf('Selecting segments for dictionary learning... (%d per class)\n\n',CalDicSegNUM);

filenames = [];
for cls = 1:NUMclass
    
    spinimageBaseDir = fullfile(spinimage_dir, LISTclass(cls).name);
    
    for INDseg = 1:CalDicSegNUM
        
        INNAMEsel = LISTclass(cls).fnames(INDseg).name;
        OUTNAMEsel = sprintf('%s_%s',LISTclass(cls).name, INNAMEsel);
        [~,INbase] = fileparts(INNAMEsel);
        [~,OUTbase] = fileparts(OUTNAMEsel);
        
        filenames = [filenames {OUTNAMEsel}];
        
        INselSpinIMG = fullfile(spinimageBaseDir, sprintf('%s.mat',INbase));
        OUTselSpinIMG = fullfile(BASESEGsel, sprintf('%s.mat',OUTbase));
        
        if exist(OUTselSpinIMG,'file') == 0
            unix(['cp ' INselSpinIMG ' ' OUTselSpinIMG]);  % Copy selected data
        end
        
    end
    
end

imageFileList = filenames;
segBaseDir = BASESEGsel;
dicBaseDir = dicLearn_dir;

params.numTextonImages = length(imageFileList);

myCalculateDictionary(imageFileList, segBaseDir, dicBaseDir, '.mat', params);


%% Build Histograms
hist_dir = fullfile(data_dir, 'hist');
for cls = 1:NUMclass
    
    BASEhist_temp = fullfile(hist_dir, LISTclass(cls).name);
    if ~exist(BASEhist_temp,'dir')
       mkdir(BASEhist_temp);
    end
    
    filenames = [];
    for f = 1:LISTclass(cls).fnum;
        filenames{f} = LISTclass(cls).fnames(f).name;
    end

    segFileList = filenames;
    spinimageBaseDir = fullfile(spinimage_dir, LISTclass(cls).name);
    histBaseDir = BASEhist_temp;
    
    myBuildHistograms(segFileList, spinimageBaseDir, dicBaseDir, histBaseDir, '.mat' ,params);
    
end

%% Run experiments multiple times
conf = zeros(NUMclass,NUMclass,NUMfold,NUMexp);
acc_final = zeros(1,NUMexp);
c_sel = zeros(NUMexp,1);

macro_precision = zeros(1,NUMexp);
marro_recall = zeros(1,NUMexp);

fig1 = figure; hold on

for INDexp = 1:NUMexp
    %% Split data into training and validation set
    DATA = [];
    LABEL = [];
    
    fprintf('Splitting dataset into training and validation set ...\n\n');
    for cls = 1:NUMclass

        fprintf('Class %d: %s\n', cls, LISTclass(cls).name);

        hist_all = [];
        histBaseDir = fullfile(hist_dir, LISTclass(cls).name);

        filenames = [];    
        for f = 1:LISTclass(cls).fnum;

            imageFName = LISTclass(cls).fnames(f).name;
            [dirN base] = fileparts(imageFName);
            baseFName = fullfile(dirN, base);
            outFName2 = fullfile(histBaseDir, sprintf('%s_hist_%d.mat', baseFName, params.dictionarySize));

            load(outFName2);
            hist_all = [hist_all;H];
            hist_all = hist_all./repmat(sum(hist_all,2),[1 size(hist_all,2)]);  % Normalize histogram

        end

        DATA = [DATA;hist_all];
        LABEL = [LABEL;cls*ones(LISTclass(cls).fnum,1)];        

    end
    
    fold_indices = crossvalind('Kfold',LABEL,NUMfold);  % Generate fold indices
    
    fprintf('\n');

    %% Compute histogram intersection kernel
    %kernelBaseDir = fullfile(data_dir, 'HIKernel');
    %if ~exist(kernelBaseDir,'dir')
    %    mkdir(kernelBaseDir);
    %end

    fprintf('Computing histogram intersection kernel ...\n');
    tic;
    K = hist_isect(DATA, DATA);
    TIMEkernel = toc;
    fprintf('Computing time: %f seconds\n\n',TIMEkernel);
    
    %% Run cross validation on parameter C
    
    fprintf('Running cross validation on C ...\n\n');
    ACCval = zeros(length(C_cand), NUMfold);
    MACPRE = zeros(length(C_cand), NUMfold);
    MACREC = zeros(length(C_cand), NUMfold);
    conf_tmp = zeros(NUMclass,NUMclass,NUMfold,length(C_cand));
    
    for indC = 1:length(C_cand)
        
        C_test = C_cand(indC);
        
        fprintf('C_test = %.2e\n',C_test);
        svmtrain_option = sprintf('-c %f -t 4 -q',C_test);
        
        for fold = 1:NUMfold
            
            INDtrain = find(fold_indices ~= fold);
            INDval = find(fold_indices == fold);
            
            train_label = LABEL(fold_indices ~= fold);
            val_label = LABEL(fold_indices == fold);
            
            SIZEtrain = length(train_label);
            K_cv_train = K(INDtrain,INDtrain);
            K_cv_train_svm = [(1:SIZEtrain)', K_cv_train];
            
            SIZEval = length(val_label);
            K_cv_val = K(INDval,INDtrain);
            K_cv_val_svm = [(1:SIZEval)', K_cv_val];
            
            model = svmtrain(train_label, K_cv_train_svm, svmtrain_option);
            
            fprintf('fold %d, ',fold);
            [predict_label, accuracy, dec_values] = svmpredict(val_label, K_cv_val_svm, model);
            
            ACCval(indC,fold) = accuracy(1);
            
            [~,MACPRE(indC,fold),MACREC(indC,fold)] = perf_stats(val_label, predict_label, NUMclass);
            
            for i = 1:NUMclass
                for j = 1:NUMclass
                    conf_tmp(i,j,fold,indC) = sum(predict_label(val_label == i) == j)/ ...
                        sum(val_label == i);   
                end
            end
            
        end
        
    end
    
    ACCcv = mean(ACCval,2);
    MACPREcv = mean(MACPRE,2);
    MACRECcv = mean(MACREC,2);
    [acc_final(INDexp),C_ind] = max(ACCcv,[],1);
    c_sel(INDexp) = C_cand(C_ind);
    macro_precision(INDexp) = mean(MACPREcv(C_ind));
    macro_recall(INDexp) = mean(MACRECcv(C_ind));
    
    fprintf('\n');
    fprintf('C_sel: %f \n\n',c_sel(INDexp));

    %% Compute confusion matrix
    for fold = 1:NUMfold        
        conf(:,:,fold,INDexp) = conf_tmp(:,:,fold,C_ind);
        subplot(NUMfold,NUMexp,(fold-1)*NUMexp+INDexp);
        imagesc(conf(:,:,INDexp));
    end
    set(fig1,'Position',[2 275 1438 512]); 
    
end

for INDexp = 1:NUMexp
    fprintf('exp %2d  C: %.2e  acc: %f %%  mp: %f %%  mr: %f %%\n', ...
        INDexp,c_sel(INDexp),acc_final(INDexp),macro_precision(INDexp), ...
        macro_recall(INDexp));
end
fprintf('\n');
