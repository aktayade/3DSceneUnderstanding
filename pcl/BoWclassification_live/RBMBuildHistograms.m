function [ H_all ] = RBMBuildHistograms( segFileList, dataBaseDir, dicBaseDir, histBaseDir, featureSuffix, params)

fprintf('Building Histograms\n\n');

%% Load RBM dictionary
inFName = fullfile(dicBaseDir, sprintf('RBMdictionary_%d_%.2e_%.2e.mat', ...
    params.dictionarySize,params.sparsityParam,params.beta));
load(inFName,'RBMdictionary');
fprintf('Loaded texton dictionary: %d textons\n\n', params.dictionarySize);

%% Compute texton labels and whole-seg histograms
H_all = [];

for f = 1:length(segFileList)

    segFName = segFileList{f};
    [dirN base] = fileparts(segFName);
    baseFName = fullfile(dirN, base);
    inFName = fullfile(dataBaseDir, sprintf('%s%s', baseFName, featureSuffix));
    
    outFName = fullfile(histBaseDir, sprintf('%s_texton_ind_%d_%.2e_%.2e.mat', baseFName, params.dictionarySize,params.sparsityParam,params.beta));
    outFName2 = fullfile(histBaseDir, sprintf('%s_hist_%d_%.2e_%.2e.mat', baseFName, params.dictionarySize,params.sparsityParam,params.beta));
    if(exist(outFName,'file')~=0 && exist(outFName2,'file')~=0)
        if(nargout>1)
            load(outFName2, 'H');
            H_all = [H_all; H];
        end
        continue;
    end
    
    %% Load descriptors    
    load(inFName, 'features');
    ndesc = size(features.data,1);
    
    %% Find texton indices and compute histogram 
    texton_ind.data = zeros(ndesc,1);
    desc_seg = features.data';
    
    %desc_seg = desc_seg - repmat(mean(desc_seg,1),[size(desc_seg,1) 1]);
    %desc_seg = desc_seg./ repmat(std(desc_seg,[],1),[size(desc_seg,1) 1]);
    
    %desc_seg(desc_seg~=0)=1;
    
    %desc_seg = desc_seg./ repmat(max(desc_seg,[],1),[size(desc_seg,1) 1]);
    
    D=desc_seg;
    D(desc_seg<repmat(mean(desc_seg,1),[size(desc_seg,1) 1])) = 0;
    D(desc_seg>=repmat(mean(desc_seg,1),[size(desc_seg,1) 1])) = 1;
    desc_seg = D;
    
    %D =desc_seg;
    %M = repmat(mean(desc_seg,1),[size(desc_seg,1) 1]);
    %D =1./(1+exp(-1000*(D-M)));
    %desc_seg = D;
    
    %desc_seg = round(100*desc_seg./repmat(max(desc_seg,[],1),[size(desc_seg,1) 1]));
    
    transformed = FeedForward(RBMdictionary, desc_seg);  % Feed forward in RBM
    texton_ind.data = transformed';
    
    %texton_ind.data = (texton_ind.data - params.sparsityParam)./ ...
    %    repmat(sum(texton_ind.data,2),[1 size(texton_ind.data,2)]);
    
    %[~,max_ind] = max(texton_ind.data,[],2);
    %H = hist(max_ind,1:params.dictionarySize);
    
    %texton_ind.data = log(texton_ind.data);
    %texton_ind.data(isinf(texton_ind.data)) = 0;
    %texton_ind.data = -texton_ind.data;
    
    H = sum(texton_ind.data,1);
    %H = max(texton_ind.data,[],1);
    H_all = [H_all; H];

    %% Save texton indices and histograms
    if(isdir(histBaseDir)==0)
        mkdir(histBaseDir);
    end
    save(outFName, 'texton_ind');
    save(outFName2, 'H');
    
end

%% Save histograms of all images in this directory in a single file
% outFName = fullfile(histBaseDir, sprintf('histograms_%d.mat', params.dictionarySize));
%save(outFName, 'H_all', '-ascii');


end
