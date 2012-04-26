function [seg_count seg_ind] = InputFeature(rawdata_name, spinimage_dir, seg_size_thres)

if ~exist(spinimage_dir,'dir')
    mkdir(spinimage_dir);
end

fprintf('Loading features from %s...\n\n',rawdata_name);
DATA = importdata(rawdata_name);
DATA = DATA.data;
AllSegNum = unique(DATA(:,1));
NUMseg = length(AllSegNum);
fprintf('Nummber of lines in feature_live.pcd.txt:  %d\n\n',NUMseg);

seg_ind = zeros(NUMseg,1);

seg_count = 1;
des_count = 0;

for INDseg = 1:NUMseg
    
    num_tmp = AllSegNum(INDseg);  % Get segment index
    seg_ind(seg_count) = num_tmp;
    
    ind_tmp = find(DATA(:,1)==num_tmp);
    DATA_tmp = DATA(ind_tmp,2:end);  % Get spin images of that segment
    
    num_desc_tmp = size(DATA_tmp,1);
    if(num_desc_tmp < seg_size_thres)
        continue;
    end
    des_count = des_count + num_desc_tmp;
    
    nan_ind = isnan(DATA_tmp(:,1));
    DATA_tmp(nan_ind,:) = [];
    
    features.segind = num_tmp;
    features.data = DATA_tmp;
    features.NUMdesc = length(ind_tmp);
    
    fprintf('Segment %4d  ', num_tmp);
    fprintf('Descriptor #: %6d',num_desc_tmp);
    if sum(nan_ind)~=0
        fprintf('  NaN value detected!!');
    end
    fprintf('\n');
    
    ts = num2str(seg_count+10000);
    tmp_name = fullfile(spinimage_dir,['seg_' ts(2:end) '.mat']);
    save(tmp_name, 'features')
    
    seg_count = seg_count+1;
    
end

seg_count = seg_count-1;
seg_ind(seg_count+1:end) = [];

fprintf('\nNumber of segments:\t\t%6d\n',seg_count);
fprintf('Number of descriptors loaded:\t%6d\n\n',size(DATA,1));
