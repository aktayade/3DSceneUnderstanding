load('mapped_node_features/home.mat');

% Use all 17 classes
PickClasses = [1:17];

Features = [];
Labels = [];
for i = PickClasses
    ClassIndex = find(data(:, 3) == i);
    Features = [Features; data(ClassIndex, 4:end)];
    Labels = [Labels; data(ClassIndex, 3)];
end

% Generate training, validation, and test set
%TotalSz = 0;
%for i = 1:length(PickClasses)
%    TotalSz = TotalSz + sum(Features(:,3)==i);
%end
TotalSz = size(Features, 1);
TrainSize = round(5*TotalSz/7);
TestSize = round(TotalSz/7);
ValidSize = TotalSz - TrainSize - TestSize;

RandomIdx = randperm(TotalSz);

FeatTrain = Features(RandomIdx(1:TrainSize), :);
LabelTrain = Labels(RandomIdx(1:TrainSize), :);

FeatTest = Features(RandomIdx(TrainSize+1:TrainSize+TestSize), :);
LabelTest = Labels(RandomIdx(TrainSize+1:TrainSize+TestSize), :);

FeatValid = Features(RandomIdx(TrainSize+TestSize+1:end), :);
LabelValid = Labels(RandomIdx(TrainSize+TestSize+1:end), :);

% Write features and labels into text file
DIMfeat = size(FeatTrain,2);

fid = fopen('data_c17_train.txt','w');
for INDtrain = 1:TrainSize
    for INDfeat = 1:DIMfeat
        fprintf(fid,'%.9f',FeatTrain(INDtrain,INDfeat));
        if INDfeat == DIMfeat
            fprintf(fid,'\n');
        else
            fprintf(fid,' ');
        end
    end
    fprintf(fid,'%d\n',LabelTrain(INDtrain));
end
fclose(fid);

fid = fopen('data_c17_valid.txt','w');
for INDvalid = 1:ValidSize
    for INDfeat = 1:DIMfeat
        fprintf(fid,'%.9f',FeatValid(INDvalid,INDfeat));        
        if INDfeat == DIMfeat
            fprintf(fid,'\n');
        else
            fprintf(fid,' ');
        end
    end
    fprintf(fid,'%d\n',LabelValid(INDvalid));
end
fclose(fid);

fid = fopen('data_c17_test.txt','w');
for INDtest = 1:TestSize
    for INDfeat = 1:DIMfeat
        fprintf(fid,'%.9f',FeatTest(INDtest,INDfeat));
        if INDfeat == DIMfeat
            fprintf(fid,'\n');
        else
            fprintf(fid,' ');
        end
    end
    fprintf(fid,'%d\n',LabelTest(INDtest));
end
fclose(fid);
