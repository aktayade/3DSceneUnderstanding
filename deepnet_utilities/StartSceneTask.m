global_paths;

load('home.mat');
labels = data(:,3)';
data = data(:,4:end)';

taskName = 'home';
set_task_parameters(taskName, 'hiddenLayers', [200 200 200], 'binaryInput', false, 'useRBMs', true, 'modelSelectDensity', 10, 'modelSelectKFolds', 4);
create_cross_validated_task(taskName, data, labels, 4);
stage_model_select_pretrain(taskName, false);



load('office.mat');
labels = data(:,3)';
data = data(:,4:end)';

taskName = 'office';
set_task_parameters(taskName, 'hiddenLayers', [200 200 200], 'binaryInput', false, 'useRBMs', true, 'modelSelectDensity', 10, 'modelSelectKFolds', 4);
create_cross_validated_task(taskName, data, labels, 4);
stage_model_select_pretrain(taskName, false);


% stage_finetune('', true);
% stage_external_classify_raw('', true);