global_paths;

load('home.mat');
labels = data(:,3)';
data = data(:,4:end)';

taskName = 'home';
set_task_parameters(taskName, 'hiddenLayers', [200 200 200], 'inputType', 'c', 'useRBMs', true, 'modelSelectDensity', 10, 'modelSelectKFolds', 4);
create_cross_validated_task(taskName, data, labels, 4, [], true);


load('office.mat');
labels = data(:,3)';
data = data(:,4:end)';

taskName = 'office';
set_task_parameters(taskName, 'hiddenLayers', [200 200 200], 'inputType', 'c', 'useRBMs', true, 'modelSelectDensity', 10, 'modelSelectKFolds', 4);
create_cross_validated_task(taskName, data, labels, 4, [], true);


% stage_model_select_pretrain('', true);
% stage_finetune('', true);
% stage_external_classify('', true);