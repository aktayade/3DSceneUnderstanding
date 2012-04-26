global_paths;

load('data_office_kmeans_unm.mat');
data = DATA';
labels = LABEL';

% Scale data to be poisson RBM safe
sf = 100/max(data(:));
data = data * sf;

taskName = 'office-spinhistograms';
set_task_parameters(taskName, 'hiddenLayers', [1000 750 500], 'inputType', 'p', 'useRBMs', true, 'modelSelectDensity', 10, 'modelSelectKFolds', 4);
create_cross_validated_task(taskName, data, labels, 4, [], false);

% stage_model_select_pretrain('', true);
% stage_finetune('', true);
% stage_external_classify('', true);