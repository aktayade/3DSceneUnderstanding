global_paths;

load('histogram.mat');
data = DATA';
labels = LABEL';

taskName = 'spinhistograms';
set_task_parameters(taskName, 'hiddenLayers', [400 300 200], 'inputType', 'p', 'useRBMs', true, 'modelSelectDensity', 10, 'modelSelectKFolds', 4);
create_cross_validated_task(taskName, data, labels, 4, [], false);

% stage_model_select_pretrain('', true);
% stage_finetune('', true);
% stage_external_classify('', true);