global_paths;

load('histogram.mat');
data = DATA';
labels = LABEL';

taskName = 'spinhistograms';
set_task_parameters(taskName, 'hiddenLayers', [200 200 200], 'binaryInput', true, 'useRBMs', true, 'modelSelectDensity', 10, 'modelSelectKFolds', 4);
create_cross_validated_task(taskName, data, labels, 4);
stage_model_select_pretrain_old(taskName, false);

% stage_finetune_old('', true);
% stage_external_classify_raw_old('', true);