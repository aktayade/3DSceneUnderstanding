taskName = 'test';
set_task_parameters(taskName, 'hiddenLayers', [20 20], 'binaryInput', false, 'useRBMs', true, 'modelSelectDensity', 2, 'modelSelectKFolds', 2);
create_cross_validated_task(taskName, rand(5,200), randi(4,1,200), 2, rand(5,50))
stage_model_select_pretrain(taskName, false);
stage_finetune(taskName, false);