taskName = 'splittest';
set_task_parameters(taskName, 'hiddenLayers', [100 40], 'inputType', 'cpb', 'useRBMs', true, 'modelSelectDensity', 2, 'modelSelectKFolds', 2,...
                    'splitInputSizes', [5 10 20], 'splitHiddenSizes', [20 30 50]);
xtrain = [randn(5, 200); randi(50, 10, 200); rand(20, 200)];
unlabeled = [randn(5, 30); randi(50, 10, 30); rand(20, 30)];
create_cross_validated_task(taskName, xtrain, randi(4,1,200), 2, unlabeled, true)
stage_model_select_pretrain(taskName, false);
%stage_finetune(taskName, false);
%stage_external_classify(taskName, false);
%results_summary(taskName, 'acc');
%results_summary(taskName, 'MP');
%results_summary(taskName, 'MR');


%taskName = 'test';
%set_task_parameters(taskName, 'hiddenLayers', [20 20], 'inputType', 'c', 'useRBMs', true, 'modelSelectDensity', 2, 'modelSelectKFolds', 2);
%create_cross_validated_task(taskName, rand(5,200), randi(4,1,200), 2, rand(5,50), true)
%stage_model_select_pretrain(taskName, false);
%stage_finetune(taskName, false);
%stage_external_classify(taskName, false);
%results_summary(taskName, 'acc');
%results_summary(taskName, 'MP');
%results_summary(taskName, 'MR');