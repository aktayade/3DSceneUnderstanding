matlabpool 4
while true
    stage_model_select_pretrain('', false);
    stage_finetune('', false);
    stage_external_classify('', false);
    pause(20);
end