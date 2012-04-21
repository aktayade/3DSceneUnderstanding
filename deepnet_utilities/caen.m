temppath = fileparts(tempname);
copyfile('ExternalClassifiers\svm_multiclass_learn.exe', sprintf('%s\\svm_multiclass_learn.exe', temppath));
copyfile('ExternalClassifiers\svm_multiclass_classify.exe', sprintf('%s\\svm_multiclass_classify.exe', temppath));

matlabpool 4
while true
    stage_model_select_pretrain('', false);
    stage_finetune('', false);
    stage_external_classify('', false);
    pause(20);
end