temppath = fileparts(tempname);
copyfile('ExternalClassifiers\svm_multiclass_learn.exe', sprintf('%s\\svm_multiclass_learn.exe', temppath));
copyfile('ExternalClassifiers\svm_multiclass_classify.exe', sprintf('%s\\svm_multiclass_classify.exe', temppath));

while true
    matlabpool 4
    stage_model_select_pretrain('', false);
    matlabpool close
    stage_finetune('', false);
    matlabpool 4
    stage_external_classify('', false);
    matlabpool close
    pause(60)
end