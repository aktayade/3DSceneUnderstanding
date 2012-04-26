% Optimal sparsity: 0.0428
% Optimal lambda for finetuning: 0.0018
% Beta: 10
% Lambda for pretraining = 0.001;
global_paths;

load('data_office_rbm_concat_unm.mat');
data = DATA';
labels = LABEL';

% Scale data to be poisson RBM safe
scaleFactor = 100/max(data(:));
data = data * scaleFactor;

mlp = MultiLayerPerceptron('OfficeSpinRbm400PRBM', 400, 17, 'p', true, true);
mlp.Pretrain(data, false, .0428, .001, 10);
mlp.TrainSoftmax(data, labels, .0018);
mlp.FineTune(data, labels, .0018, false);

save('OfficeSpinRbm400PRBMClassifier.mat', 'mlp', 'scaleFactor');