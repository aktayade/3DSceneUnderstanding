global_paths;

threshold = .5;

load('OfficeSpinRbm400PRBMClassifier.mat');
%load('OfficeSpinRbmData.mat');
data = DATA_unm' * scaleFactor;
labels = mlp.Predict(data, threshold);
%save('OfficeSpinRbmPrediction.out', 'labels', '-ascii');