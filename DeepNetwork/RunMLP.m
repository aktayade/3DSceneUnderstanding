

    [nonSpeakerFeatures, dataMean, dataStd] = StandardizeData(nonSpeakerFeatures);
    thisSpeakerFeatures = StandardizeData(features(speakerFilter,:), dataMean, dataStd);
    
    useRBM = true;
    hiddenLayers = [200 200];
    if useRBM
        pretrainType = 'RBM';
    else
        pretrainType = 'Autoencoder';
    end
    layerName = '';
    for ii = 1:length(hiddenLayers)
        layerName = [layerName sprintf('_%i', hiddenLayers(ii))];
    end
    mlp = MultiLayerPerceptron(sprintf('%s_%i_%s_%s', data_name, speaker, pretrainType, layerName), hiddenLayers, false, useRBM);
    
    trainData = nonSpeakerFeatures';
    trainLabels = labels(speakerFilter);
    testLabels = labels(~speakerFilter);
    mlp.Pretrain(trainData, false, [.1, .1], [3e-3 3e-3], [3 3]);
    mlp.TrainSoftmax(trainData, trainLabels);
    preFineTunePred = mlp.Predict(testData);
    prefinetuneacc = mean(testLabels(:) == preFineTunePred(:));
    mlp.FineTune(trainData, trainLabels, false);
    fprintf('Before Finetuning Test Accuracy: %0.3f%%\n', prefinetuneacc * 100);

    fineTunePred = mlp.Predict(testData);
    finetunedacc = mean(testLabels(:) == fineTunePred(:));
    fprintf('After Finetuning Test Accuracy: %0.3f%%\n', finetunedacc * 100);

    speaker_features{speaker} = ;
    speaker_labels{speaker} = labels(speakerFilter);
end

% Save the result
pretrained_name = sprintf('%s_SVM%i_features%i', data_name, K, pfa_feat_count);
save(['Data/' pretrained_name '.mat'], 'speaker_features', 'speaker_labels');
end