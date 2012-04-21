function [gsvm, lsvm, msvm, sm, parameters] = external_classifiers_performance(xtrain, ytrain, xtest, ytest, numClasses)
    global_paths;
    parameters.lambdaSpace = logspace(-4, 1, 10);
    parameters.sigmaSpace = logspace(-1, 4, 10);
    parameters.softmaxLambdaSpace = logspace(-4, 1, 5);
    
    % Various svms
    [~, msvm] = grid_search({parameters.lambdaSpace}, @multiSvmPerformance, '', false, xtrain', ytrain', xtest', ytest', numClasses);
    [~, gsvm] = grid_search({parameters.lambdaSpace; parameters.sigmaSpace}, @gaussianSvmPerformance, '', false, xtrain', ytrain', xtest', ytest', numClasses);
    [~, lsvm] = grid_search({parameters.lambdaSpace}, @linearSvmPerformance, '', false, xtrain', ytrain', xtest', ytest', numClasses);
    
    % Softmax
    [~, sm] = grid_search({parameters.softmaxLambdaSpace}, @softmaxPerformance, '', false, xtrain, ytrain, xtest, ytest, numClasses);
end

function result = gaussianSvmPerformance(params, xtrain, ytrain, xtest, ytest, numClasses)
    fprintf('Training Gaussian SVM with lambda(%f) sigma(%f)\n', params(1), params(2));
    c = 1/params(1);
    g = 1/(2*params(2)^2);
    
    assert(size(ytrain,1) > 1, 'training vector dimensions swapped');
    model = svmtrain(ytrain, xtrain, ['-c ' num2str(c) ' -g ' num2str(g) ' -m 1000']);
    [class] = svmpredict(ytest, xtest, model);
    
    [result.acc result.MP result.MR] = perf_stats(ytest, class, numClasses);
    result.score = result.acc;
end

function result = linearSvmPerformance(params, xtrain, ytrain, xtest, ytest, numClasses)
    fprintf('Training Linear SVM with lambda(%f)\n', params(1));
    c = 1/params(1);
    xtrain = sparse(xtrain);
    xtest = sparse(xtest);

    assert(size(ytrain,1) > 1, 'training vector dimensions swapped');
    model = train(ytrain, xtrain, ['-c ' num2str(c)]);
    [class] = predict(ytest, xtest, model);
    
    [result.acc result.MP result.MR] = perf_stats(ytest, class, numClasses);
    result.score = result.acc;
end

function result = multiSvmPerformance(params, xtrain, ytrain, xtest, ytest, numClasses)
    fprintf('Training MultiClass SVM with lambda(%f)\n', params(1));
    c = 1/params(1);

    assert(size(ytrain,1) > 1, 'training vector dimensions swapped');
    
    train_data_filename = tempname;
    test_data_filename = tempname;
    test_predict_filename = tempname;
    model_filename = tempname;
    
    f = fopen(train_data_filename, 'w');
    for ii = 1:size(xtrain,1)
        s = sprintf('%i', ytrain(ii));
        for jj = 4:size(xtrain,2)
            s = sprintf('%s %i:%f', s, jj-3, xtrain(ii,jj));
        end
        fprintf(f, '%s #\n', s);
    end
    fclose(f);
    
    f = fopen(test_data_filename, 'w');
    for ii = 1:size(xtest,1)
        s = sprintf('%i', ytest(ii));
        for jj = 4:size(xtest,2)
            s = sprintf('%s %i:%f', s, jj-3, xtest(ii,jj));
        end
        fprintf(f, '%s #\n', s);
    end
    fclose(f);

    % Hack for caen machines. Assumes executables have been copied to temp
    if strfind(pwd, '\\storage.adsroot.itcs.umich.edu')
        temppath = fileparts(tempname);
        system(sprintf('%s\\svm_multiclass_learn.exe -c %f %s %s', temppath, c, train_data_filename, model_filename));
        system(sprintf('%s\\svm_multiclass_classify.exe %s %s %s', temppath, test_data_filename, model_filename, test_predict_filename));
    else
        system(sprintf('.\\ExternalClassifiers\\svm_multiclass_learn.exe -c %f %s %s', c, train_data_filename, model_filename));
        system(sprintf('.\\ExternalClassifiers\\svm_multiclass_classify.exe %s %s %s', test_data_filename, model_filename, test_predict_filename));
    end
    
    predictions = load(test_predict_filename);
    class = predictions(:, 1);
    
    [result.acc result.MP result.MR] = perf_stats(ytest, class, numClasses);
    result.score = result.acc;
    
    delete(train_data_filename);
    delete(test_data_filename);
    delete(test_predict_filename);
    delete(model_filename);
end

function result = softmaxPerformance(params, xtrain, ytrain, xtest, ytest, numClasses)
    fprintf('Training softmax with parameter lambda(%f)\n', params(1));
    smmodel = softmaxClassifierTrain(numClasses, params(1), xtrain, ytrain, 400);
    pred = softmaxClassifierPredict(smmodel, xtest);
    [result.acc result.MP result.MR] = perf_stats(ytest, pred, numClasses);
    result.score = result.acc;
end