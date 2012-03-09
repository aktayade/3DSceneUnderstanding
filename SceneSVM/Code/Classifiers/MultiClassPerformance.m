function [ risk_out, accuracy, classifier, macroPrecision, macroRecall ] = MultiClassPerformance(train, label_train, test, label_test, parameters, passthrough)
%This function returns the risk given training and test data and a set of
%parameters.  Passthrough is the Alpha value

    fprintf('Training MultiClass SVM with lambda(%f)\n', parameters(1));
    c = 1/parameters(1);

    assert(size(label_train,1) > 1, 'training vector dimensions swapped');
    
    train_data_filename = tempname;
    test_data_filename = tempname;
    test_predict_filename = tempname;
    model_filename = tempname;
    
    f = fopen(train_data_filename, 'w');
    for ii = 1:size(train,1)
        s = sprintf('%i', label_train(ii));
        for jj = 4:size(train,2)
            s = sprintf('%s %i:%f', s, jj-3, train(ii,jj));
        end
        fprintf(f, '%s #\n', s);
    end
    fclose(f);
    
    f = fopen(test_data_filename, 'w');
    for ii = 1:size(test,1)
        s = sprintf('%i', label_test(ii));
        for jj = 4:size(test,2)
            s = sprintf('%s %i:%f', s, jj-3, test(ii,jj));
        end
        fprintf(f, '%s #\n', s);
    end
    fclose(f);

    system(sprintf('svm_multiclass_learn.exe -c %f %s %s', c, train_data_filename, model_filename));
    system(sprintf('svm_multiclass_classify.exe %s %s %s', test_data_filename, model_filename, test_predict_filename));
    
    predictions = load(test_predict_filename);
    class = predictions(:, 1);
    
    accuracy = sum(class == label_test)/length(label_test)
    risk_out = 1-accuracy;
    
    % Macro precision and recall:
    class_count = length(unique(label_test));
    MR = zeros(class_count,1);
    MP = zeros(class_count,1);
    classes = unique(label_test);
    for ii = 1:class_count
        c = classes(ii);
        TP = size(find(label_test==c & class==c),1);
        FP = size(find(label_test~=c & class==c),1);
        FN = size(find(label_test==c & class~=c),1);
        MR(ii) = TP/(TP+FN);
        MP(ii) = TP/(TP+FP);
    end
    macroPrecision = nanmean(MP);
    macroRecall = nanmean(MR);

    classifier = 0;
    
    delete(train_data_filename);
    delete(test_data_filename);
    delete(test_predict_filename);
    delete(model_filename);
end
