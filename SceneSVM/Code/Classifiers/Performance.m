function [ risk_out, accuracy, classifier, macroPrecision, macroRecall ] = Performance(train, label_train, test, label_test, parameters, passthrough)
%This function returns the risk given training and test data and a set of
%parameters.  Passthrough is the Alpha value
    fprintf('Training Gaussian SVM with lambda(%f) sigma(%f)\n', parameters(1), parameters(2));
    c = 1/parameters(1);
    g = 1/(2*parameters(2)^2);
    
    %%%%%%%%%% Standard SVM %%%%%%%%%%%%
    string=['-c ' num2str(c) ' -g ' num2str(g) ' -m 1000'];
    assert(size(label_train,1) > 1, 'training vector dimensions swapped');
    model = svmtrain(label_train, train, string);
    %%% test the classifier 
    [class, accuracy, dec_values] = svmpredict(label_test, test, model);
    accuracy = accuracy(1);
    risk_out = 1-(accuracy/100.0);
    
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

    classifier = model;
end
