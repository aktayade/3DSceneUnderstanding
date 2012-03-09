function [ risk_out, accuracy, classifier, macroPrecision, macroRecall ] = LinearPerformance(data_train, label_train, data_test, label_test, parameters, passthrough)
%LINEARPERFORMANCE Performance function using LibLinear
%   Uses a linear kernel, good for very large datasets (speed)

    data_train = sparse(data_train);
    data_test = sparse(data_test);

    c = 1/parameters(1); %c=1/lambda/size(label_train,1);
    %%%%%%%%%% Standard SVM %%%%%%%%%%%%
    string=['-c ' num2str(c)]
    assert(size(label_train,1) > 1, 'training vector dimensions swapped');
    model = train(label_train, data_train, string);
    [class, accuracy, dec_values] = predict(label_test, data_test, model);
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

