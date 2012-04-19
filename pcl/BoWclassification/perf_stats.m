function [ accuracy, MP, MR ] = perf_stats( groundTruth, guesses, numClasses )
%PERF_STATS Returns accuracy, macro precision, macro recall
    accuracy = mean(groundTruth(:) == guesses(:));
    
    % Macro precision and recall:
    MRbuf = zeros(numClasses,1);
    MPbuf = zeros(numClasses,1);
    for c = 1:numClasses
        TP = sum(groundTruth(:)==c & guesses(:)==c);
        FP = sum(groundTruth(:)~=c & guesses(:)==c);
        FN = sum(groundTruth(:)==c & guesses(:)~=c);
        MRbuf(c) = TP/(TP+FN);
        MPbuf(c) = TP/(TP+FP);
    end
    MP = nanmean(MPbuf);
    MR = nanmean(MRbuf);
end

