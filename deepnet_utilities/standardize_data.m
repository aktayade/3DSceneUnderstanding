function [data,dataMean,dataStd] = standardize_data(data,dataMean,dataStd)
% STANDARDIZE_DATA Will normalize and center a dataset.
%   Optionally, give it a mean and std to use to standardize the
%   data.
    if (nargin < 2)
        dataMean = mean(data,2);
        dataStd = std(data,0,2);
    end
    n = size(data,2);
    data = (data - repmat(dataMean, 1, n)) ./ repmat(dataStd, 1, n);
    data(isnan(data)) = 0;
end

