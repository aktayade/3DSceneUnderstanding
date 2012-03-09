function [ sigmaJaak ] = Jaakkola( xtrain )
% Author: Tyler Sanderson (tysand@umich.edu)
%JAAKKOLA Returns suggested sigma value for data
        [~, d] = knnsearch(xtrain,xtrain,'k',2);
        sigmaJaak = mean(d(:,2));
end
