function [ optimal_parameters, effort, shownWork, MP_grid, MR_grid ] = ...
    GridSearch( data, labels, param_space, kfold_indices, GetPerformance, passthrough )
% Author: Tyler Sanderson (tysand@umich.edu)
%GRIDSEARCH Performs a grid search and returns the minimizer
%   data - training data to pass to GetPerformance()
%   labels - training labels to pass to GetPerformance()
%   param_space - Parameter values cell-matrix
%   KFolds - Passed through to GetPerformance(), determines kfold_indicies
%   GetPerformance() - a function which returns the error value at a point
%       on the search space given the point.  Expected to take the
%       following parameters in order:
%           training data
%           training labels
%           testing data
%           testing labels
%           parameters - 1xm matrix containing the parameters to use
%           passthrough (extra parameters)
%   passthrough - Extra params to pass to the Performance function
% Returns - An 1xm matrix containing minimizing parameters, also the
%           'effort' as measured by the number of search points looked at

if (~iscell(param_space))
    throw(MException('Input:NotCellArray', ...
        'param_space must be cell array'));
end

pdim = size(param_space,1);

sz = zeros(1,pdim);
for i=1:pdim
    sz(i) = size(cell2mat(param_space(i)), 2);
end
if (pdim == 1)
    sz = [sz 1];
end
err_grid = zeros(sz);
MP_grid = zeros(sz);
MR_grid = zeros(sz);

nel = numel(err_grid);
coords = cell(1,ndims(err_grid));
params = zeros(1, ndims(sz));
KFolds = max(kfold_indices);
kfold_accum = zeros(1, KFolds);
MP_accum = zeros(1, KFolds);
MR_accum = zeros(1, KFolds);
for ii=1:nel
    % Get the coordinates and corresponding parameters
    [ coords{:} ] = ind2sub(sz, ii);
    c = cell2mat(coords);
    for d=1:pdim
        pspace = cell2mat(param_space(d));
        params(d) = pspace(c(d));
    end
    
    for k = 1:KFolds
        test = (kfold_indices == k); train = ~test;
        [kfold_accum(k),~,~,MP_accum(k),MR_accum(k)] = GetPerformance(data(train,:),labels(train),data(test,:),labels(test), params, passthrough);
    end
    
    err_grid(ii) = mean(kfold_accum);
    MP_grid(ii) = mean(MP_accum);
    MR_grid(ii) = mean(MR_accum);
end

% Select the minimum point for all dimensions:
mxA = min(err_grid(:));
idx0 = find(err_grid == mxA);
clear idx
[idx{1:ndims(err_grid)}] = ind2sub(size(err_grid),idx0);
idx = cat(2,idx{:});
position = idx(1,:);

optimal_parameters = zeros(1,pdim);
for d=1:pdim
    pspace = cell2mat(param_space(d));
    optimal_parameters(d) = pspace(position(d));
end

effort = nel;
shownWork = err_grid;

end

