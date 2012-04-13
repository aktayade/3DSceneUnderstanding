function [ optimal_parameters, shownWork ] = ...
    lineSearch( startPoint, data, labels, param_space, kfold_indices, GetPerformance, passthrough )
% Author: Tyler Sanderson (tysand@umich.edu)
%LINESEARCH Performs a line search and returns the minimizer
%   startPoint - 1xm matrix containing the search start parameters
%   data - training data to pass to GetPerformance()
%   labels - training labels to pass to GetPerformance()
%   delta - A 1xm matrix containing the step sizes for the search
%   lowerbound/upperbound - 1xm matrix containing search-space bound
%   KFolds - Passed through to GetPerformance(), determines kfold_indicies
%   GetPerformance() - a function which returns the error value at a point
%       on the search space given the point.  Expected to take the
%       following parameters in order:
%           parameters - 1xm matrix containing the parameters to use
%           data - the data passed through from LineSearch
%           labels - the labels for the data passed through from LineSearch
%           KFolds - Passed through from LineSearch
%           kfold_indicies - Indicies generated for kfold cross validation
%               will be a nx1 matrix with values between 1 and KFolds
%               indicating which fold a datapoint is in.
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
err_grid = repmat(bitmax,sz);
MP_grid = zeros(sz);
MR_grid = zeros(sz);

nel = numel(err_grid);
coordsTemplate = cell(1,pdim);
KFolds = max(kfold_indices);

% Calculate initial position
positioni = zeros(1,pdim);
params = zeros(1,pdim);
for d=1:pdim
    pspace = cell2mat(param_space(d));
    [~,c] = min(abs(pspace - startPoint(d)));
    positioni(d) = c;
    params(d) = pspace(c);
end

% Start point shift, tries to center around the given initial startpoint:
% Classify each dimension of param_space as linear, logrithmic or unknown
% 0 = unknown, 1 = linear, 2 = logrithmic
dtype = zeros(pdim,1);
SP_shift = zeros(pdim,1);
for d = 1:pdim
    type = ones(1,2);
    for t = 1:2
        pspace = cell2mat(param_space(d));
        if length(pspace) == 1
            break;
        end
        if (t == 2)
            pspace = log10(pspace);
        end
        v = pspace(2) - pspace(1);
        for ii = 1:(sz(d)-1)
            %if the difference is more than 0.1% error then different
            if (abs((pspace(ii+1) - pspace(ii)) - v)) > .001 * abs(v)
                type(t) = 0;
            end
        end
    end
    if(type(1) && sz(d)>2)
        dtype(d) = 1;
        SP_shift(d) = startPoint(d) - params(d);
    elseif(type(2) && sz(d)>2)
        dtype(d) = 2;
        SP_shift(d) = log10(startPoint(d)) - log10(params(d));
    else
        dtype(d) = 0;
        SP_shift(d) = 0;
    end
end

changed = true;
while(changed)
    % Along each dimension from the current position, get the performance:
    parfor ii=1:nel
        coords = coordsTemplate;
        paramsLocal = zeros(1,pdim);
        [ coords{:} ] = ind2sub(sz, ii);
        if (sum([coords{:}] == positioni) >= (pdim-1)) && (err_grid(ii) == bitmax)
            % Get the parameter values for the given coordinates
            c = cell2mat(coords);
            for d=1:pdim
                pspace = cell2mat(param_space(d));
                paramsLocal(d) = pspace(c(d));
                % If applicable, apply a shift to center on startPoint
                switch dtype(d)
                    case 1
                        paramsLocal(d) = paramsLocal(d) + SP_shift(d);
                    case 2
                        paramsLocal(d) = 10^(log10(paramsLocal(d)) + SP_shift(d));
                end
            end
            
            % Get the performance (cross-validation)
            kfold_accum = ones(KFolds, 1);
            for k = 1:KFolds
                test = (kfold_indices == k); train = ~test;
                kfold_accum(k) = GetPerformance(data(:,train),labels(train),data(:,test),labels(test), paramsLocal, passthrough);
            end
            err_grid(ii) = mean(kfold_accum);
        end
    end   
    
    % Select the minimum point for all dimensions:
    mxA = min(err_grid(:));
    idx0 = find(err_grid == mxA);
    clear idx
    [idx{1:pdim}] = ind2sub(size(err_grid),idx0);
    idx = cat(2,idx{:});
    newposition = idx(1,:);
    changed = sum(positioni ~= newposition);
    positioni = newposition;
end

% Set values for this position
for d=1:pdim
    pspace = cell2mat(param_space(d));
    params(d) = pspace(newposition(d));
    % If applicable, apply a shift to center on startPoint
    switch dtype(d)
        case 1
            params(d) = params(d) + SP_shift(d);
        case 2
            params(d) = 10^(log10(params(d)) + SP_shift(d));
    end
end
optimal_parameters = params;
effort = sum(err_grid(:) < bitmax);
shownWork = err_grid;
end

