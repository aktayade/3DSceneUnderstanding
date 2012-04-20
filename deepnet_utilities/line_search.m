function [ optimal_parameters, performanceResults ] = line_search( startPoint, parameterSpace, performanceFunction, workFile, resumeWork, varargin )
% Author: Tyler Sanderson (tysand@umich.edu)
%LINESEARCH Performs a line search and returns the maximizer
%   startPoint - 1xm matrix containing the search start parameters
%   parameterSpace - dx1 cell array containing 1xm matricies with
%       parameters to try. d is the number of parameters
%   performanceFunction() - a function returning a structure containing a
%       positive valued field named 'score'. This score is maximized. The
%       function is expected to take a 1xm parameter matrix as it's first
%       argument. Any additional parameters passed to line_search are
%       passed on to this function.
%   workFile - path to file to save intermediate work to
%   resumeWork - true if should load intermediate work
% Returns - An 1xm matrix containing maximizing parameters, also the scores
%           of all tried values in a matrix.

% Sanitize input
if nargin < 3
    error('Not enough arguments');
end

if ~iscell(parameterSpace)
    error('parameterSpace must be a cell array');
end

if size(parameterSpace,2) > 1
    error('paramterSpace must be dx1 cell array');
end

if size(startPoint,1) > 1
    error('startPoint should be 1xm');
end

if size(startPoint,2) ~= size(parameterSpace,1)
    error('Start point should be same dimension as parameter space');
end

if ~exist('workFile', 'var')
    workFile = ''; end;
if ~exist('resumeWork', 'var')
    resumeWork = false; end;

% Initialize parameters
pdim = numel(parameterSpace);
psizes = zeros(1,pdim);
for d=1:pdim
    psizes(d) = size(parameterSpace{d}, 2);
end
if (pdim == 1) % Stops zeros(psizes) producing a square matrix
    psizes = [psizes 1];
end
scores = zeros(psizes);
performanceResults = cell(psizes);
coordsTemplate = cell(1,pdim);

% Calculate initial position
positioni = zeros(1,pdim);
params = zeros(1,pdim);
for d=1:pdim
    [~,c] = min(abs(parameterSpace{d} - startPoint(d)));
    positioni(d) = c;
    params(d) = parameterSpace{d}(c);
end

% Start point shift, tries to center around the given initial startpoint:
% Classify each dimension of param_space as linear, logrithmic or unknown
% 0 = unknown, 1 = linear, 2 = logrithmic
dtype = zeros(pdim,1);
SP_shift = zeros(pdim,1);
for d = 1:pdim
    if psizes(d) == 1 % Handle singleton dimensions as unknown type
        if startPoint(d) ~= parameterSpace{d}
            error('Singleton dimension in parameter space does not match start point value');
        end
        dtype(d) = 0;
        SP_shift(d) = 0;
        continue;
    end
    type = true(2,1);
    for t = 1:2 % Try linear and logrithmic
        pspace = parameterSpace{d};
        if (t == 2)
            pspace = log10(pspace);
        end
        v = pspace(2) - pspace(1);
        for ii = 1:(psizes(d)-1) % Check each point in the param space for a match
            %if the difference is more than 0.1% error then different
            if (abs((pspace(ii+1) - pspace(ii)) - v)) > .001 * abs(v)
                type(t) = false;
            end
        end
    end
    if type(1) && psizes(d) > 2 % If linear
        dtype(d) = 1;
        SP_shift(d) = startPoint(d) - params(d);
    elseif type(2) && psizes(d) > 2 % If logrithmic
        dtype(d) = 2;
        SP_shift(d) = log10(startPoint(d)) - log10(params(d));
    else % Otherwise unknown
        dtype(d) = 0;
        SP_shift(d) = 0;
    end
end

% Create the path to the intermediate work file if it doesn't exist
dirpath = fileparts(workFile);
if ~isempty(workFile) && ~exist(dirpath,'dir')
    mkdir(dirpath);
end

if exist('matlabpool', 'file')
    saveIterSteps = max(1, matlabpool('size'));
else
    saveIterSteps = 1;
end

changed = true;
while(changed)
    % Along each dimension from the current position, get the performance:
    batchStart = 1;
    batchEnd = 0;
    if resumeWork
        load(workFile);
        resumeWork = false;
    end
    while batchEnd < numel(scores)
        % To work well in parallel, collect saveIterSteps samples of real work
        workCount = 0;
        while workCount < saveIterSteps && batchEnd < numel(scores)
            batchEnd = batchEnd + 1;
            coords = coordsTemplate;
            [ coords{:} ] = ind2sub(psizes, batchEnd);
            if (sum([coords{:}] == positioni) >= (pdim-1)) && (scores(batchEnd) == 0)
                workCount = workCount + 1;
            end
        end
    
        parfor ii=batchStart:batchEnd
            coords = coordsTemplate;
            paramsLocal = zeros(1,pdim);
            [ coords{:} ] = ind2sub(psizes, ii);
            if (sum([coords{:}] == positioni) >= (pdim-1)) && (scores(ii) == 0)
                % Get the parameter values for the given coordinates
                c = cell2mat(coords);
                for d=1:pdim
                    paramsLocal(d) = parameterSpace{d}(c(d));
                    % If applicable, apply a shift to center on startPoint
                    switch dtype(d)
                        case 1
                            paramsLocal(d) = paramsLocal(d) + SP_shift(d);
                        case 2
                            paramsLocal(d) = 10^(log10(paramsLocal(d)) + SP_shift(d));
                    end
                end

                performanceResults{ii} = performanceFunction(paramsLocal, varargin{:});
                scores(ii) = performanceResults{ii}.score;
            end
        end
        batchStart = batchEnd + 1;
        if ~isempty(workFile)
            save(workFile,'batchStart', 'batchEnd', 'scores', 'performanceResults', 'positioni', 'varargin');
        end
    end
    % Select the max score for all dimensions:
    mxA = max(scores(:));
    idx0 = find(scores == mxA);
    idx0 = idx0(1);
    clear idx
    [idx{1:pdim}] = ind2sub(size(scores),idx0);
    idx = cat(2,idx{:});
    newposition = idx(1,:);
    changed = any(positioni ~= newposition);
    positioni = newposition;
end

% Set values for this position
for d=1:pdim
    params(d) = parameterSpace{d}(newposition(d));
    % If applicable, apply a shift to center on startPoint
    switch dtype(d)
        case 1
            params(d) = params(d) + SP_shift(d);
        case 2
            params(d) = 10^(log10(params(d)) + SP_shift(d));
    end
end
optimal_parameters = params;
end

