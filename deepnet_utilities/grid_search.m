function [ optimal_parameters, performanceResults ] = grid_search( parameterSpace, performanceFunction, workFile, resumeWork, varargin )
% Author: Tyler Sanderson (tysand@umich.edu)
%GRIDSEARCH Performs a grid search and returns the maximizer
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
if nargin < 2
    error('Not enough arguments');
end

if ~iscell(parameterSpace)
    error('parameterSpace must be a cell array');
end

if size(parameterSpace,2) > 1
    error('paramterSpace must be dx1 cell array');
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

% Along each dimension from the current position, get the performance:
batchStart = 1;
batchEnd = 0;
if resumeWork
    load(workFile);
end
while batchEnd < numel(scores)
    % To work well in parallel, collect saveIterSteps samples of real work
    batchEnd = min(batchStart + saveIterSteps - 1, numel(scores));

    parfor ii=batchStart:batchEnd
        coords = coordsTemplate;
        paramsLocal = zeros(1,pdim);
        [ coords{:} ] = ind2sub(psizes, ii);
        % Get the parameter values for the given coordinates
        for d=1:pdim
            paramsLocal(d) = parameterSpace{d}(coords{d});
        end

        performanceResults{ii} = performanceFunction(paramsLocal, varargin{:});
        scores(ii) = performanceResults{ii}.score;
    end
    batchStart = batchEnd + 1;
    if ~isempty(workFile)
        save(workFile,'batchStart', 'batchEnd', 'scores', 'performanceResults', 'varargin');
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

% Set values for this position
for d=1:pdim
    params(d) = parameterSpace{d}(newposition(d));
end
optimal_parameters = params;
end

