function [ ] = find_work( callback, taskName, stayActive, target, searchPlace, requiredRead, antirequiredRead )
%FIND_WORK Searches a target directory for work and makes a callback to
%complete the work found
%   callback is the work function to be called
%       callback receives loadPaths consisting of paths to the files, in
%       order of: searchPlace file/target file, requiredRead files,
%       antirequiredRead files.
%   taskName is the base stem name of files to find
%   if stayActive is true then will run indefinitely looking for work
%   if target is non-empty then only that specific file will be searched
%   searchPlace is the target directory to be searched. Cell array format.
%   requiredRead is a cell array of cell arrays containing directories that
%       are searched for files that need to exist
%   antirequiredRead is similar but is searched for files that cannot exist
%
    loadPaths = cell(length(requiredRead) + length(antirequiredRead)+ 1, 1);
    
    if ~isempty(target)        
        loadPaths{1} = generate_read_file_path(searchPlace, target);
        if ~exist(loadPaths{1}, 'file')
            fprintf('Target file not found: %s\n', loadPaths{1});
            return;
        end
        for m = 1:length(requiredRead)
            loadPaths{m+1} = generate_read_file_path(requiredRead{m}, target);
            if ~exist(loadPaths{m+1}, 'file')
                fprintf('Required file not found: %s\n', loadPaths{m+1});
                return;
            end
        end
        n = length(requiredRead);
        for m = 1:length(antirequiredRead)
            loadPaths{m+n+1} = generate_write_file_path(antirequiredRead{m}, target);
            if exist(loadPaths{m+n+1}, 'file')
                fprintf('Conflicting file exists, cannot continue: %s\n', loadPaths{m+n+1});
                return;
            end
        end
        load(loadPaths{1});
        if ~strcmp(masterTaskName, taskName)
            fprintf('masterTaskName in file does not match task name\n');
            return;
        end
        callback(loadPaths, taskName);
        return;
    end
    
    searchStem = generate_read_file_path(searchPlace, taskName);
    fprintf('Looking for data with stem: %s\n', searchStem);
    
    if isempty(taskName)
        prospectStart = 3;
    else
        prospectStart = 1;
    end
    
    while true % Run indefinitely
        prospects = ls([searchStem '*']);
        for ii = prospectStart:size(prospects,1)
            ok = true;
            loadPaths{1} = generate_read_file_path(searchPlace, prospects(ii,:));
            for m = 1:length(requiredRead)
                loadPaths{m+1} = generate_read_file_path(requiredRead{m}, prospects(ii,:));
                if ~exist(loadPaths{m+1}, 'file')
                    ok = false;
                end
            end
            n = length(requiredRead);
            for m = 1:length(antirequiredRead)
                loadPaths{m+n+1} = generate_write_file_path(antirequiredRead{m}, prospects(ii,:));
                if exist(loadPaths{m+n+1}, 'file')
                    ok = false;
                end
            end
            load(loadPaths{1});
            if ~strcmp(masterTaskName, taskName) && ~isempty(taskName)
                ok = false;
            end
            if ok
                callback(loadPaths, taskName);
                fprintf('Work completed!  Looking for more work...\n');
            end
        end
        
        if ~stayActive
            break;
        end
        
        pause(30);
    end
end




