function [ task ] = set_task_parameters( taskName, varargin )
%SET_TASK_PARAMETERS Sets variables for a task
%   The needed parameters are task specific. See the code files for
%   each stage to determine which need to be set.
%   taskName is the name of the task to set parameters for
%   Remaining arguments follow: 'varname1', value1, 'varname2', value2

    % validate input
    for ii = 1:length(varargin)
        if mod(ii, 2) == 1 && ~isa(varargin{ii}, 'char')
            error('Parameter names must be strings');
        end
    end
    
    if mod(length(varargin),2) == 1
        error('Parameters must be given values');
    end

    fpath = generate_write_file_path({'tasks', 'parameters'}, taskName);
    
    task = struct;
    
    if exist(fpath, 'file')
        acceptableAnswer = false;
        while ~acceptableAnswer
            a = input('Task parameters already exist, (O)verwrite, (U)pdate, or (C)ancel?\n', 's');
            switch(lower(a(1)))
                case 'o'
                    acceptableAnswer = true;
                case 'u'
                    load(fpath);
                    acceptableAnswer = true;
                case 'c'
                    fprintf('No action taken\n');
                    return;
            end
        end
    end
    
    for ii = 1:2:length(varargin)
        task.(varargin{ii}) = varargin{ii+1};
    end
    
    save(fpath, 'task');
end

