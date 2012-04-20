function [ ] = update_task_parameters( taskName, varargin )
%UPDATE_TASK_PARAMETERS Appends a paramter to existing task parameters
%   This function should only be used INFREQUENTLY by automated processes
%   there is a danger of overwriting a parallel process's data

    task = read_task_parameters(taskName);
    
    for ii = 1:2:length(varargin)
        task.(varargin{ii}) = varargin{ii+1};
    end
    
    fpath = generate_write_file_path({'tasks', 'parameters'}, taskName);
    save(fpath, 'task');
end