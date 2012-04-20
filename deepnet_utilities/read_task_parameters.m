function [task] = read_task_parameters( taskName )
%READ_TASK_PARAMETERS Returns task parameters for a given task
    fpath = generate_read_file_path({'tasks', 'parameters'}, [taskName '.mat']);
    if ~exist(fpath, 'file')
        error('Task parameters have not been created. Do so with set_task_parameters');
    end
    
    load(fpath);
end