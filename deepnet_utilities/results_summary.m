function [ ] = results_summary( taskName, performanceMeasure )
%RESULTS_SUMMARY Loads data from various stages, summarizes it and displays
%   performanceMeasure can be: 'acc', 'MP', 'MR' for accuracy, macro
%   precision, macro recall
%   
    if ~exist('taskName', 'var') || isempty(taskName)
        error('Must give task name');
    end
    if ~exist('performanceMeasure', 'var')
        performanceMeasure = 'acc';
    end

    global_paths;
    rawData = generate_read_file_path({'tasks', 'output', 'taskData'}, taskName);
    finetune = cell(0,1);
    finetuneCount = 0;
    externalraw = cell(0,4); % gsvm, lsvm, msvm, sm
    externalrawCount = 0;
    
    prospects = ls([rawData '*']);
    for ii = 1:size(prospects,1)
        finetunedPath = generate_read_file_path({'tasks', 'output', 'finetuned'}, prospects(ii,:));
        load(finetunedPath);
        if strcmp(masterTaskName, taskName)
            finetuneCount = finetuneCount + 1;
            finetune{finetuneCount,1} = work;
        end
        prospects(ii,:)
        externalrawPath = generate_read_file_path({'tasks', 'output', 'externalraw'}, prospects(ii,:));
        load(externalrawPath);
        if strcmp(masterTaskName, taskName)
            externalrawCount = externalrawCount + 1;
            externalraw{externalrawCount,1} = gsvm;
            externalraw{externalrawCount,2} = lsvm;
            externalraw{externalrawCount,3} = msvm;
            externalraw{externalrawCount,4} = sm;
        end
    end
    
    display_externals(externalraw, performanceMeasure);
    display_finetune(finetune, performanceMeasure);

end


function display_externals(externalraw, performanceMeasure)
    gsvm = average_field(externalraw(:,1), performanceMeasure)
    lsvm = average_field(externalraw(:,2), performanceMeasure)
    msvm = average_field(externalraw(:,3), performanceMeasure)
    sm = average_field(externalraw(:,4), performanceMeasure)
    max_gsvm = max(max(gsvm))
    max_lsvm = max(max(lsvm))
    max_msvm = max(max(msvm))
    max_sm   = max(max(sm))
end


function display_finetune(finetune, performanceMeasure)
    performanceMeasure(1) = upper(performanceMeasure(1));
    pft_finetune = average_field(finetune, ['pft' performanceMeasure])
    ft_finetune = average_field(finetune, ['ft' performanceMeasure])
    max_pft_finetune = max(max(pft_finetune))
    max_ft_finetune = max(max(ft_finetune))
end

function result = average_field(cellstruct, field)
    total = retrieve_cellstruct_field(cellstruct{1}, field);
    assert(size(cellstruct,2) == 1, 'Must be nx1 cell struct array');
    for ii = 2:size(cellstruct,1)
        total = total + retrieve_cellstruct_field(cellstruct{ii}, field);
    end
    result = total / size(cellstruct,1);
end

