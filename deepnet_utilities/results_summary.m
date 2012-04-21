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
    externalfinetuned = cell(0,8); % ft/pft(gsvm, lsvm, msvm, sm)
    externalfinetunedCount = 0;
    
    prospects = ls([rawData '*']);
    for ii = 1:size(prospects,1)
        prospects(ii,:)
        finetunedPath = generate_read_file_path({'tasks', 'output', 'finetuned'}, prospects(ii,:));
        if ~exist(finetunedPath, 'file')
            fprintf('Failed to open %s\n', finetunedPath);
        else
            load(finetunedPath);
            if strcmp(masterTaskName, taskName)
                finetuneCount = finetuneCount + 1;
                finetune{finetuneCount,1} = work;
            end
        end
        
        
        externalrawPath = generate_read_file_path({'tasks', 'output', 'externalraw'}, prospects(ii,:));
        if ~exist(externalrawPath, 'file')
            fprintf('Failed to open %s\n', externalrawPath);
        else
            load(externalrawPath);
            if strcmp(masterTaskName, taskName)
                externalrawCount = externalrawCount + 1;
                externalraw{externalrawCount,1} = gsvm;
                externalraw{externalrawCount,2} = lsvm;
                externalraw{externalrawCount,3} = msvm;
                externalraw{externalrawCount,4} = sm;
            end
        end
        externalfinetunedPath = generate_read_file_path({'tasks', 'output', 'externalfinetuned'}, prospects(ii,:));
        if ~exist(externalfinetunedPath, 'file')
            fprintf('Failed to open %s\n', externalfinetunedPath);
        else
            load(externalfinetunedPath);
            if strcmp(masterTaskName, taskName)
                externalfinetunedCount = externalfinetunedCount + 1;
                externalfinetuned{externalfinetunedCount,1} = ftgsvm;
                externalfinetuned{externalfinetunedCount,2} = ftlsvm;
                externalfinetuned{externalfinetunedCount,3} = ftmsvm;
                externalfinetuned{externalfinetunedCount,4} = ftsm;
                externalfinetuned{externalfinetunedCount,5} = pftgsvm;
                externalfinetuned{externalfinetunedCount,6} = pftlsvm;
                externalfinetuned{externalfinetunedCount,7} = pftmsvm;
                externalfinetuned{externalfinetunedCount,8} = pftsm;
            end
        end
    end
    
    if externalrawCount > 0
        display_externals(externalraw, performanceMeasure);
    end
    
    if finetuneCount > 0
        display_finetune(finetune, performanceMeasure);
    end
    
    if externalfinetunedCount > 0
        display_externalfinetuned(externalfinetuned, performanceMeasure);
    end

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

function display_externalfinetuned(externalfinetuned, performanceMeasure)
    for layer = 1:length(externalfinetuned{1,1})
        fprintf('---------------- LAYER %i ------------------\n\n', layer);
        ftgsvm = average_field_layer(externalfinetuned(:,1), performanceMeasure, layer)
        ftlsvm = average_field_layer(externalfinetuned(:,2), performanceMeasure, layer)
        ftmsvm = average_field_layer(externalfinetuned(:,3), performanceMeasure, layer)
        ftsm   = average_field_layer(externalfinetuned(:,4), performanceMeasure, layer)
        pftgsvm = average_field_layer(externalfinetuned(:,5), performanceMeasure, layer)
        pftlsvm = average_field_layer(externalfinetuned(:,6), performanceMeasure, layer)
        pftmsvm = average_field_layer(externalfinetuned(:,7), performanceMeasure, layer)
        pftsm   = average_field_layer(externalfinetuned(:,8), performanceMeasure, layer)
        max_ftgsvm = max(max(ftgsvm))
        max_ftlsvm = max(max(ftlsvm))
        max_ftmsvm = max(max(ftmsvm))
        max_ftsm   = max(max(ftsm))
        max_pftgsvm = max(max(pftgsvm))
        max_pftlsvm = max(max(pftlsvm))
        max_pftmsvm = max(max(pftmsvm))
        max_pftsm   = max(max(pftsm))
    end
end

function result = average_field_layer(cellstruct, field, layer)
    total = retrieve_cellstruct_field(cellstruct{1}{layer}, field);
    assert(size(cellstruct,2) == 1, 'Must be nx1 cell struct array');
    for ii = 2:size(cellstruct,1)
        total = total + retrieve_cellstruct_field(cellstruct{ii}{layer}, field);
    end
    result = total / size(cellstruct,1);
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

