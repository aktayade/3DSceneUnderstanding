function [path] = generate_read_file_path(folders, file, varargin)
%GENERATE_READ_FILE_PATH Standardizes the file system layout
% folders is a cell array with each entry being a folder layer
% Extra variable arguments will be appended to the filename with -
% for example myfile-arg1-arg2.mat
%
    if isa(folders, 'cell') && ~isempty(folders)
        folderPath = folders{1};
        for ii = 2:length(folders)
            folderPath = [folderPath '\' folders{ii}];
        end
        folderPath = [folderPath '\'];
    elseif isempty(folders)
        folderPath = '';
    else
        folderPath = [folders '\'];
    end

    finalFileName = file;
    for ii = 1:length(varargin)
        if isa(varargin{ii}, 'char')
            finalFileName = [finalFileName '-' varargin{ii}];
        else
            finalFileName = [finalFileName '-'];
            for rr = 1:length(varargin{ii})
                finalFileName = [finalFileName num2str(varargin{ii}(rr)) '_'];
            end
            finalFileName = finalFileName(1:(end-1));
        end
    end
    path = [folderPath finalFileName];
end