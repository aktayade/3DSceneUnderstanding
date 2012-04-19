function LISTclass = InputFeature(rawdata_dir, spinimage_dir, mapping_file)

load(mapping_file);
mapping = mapping';
maxlinelen = 10000;

LISTclass = struct('fnames', [], 'fnum', []);
LISTclass.fnames = struct('name',[]);

%spinimage_dir = 'cache/office/spin_images';
NUMclass = max(mapping(2,:));
AllLabels = (1:NUMclass)';
for i = 1:length(AllLabels)
    tl = num2str(AllLabels(i)+100);
    tmp_name = fullfile(spinimage_dir,['label_' tl(2:end)]);
    if exist(tmp_name,'dir') == 0
        mkdir(tmp_name);
    end
end

%rawdata_dir = '../featdata/rawfeat_office/';
LISTscene = dir(rawdata_dir);
LISTscene = LISTscene(3:end);  % Remove '.' and '..'
NUMscene = length(LISTscene);

AllScene_NUMseg = zeros(size(AllLabels));
AllScene_NUMdiscard = 0;
AllScene_NUMdesc = zeros(size(AllLabels));

for INDscene = 1:NUMscene

    name_raw_data = fullfile(rawdata_dir, LISTscene(INDscene).name);
    fprintf('Loading features from %s...\n\n',name_raw_data);
    fid = fopen(name_raw_data);
    lines = textscan(fid, '%s', 'delimiter', '\n', 'bufsize', maxlinelen);
    fclose(fid);
    
    lines = lines{1};
    NUMline = size(lines,1);  % Get number of segments
    
    NUMseg = zeros(size(AllLabels));
    NUMdiscard = 0;
    NUMdesc = zeros(size(AllLabels));
    for INDline = 1:NUMline

        tmp_line = lines{INDline};

        if tmp_line(1) ~= '(';
            
            FLAGskip = 0;
            FLAGskip_print = 0;
            FLAGerrlabel = 0;
            FLAGerrlabel_print = 0;

            tmp_feat.label = [];
            tmp_feat.data = [];
            tmp_feat.NUMdesc = [];

            tmp_label = str2double(tmp_line);  % Get label
            
            if sum(mapping(1,:)==tmp_label) ~= 0
                conv_label = mapping(2,mapping(1,:)==tmp_label);  % Convert the label
            else
                conv_label = 0;
            end
            
            NUMseg(AllLabels==conv_label) = NUMseg(AllLabels==conv_label)+1;
            AllScene_NUMseg(AllLabels==conv_label) = AllScene_NUMseg(AllLabels==conv_label)+1;

            if sum(AllLabels==conv_label) == 0            
                NUMdiscard = NUMdiscard+1;  % Report incorrect label
                fprintf('Segment %4d  Label: %3d  ', sum(NUMseg)+NUMdiscard, tmp_label);
                fprintf('Accu:  NaN  ');
                
                FLAGerrlabel = 1;
                continue;
            else
                fprintf('Segment %4d  Label: %3d  ', sum(NUMseg)+NUMdiscard, tmp_label);
                fprintf('Accu: %4d  ', NUMseg(AllLabels==conv_label));
            end

            tl = num2str(conv_label+100);
            ts = num2str(AllScene_NUMseg(AllLabels==conv_label)+10000);
            tmp_name = fullfile(spinimage_dir,['label_' tl(2:end)],['seg_' ts(2:end) '.mat']);

            if(exist(tmp_name,'file')~=0)
                FLAGskip = 1;
                continue;
            end
            
            tmp_feat.label = conv_label;

        else
            
            if FLAGskip == 1  % Skip the segment if the feature is already saved
                                
                if FLAGskip_print == 0
                    FLAGskip_print = 1;
                    load (tmp_name);
                    tmp_feat.NUMdesc = size(features.data,1);
                    NUMdesc(AllLabels==conv_label) = NUMdesc(AllLabels==conv_label) + tmp_feat.NUMdesc;
                    LISTclass(AllLabels==conv_label).fnames(AllScene_NUMseg(AllLabels==conv_label)).name = ['seg_' ts(2:end) '.mat'];                    
                    fprintf('Descriptor #: %6d',tmp_feat.NUMdesc);
                    fprintf('  ...skip!!!\n');
                end
                continue;
                
            end
            
            if FLAGerrlabel == 1  % Discard the segment if the label is not concerned
                
                if FLAGskip_print == 0
                    FLAGskip_print = 1;
                    fprintf('Descriptor #:         ...unconcerned label!!!\n');
                end
                continue;
                
            end

            tmp_data = textscan(tmp_line(2:end-1),'%s');  % Get rid of '(' and ')'
            tmp_data = tmp_data{1};
            DIMdata = length(tmp_data);

            %for i = 1:DIMdata-1;
            %    tmp_data{i} = tmp_data{i}(1:end-1);  % Get rid of ','
            %end

            tmp_data = str2double(tmp_data)';        
            tmp_feat.data = [tmp_feat.data;tmp_data];  % Add the feature vector

            if INDline ~= NUMline
                tmp_line_next = lines{INDline+1};
                if tmp_line_next(1) == '('
                    continue
                end
            end

            tmp_feat.NUMdesc = size(tmp_feat.data,1);  % Get the number of descriptors of this segment
            NUMdesc(AllLabels==conv_label) = NUMdesc(AllLabels==conv_label) + tmp_feat.NUMdesc;
            fprintf('Descriptor #: %6d',tmp_feat.NUMdesc);

            features = tmp_feat;
            
            LISTclass(AllLabels==conv_label).fnames(AllScene_NUMseg(AllLabels==conv_label)).name = ['seg_' ts(2:end) '.mat'];
            
            fprintf('\n');
            save(tmp_name, 'features')

        end

    end
    
    fprintf('\n');
    for i = 1:length(AllLabels)
        fprintf('Label %3d  Seg #: %4d  Desc #: %6d\n',AllLabels(i),NUMseg(i),NUMdesc(i));
    end

    fprintf('\nNumber of used segments:\t%6d\n',sum(NUMseg));
    fprintf('Number of discard segments:\t%6d\n',NUMdiscard);
    fprintf('Number of descriptors:\t\t%6d\n\n',sum(NUMdesc));

    AllScene_NUMdiscard = AllScene_NUMdiscard + NUMdiscard;
    AllScene_NUMdesc = AllScene_NUMdesc + NUMdesc;
    
end

fprintf('-------------------------------------------------------\n');
fprintf('converting map\n');
fprintf('-------------------------------------------------------\n\n');

for i = 1:size(mapping,1)
    for j = 1:size(mapping,2)
        
        fprintf('%4d',mapping(i,j));
    end
    fprintf('\n');
end

fprintf('\n');

fprintf('-------------------------------------------------------\n');
fprintf('statistics of all feature loaded\n');
fprintf('-------------------------------------------------------\n\n');

for i = 1:length(AllLabels)
    tl = num2str(AllLabels(i)+100);
    LISTclass(i).fnum = AllScene_NUMseg(i);
    LISTclass(i).name = ['label_' tl(2:end)];
    fprintf('Converted Label %2d  Seg #: %4d  Desc #: %8d\n',AllLabels(i),AllScene_NUMseg(i),AllScene_NUMdesc(i));
end

fprintf('\nNumber of used segments in all scene:\t\t%8d\n',sum(AllScene_NUMseg));
fprintf('Number of discard segments in all scene:\t%8d\n',AllScene_NUMdiscard);
fprintf('Number of descriptors in all scene:\t\t%8d\n\n',sum(AllScene_NUMdesc));

