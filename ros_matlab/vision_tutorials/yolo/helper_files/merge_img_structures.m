function outStruct = merge_img_structures
% Assumes .mat image structures in ./ros_matlab/code/_vision/data
% Will output a yoloTrainingData_yyyymmdd_hhmmss.mat into a merge folder

    % only load .mat files (assumes they are strictly img structures)
    files = dir('*.mat');
    
    % File length
    file_len = length(files);

    % Create a merge folder and move inside
    if exist('merge', 'dir') ~= 7                    % If the folder does not exist, create it 
        mkdir('merge');
        cd('merge')
    end    

    % Initialize a cell array to hold the loaded data structures
    loadedData = cell(file_len, 1);
    
    for i = 1:file_len
        % Construct the full path to the file
        filePath = fullfile(files(i).folder, files(i).name);
        
        % Load structures inside cell. Still need to refer to them by internal field name: myImgStruct to access data
        loadedData{i} = load(filePath);
    end    
  
    % Timestamp for folder
    formattedDateTimeStr = datetime('now', 'Format', 'yyyyMMdd_HHmmss');

    ctr = 1;
    field_names = cell(1,file_len);
    
    for i = 1:file_len
        str = loadedData{i};
        try
            field_names{i} = fieldnames(str.myImgStruct); % Hold cell array of field names
        catch
            field_names{i} = fieldnames(str.outStruct);
        end       

        %% Create folder and cd into folder for set of images    
        parts = strsplit( files(i).name, '_');     
        tokens = strjoin( parts(1:end-2), '_');
        outputFileName = append(tokens,"_", char(formattedDateTimeStr)); 
        fullPath = fullfile(files(1).folder,outputFileName); % Creates a full file path   
        % if exist('data', 'dir') ~= 7                    % If the folder does not exist, create it 
        %     mkdir('data');
        %     cd('./data');
        if exist(fullPath,'dir') ~= 7
            mkdir(fullPath);
            cd(fullPath);
        end

        % else
        %     cd('./data')
        %     if ~exist(fullPath,'dir')
        %         mkdir(fullPath);
        %         cd(fullPath);
        %     else
        %         cd(fullPath);
        %     end
        % end

        %% Save images

        % Use field names to set outStruct to the equivalent images
        for j = 1:length(field_names{i})
            field = append('img',num2str(ctr));

            % Copy the image over
            entry = field_names{i}{j};
            try
                imwrite(str.myImgStruct.(entry), append(field,'.jpg') );
                outStruct.(field) = str.myImgStruct.(entry);
            catch
                imwrite(str.outStruct.(entry), append(field,'.jpg') );
                outStruct.(field) = str.outStruct.(entry);
            end
            
            % Increase counter
            ctr = ctr + 1;
        end

        cd('..');
    end

    %% Save struct to file
    
    % Add timestamp
    outputFileName = append('merged_img_struc',"_", char(formattedDateTimeStr)); 
    fullPath = fullfile(files(1).folder, 'merge', outputFileName); % Creates a full file path   
    if exist('merge', 'dir') ~= 7                    % If the folder does not exist, create it 
        mkdir('merge');
        cd('merge')
    end
    save(fullPath, 'outStruct');
    fprintf('File saved as %s\n', outputFileName);
end
