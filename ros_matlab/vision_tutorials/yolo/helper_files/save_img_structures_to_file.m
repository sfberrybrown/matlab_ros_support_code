function save_img_structures_to_file(optns)
%--------------------------------------------------------------------------
% save_img_structures_to_file
%
% Note:Assumes a parent path at ros_matlab
%
% Extracts images from "myImgStruct" structure (created by collect_obj_images.m) 
% and save them to file. 
%
% Uses the base name of file_name as folder_name and then inside saves them
% as img##.jpg.
%--------------------------------------------------------------------------

    % Load .mat files
    if nargin == 0
        files = dir( fullfile('./vision_tutorials/yolo/data','*.mat') );
    else
        files = dir( fullfile(optns{'rHandle'}.yolo_data_path,'*.mat') );
    end
    
    % File length
    file_len = length(files);

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

        %% Create folder and cd into folder for set of images    
        tokens          = strsplit(files(i).name,'_');     
        base_name       = strjoin(tokens(1), '_');
        outputFileName  = append(base_name,"_", char(formattedDateTimeStr)); 
        relPath         = fullfile('./vision_tutorials/yolo/data',outputFileName);                         % Creates a full file path   
                
        if exist(relPath, 'dir')~=7  
            mkdir(relPath);
        end

        %% Save images: assumes image structure of strct.myImgStruct

        str = loadedData{i};
        field_names{i} = fieldnames(str.myImgStruct); % Hold cell array of field names        

        % Use field names to set outStruct to the equivalent images
        for j = 1:length(field_names{i})
            %field = append('img',num2str(ctr));

            % Save the image to file
            entry = field_names{i}{j};
            fileName = append(entry,'.jpg');
            imwrite( str.myImgStruct.(entry), fullfile(relPath,fileName) );

            % Increase counter
            ctr = ctr + 1;
        end
    end
end
