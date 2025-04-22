    %% 00 Connect to ROS (use your own masterhost IP address)
    clc
    clear
    rosshutdown;

    %% 02 goHome,reset,optns...
    optns = startRobotWorld;

    %% 03 Get Images

    % For each object name,  you will get many images in accordance to
    % strategy. It's possible to set a world where you group objects you
    % care about and capture all objects in the same image.
    obj_names = {'block1'};
    % obj_names = {'marker1',...                
    %              'marker2',...
    %              'marker3',...
    %              'spam1',...
    %              'spam2',...
    %              'spam3',...
    %              'block1',...
    %              'block1',...     % Red
    %              'block2',...     % Green
    %              'block3',...     % Blue
    %              'block4'         % Purple
    %              };


    obj_len = length(obj_names); 
    img_struct = cell(1,obj_len);

    % Save image structures in the cells
    for i=1:length(obj_names)

        % Move arm around object and save imgs as struct
        img_struct{i} = collect_obj_images(obj_names{i},optns);
    end

    %% 04 Label Images
    % For this step use matlab's built-in imageLabeler;
    imageLabeler;

    %% 05 Train Detector