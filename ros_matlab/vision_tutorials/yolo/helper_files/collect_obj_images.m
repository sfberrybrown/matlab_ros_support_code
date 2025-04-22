function myImgStruct = collect_obj_images(model_name,optns)
%--------------------------------------------------------------------------
%myImgStruct = collect_obj_images() collects image data from gazebo objects in the simulation world. 
%
% The method will identify the object's position while keeping the robot arm's current height. 
% Then the robot arm will cycle about the object an 'iteration' number of
% times, visiting rot_offset points around the circle, and restarting the
% cycle at lin_offset distance away from the center at the next cycle.  
%
% Parameters:
% - rot_ffset (double) - number of points to visit around the circle
% - lin_offset (double) - displacement by which arm should move w/ e/ cycle
% - iterations (doulbe) - number of loops to execute, each one move
%                         lin_offset further away.
% Inputs:
% - model (string) - name of specific object, i.e. gCan1, yBottle2, pouch3
% - optns (dict)   - optns for all global variables
%
% Output:
% - myImgStruct (struct) - struct containing imgs of name img#
%
% Write Files:
% 'myImageFile.mat' - saves the img structure to file
%
% TODO: reload different objects directly via service
%-------------------------------------------------------------------------- 

   
    %% 01 Local Variables
    myImgStruct = struct(); % Keep images here
    
    % Dict
    optns("rot_offset")          = {12};        % 2pi / rot_offset
    optns("lin_offset")          = {0.10};      % Displacement in (m)
    optns("iterations")          = {2};         % How many loops
    optns("data_path")           = {'./vision_tutorials/yolo/data'}; % Where to store images

    % Pull out ROS class handle
    r = optns{'rHandle'}; 
           
    % Modify rot_offset with random numbers to introduce variance
    rot_offset = optns{"rot_offset"};
    rand_valence = randi([0, 1]) * 2 - 1;               % randomly shifts between 1 and -1
    rand_addition = randi(3);                           % random offset between 1 and 3
    rand_offset = rand_valence * rand_addition;
    rot_offset = rot_offset + rand_offset; 

    % Calculate total number of operations
    total_operations = rot_offset*optns{"iterations"};
    
    %% Center Arm around Object
    models = getModels(optns);                         % Extract gazebo model list
    
    % Retrieve desired index
    try
        index = strcmp(models.ModelNames, model_name);
    catch
        error('The model you are looking for does not exist');   
    end
    
    % Retrieve the model name
    model_name = models.ModelNames{index};         % rCan3=26, yCan1=27,rBottle2=32...%model_name = models.ModelNames{i}  
    
    % Extract pose of that model
    get_robot_gripper_pose_flag = 0;
    [rob, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name,get_robot_gripper_pose_flag,optns);
    
    % Keep the current robot's height
    mat_R_T_M(3,4) = rob(3,4);

    % Keep camera facing forward
    % col1 = mat_R_T_M(:,1);
    % col2 = mat_R_T_M(:,2);
    % mat_R_T_M(:,1) = col2;
    % mat_R_T_M(:,2) = col1;

    disp(mat_R_T_M);
    
    % Center robot arm around object    
    disp('Moving on top of object...');
    traj_result = moveTo(mat_R_T_M,optns);
      
    %% Cycle around object. Place all images in structure
    try
        for i = 1:total_operations
           
            %% Read Image
            rosImg      = receive(r.rgb_sub);
            myImg       = rosReadImage(rosImg);        
            fprintf('Recorded img %d of %d...\n',i,total_operations);
    
            % Visualize 
            if optns{'debug'}
                imshow(myImg)
            end
        
            % Save img before transformation to file
            imageName = sprintf('img%d', i);
            myImgStruct.(imageName) = myImg;
            
        
            %% Transform offset
            % Create offsets and insert noise for randomness
            % Linear offset
            lOff_k = ceil( i/rot_offset );             % Ie n=4: (1,2,3,4)=>1, (5,6,7,8)=>2
                            
            % Rotation offset        
            rOff_k = mod( i,optns{"rot_offset"} );              % Ie n=4: (1,2,3,4), (1,2,3,4),...
               
            % Compute an offset arm from base that rotates with each iteration
            d = (optns{"lin_offset"} + 0.05*rand )* lOff_k;     % Add to this offset some random difference of upto 5cm
            d = [d,0,0,0]';
            R = trotz(2*pi/rot_offset * rOff_k);
            T = R*d;
        
            % Ajust matlab2ros coord frames
            temp = T(2);
            T(2) = T(1);
            T(1) = -temp; 
            new_R_T_M = mat_R_T_M; % Do not modify base pose, you will use it anew in each iteration
            new_R_T_M(:,4) = new_R_T_M(:,4) + T;
            
            %% Move to new location around circle, will later take a picture
            traj_result = moveTo(new_R_T_M,optns);    
            fprintf('Cycle %d/%d...\n\n', i,total_operations);
        end
    catch
        %% Save img to structure if there is an exception

        % Create file name associated with model
        outputFileName = set_inputObj_FileName(model_name);
    
        % Save in data folder
        fullPath = fullfile(optns{'data_path'}, outputFileName); % Creates a full file path   
        if ~exist(optns{'data_path'}, 'dir')                     % If the folder does not exist, create it 
            mkdir(optns{'data_path'});
        end
    
        % Save struct to file
        save(fullPath, 'myImgStruct')
        fprintf('File saved as %s\n', outputFileName);
    end
    
    %% Save last img to structure
    % Name images as img00 and save them to file with random class name
    imageName   = sprintf('img%d', i+1); 
    rosImg      = receive( r.rgb_sub );
    myImg       = rosReadImage(rosImg);
    
    myImgStruct.(imageName) = myImg;
    
    %% Save struct to desired file name path

    % Create file name associated with model
    outputFileName = set_inputObj_FileName(model_name);

    % Save in data folder
    fullPath = fullfile(optns{'data_path'}, outputFileName); % Creates a full file path   
    if exist(optns{'data_path'}, 'dir') ~= 7                    % If the folder does not exist, create it 
        mkdir(optns{'data_path'});
    end

    % Save struct to file
    save(fullPath, 'myImgStruct')
    fprintf('File saved as %s\n', outputFileName);

end