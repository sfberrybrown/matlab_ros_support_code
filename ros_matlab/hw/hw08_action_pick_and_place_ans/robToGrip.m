function mat_R_T_G = robToGrip
    %% Local variables
    tf_listening_time   = 10;    % Time (secs) to listen for transformation in ros
    frameAdjustmentFlag = 1;     % Indicates matlab's base_link does not match ros. Need adjustment.
    toolAdjustmentFlag  = 1;     % Indicates we have fingers but have not adjusted IKs for it.    
   
    %% 1. Get Poses from matlab wrt to World
    disp('Setting the goal...');

    % Robot's base_link and model pose wrt gazebo world origin
    W_T_R = get_model_pose('robot');
    
    %% 2. Get Goal|Current Pose wrt to **MATLAB** base link in matlab format
    mat_W_T_R = ros2matlabPose(W_T_R, frameAdjustmentFlag, 0);
    
    % Change reference frame from world to robot's base_link

    z_offset = 0.02; %0.052; % Can height is 5.2cm

    %% 3. Modify orientation of robot pose to be a top-down pick (see tool0 vs base_link) w fingers aligned with matlab's y_world -axis
    fing_along_y = eul2tform([-pi/2 -pi 0]); % ZYX axis
    %fing_along_x = eul2tform([0 0 pi]); 
           
    %% 4. Current Robot Pose in Cartesian Format
    mat_R_T_G = eye(4,4);

    if 1

        tftree       = rostf('DataFormat','struct');     
        base         = 'base_link';
        end_effector = 'tool0'; % When finger is properly modeled use 'gripper_tip_link'
    
        % Get end-effector pose wrt to base via getTransform(tftree,targetframe,sourceframe), where sourceframe is the reference frame 
        try
            current_pose = getTransform(tftree,end_effector,base,rostime('now'),'Timeout', tf_listening_time);
        catch
            % Try again
            current_pose = getTransform(tftree,end_effector,base,rostime('now'),'Timeout', tf_listening_time);
        end
    
        % Convert gripper pose to matlab format
        mat_R_T_G = ros2matlabPose(current_pose,frameAdjustmentFlag,toolAdjustmentFlag);    
    end
end