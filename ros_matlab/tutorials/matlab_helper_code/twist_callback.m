function twist_callback(~, message)
    %   Subscriber callback function for twist data    
    %   Returns no arguments - it instead sets 
    %   global variables to the values of position and orientation that are
    %   received in the ROS message MESSAGE.

    
    % Declare global variables to store position and orientation
    global lin_vel
    global ang_vel
    
    % Extract linear and angluar data from twist ROS message and assign the
    % data to the global variables.
    lin_vel = [message.Linear.X ...
               message.Linear.Y ...
               message.Linear.Z];

    ang_vel = [message.Angular.X ...
               message.Angular.Y ...
               message.Angular.Z];
end