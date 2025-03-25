function resp = trigger_service_callback(~,~,resp)
%   This trigger_service_callback would perform a reset/boolean operation
%   (i.e. reset the Gazebo simulation).
%   In this toy example, we will simply desplay a message.
% 
% Inputs:
%   ~:          associated with the serivce server object
%   request:    none
%   response:   output data struc set by service type
%

    disp('trigger_service_activated');
    disp('Reset operation initiated...');
    pause(2);
    disp('Reset operation succesfully completed.');

end