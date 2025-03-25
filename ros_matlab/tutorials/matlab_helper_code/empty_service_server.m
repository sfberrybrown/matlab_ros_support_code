function resp = empty_service_server(~,~,resp)
% This service server is an toy server. It returns no arguments. 
% It simply displays a message indicating that it has been called.
% In a real service server you will do a useful function here.

disp('A service client called and reached the service server.');
end