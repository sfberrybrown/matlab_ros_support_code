%exampleHelperROSCreateSampleNetwork Create an example ROS network
%   This network features one node and two services. 

%% Node Creation
% Create three local nodes
masterHost = 'localhost';
node_1 = ros.Node('node_1', masterHost);

%% Services

% Services need a server and a client. The server side is the side that
% does the actual work. The client is the one who sends input data and
% receives the output. 

% Services like publish/subscribe have topics. We will refer to these as
% service topics. 
% 
% They also have types, we will call these service message
% types. 
%
% Finally, the server will need to be connected to a function to do the
% work you desire. 

%% Service Creation
% Create a 1st service servers that does nothing. It's a toy example
% Service Name: /add
% Service message type (has 2 inputs and 2 outputs):
% int64 a   (input)
% int64 b   (input)
% int64 sum (response)
%
%% Service Callback
% function response = sumtwoints(~, request, response)
%     response.sum = request.a + request.b;  % Perform the addition
% end

%% ROS Service messages 
% There are pre-existing/default ROS service messages types in the system.
% There is no easy tool like rosmsg list, intead if you want to see them,
% run:
%
% services = rosmsg('list'); % Lists available message types, including service types
% disp(services);
%
% In this case we will use 'roscpp_tutorials/TwoInts'


%% Create the first example of a service Server
srv1 = rossvcserver('/add', ...                     % Service topic
                    'roscpp_tutorials/TwoInts',...  % Service message
                    @sumtwoints,...            %callback
                    'DataFormat','struct');         % Use structs

% If you want to ascribe to a particular node:
% srv1 = ros.ServiceServer(node_1,'/add','roscpp_tutorials/TwoInts','DataFormat','struct');

%% A second service server is created. 
% This service uses type std_srvs/Empty (emptry request and response).
% Serves as a trigger mechanism.
% Calling the service would executes a predefined action in the callback 
% without requiring input parameters or returning any meaningful data.
% Typically used for trigger-based operations like reset operations.
srv2 = rossvcserver('/reply', ...
                    'std_srvs/Empty',...
                    @empty_service_server,...
                    'DataFormat','struct');
% 
% srv2 = ros.ServiceServer(node_3,'/reply','std_srvs/Empty',@exampleHelperROSEmptyCallback,'DataFormat','struct');
