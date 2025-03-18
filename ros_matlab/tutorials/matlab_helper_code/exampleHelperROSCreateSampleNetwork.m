%exampleHelperROSCreateSampleNetwork Create an example ROS network
%   This network is representative of a realistic ROS network and is used 
%   throughout the MATLAB ROS examples. It features three nodes,
%   two publishers, two subscribers, and two services. Additionally we use a
%   timer to control publishing of ROS messages over the network.
%
%   Node: 
%   Matlab programming does not encourage the creating
%   nodes/publishers/subscribers as one would in C++/Python. 
%   
%   Typically only the global_master_node is used and all
%   publishers/subscribers use this node. 
%
%   In C++/Python crafting each node would be a much more careful
%   deliberation. 
% 

%% Node Creation
% Create three local nodes
masterHost = 'localhost';
node_1 = ros.Node('node_1', masterHost);
node_2 = ros.Node('node_2', masterHost);
node_3 = ros.Node('node_3', masterHost);

%% Pose Topic
% Create a publisher for the '/pose' topic
twistPub = rospublisher('/pose','geometry_msgs/Twist','DataFormat','struct');

% To assign to a specific node do: 
% twistPub = ros.Publisher(node_1,'/pose','geometry_msgs/Twist','DataFormat','struct');

% Initialize a message of the type associated with the topic
twistPubmsg = rosmessage(twistPub);

%% Create a subscriber to the same toipc
twistSub = rossubscriber('/pose','DataFormat','struct');

% To assign to a specific node do: 
% twistSub = ros.Subscriber(node_2,'/pose','DataFormat','struct');

%% Scan and TF Topics

% First load a recording with messages on the /scan and /tf topics.
% We will want to then publish/subscribe this data.
messageData = load('specialROSMessageData.mat','scan','tf');

%% Create publisher for the '/scan' topic
scanPub = rospublisher('/scan','sensor_msgs/LaserScan','DataFormat','struct');

% If we want to control which nodes these come from:
% scanPub = ros.Publisher(node_3,'/scan','sensor_msgs/LaserScan','DataFormat','struct');

%% Create publishers and subscribers for the '/scan' topic
scanSub1 = rossubscriber('/scan','DataFormat','struct');

% Create two subscribers from 2 different nodes
% scanSub1 = ros.Subscriber(node_2,'/scan','sensor_msgs/LaserScan','DataFormat','struct');
% scanSub1 = ros.Subscriber(node_2,'/scan','sensor_msgs/LaserScan','DataFormat','struct');

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
                    ...,%@sumtwoints,...            %callback
                    'DataFormat','struct');         % Use structs

% If you want to ascribe to a particular node:
% srv1 = ros.ServiceServer(node_3,'/add','roscpp_tutorials/TwoInts','DataFormat','struct');

%% A second service server is created. 
% This service uses type std_srvs/Empty (emptry request and response).
% Serves as a trigger mechanism.
% Calling the service would executes a predefined action in the callback 
% without requiring input parameters or returning any meaningful data.
% Typically used for trigger-based operations like reset operations.
srv2 = rossvcserver('/reply', ...
                    'std_srvs/Empty',...
                    @exampleHelperROSEmptyCallback,...
                    'DataFormat','struct');
% 
% srv2 = ros.ServiceServer(node_3,'/reply','std_srvs/Empty',@exampleHelperROSEmptyCallback,'DataFormat','struct');

% Load sample data for inspecting messages
tf = messageData.tf;

%% Create a timer for publishing messages and assign appropriate handles
% 1. timerHandles: structure.
%    Contains two pub objects and their respective empty message objets
% 2. @exampleHelperROSSimTimer:
%    A script that:
%       (i) puts values into the PoseSteamped (Linear, Angular)
%    message structure; 
%       (ii) publishes the twistPub;
%       (iii) fills Header.Stamp of scanPub
%       (iv) publishes scanPub
% The timer will call exampleHelperROSSimTimer at a rate of 10 Hz.


timerHandles.twistPub    = twistPub;
timerHandles.twistPubmsg = twistPubmsg;

timerHandles.scanPub    = scanPub;
timerHandles.scanPubmsg = messageData.scan;

simTimer = ExampleHelperROSTimer(0.1, ...   % Rate
                                {@exampleHelperROSSimTimer, ... % Function that fills values of Handle
                                timerHandles} ...               % Structure with publishers and their messages
                                );

clear messageData
