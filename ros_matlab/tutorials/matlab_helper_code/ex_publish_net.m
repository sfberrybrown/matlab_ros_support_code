%   Example Publishing Net
%   
%   Create a simple ROS network that loads:
%   scan and tf data 
%   Create your own twist topic and message
%   
%   Additionally we use a timer to populate and publish twist data and
%   header time stamp data for scan.

%% Node Creation
% Create one local node
masterHost = 'localhost';
node_1 = ros.Node('node_1', masterHost);

%% Scan and TF Topics

% First load a recording with messages on the /scan and /tf topics.
% We will want to then publish/subscribe this data.
messageData = load('specialROSMessageData.mat','scan','tf');

%% Pose Topic
% Create a publisher for the '/pose' topic
twistPub = rospublisher('/twist','geometry_msgs/Twist','DataFormat','struct');
scanPub = rospublisher('/scan','sensor_msgs/LaserScan','DataFormat','struct');

% Initialize a message of the type associated with the topic
twistPubmsg = rosmessage(twistPub);

% Load sample data for inspecting messages
tf = messageData.tf;

%% Create a timer for publishing messages and assign appropriate handles
% 1. timerHandles: structure.
%    Contains two pub objects and their respective empty message objets
% 2. @exampleHelperROSSimTimer:
%    A script that:
%       (i) updates data of twistPubmsg (geometry_msgs/Twist: Linear, Angular)
%    message structure; 
%       (ii) publishes data via the twistPub;
%       (iii) fills Header.Stamp of scanPubmsg
%       (iv) publishes scanPub
% The timer will call exampleHelperROSSimTimer at a rate of 10 Hz.

% Handle: structure to encode any publisher objects and messages
timerHandles.twistPub    = twistPub;
timerHandles.twistPubmsg = twistPubmsg;

timerHandles.scanPub    = scanPub;
timerHandles.scanPubmsg = messageData.scan;

% Update data and publish at rate
rate = 0.1; % 10Hz
simTimer = ExampleHelperROSTimer(rate, ...                      % Rate
                                {@exampleHelperROSSimTimer, ... % Function that fills values of Handle
                                timerHandles} ...               % Structure with publishers and their messages
                                );

clear messageData
