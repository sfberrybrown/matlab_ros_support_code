function update_data_and_pub(~, ~, handles)
    %   update_data_and_pub - 
    %   Timer update function called in startExamples to publish
    %   messages at well-defined intervals
    %   
    %   Publishes /twist and /scan messages at regular intervals. 
    % 
    %   The /scan message value remains constant while
    %   the /twist message value changes continually.
    
    % Before we publish the twistPub message let's change its linear/angular sections
    if isvalid(handles.twistPub)

        % Linear section of Twist
        handles.twistPubmsg.Linear.X = handles.twistPubmsg.Linear.X + (rand(1)-0.5)./10;
        handles.twistPubmsg.Linear.Y = handles.twistPubmsg.Linear.Y + (rand(1)-0.5)./10;
        handles.twistPubmsg.Linear.Z = handles.twistPubmsg.Linear.Z + (rand(1)-0.5)./10;

        % Angular section of Twist
        handles.twistPubmsg.Angular.X = handles.twistPubmsg.Angular.X + (rand(1)-0.5)./10;
        handles.twistPubmsg.Angular.Y = handles.twistPubmsg.Angular.Y + (rand(1)-0.5)./10;
        handles.twistPubmsg.Angular.Z = handles.twistPubmsg.Angular.Z + (rand(1)-0.5)./10;
    
        % Publish the twist messages via send
        send(handles.twistPub,handles.twistPubmsg);
    end
       
    % Before publishing to scan, let's change some values (the time-stamps)
    if isvalid(handles.scanPub)
        handles.scanPubmsg.Header.Stamp = rostime('now','system',...
            'DataFormat',handles.scanPub.DataFormat);

        % Publish the scan message via send
        send(handles.scanPub,handles.scanPubmsg);
    end
end