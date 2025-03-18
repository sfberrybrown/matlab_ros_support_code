function chatter_callback(~, message)
    %   This callback simply displays the string in message.Data
    
    % Print a fixed message:
    disp('Chatter Callback message data: ');
    
    % Print the actual topic data
    message.Data
end