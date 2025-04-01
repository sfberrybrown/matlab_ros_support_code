function grip_result = place(strategy,optns)

    greenBin = [2.2 0 pi/2 -pi/2 0 0];
    
    %% 2. Move to desired location
    if strcmp(strategy,'topdown') 
        moveToQ("custom",optns,greenBin);
        [grip_result,grip_state] = doGrip('place',optns); 
        grip_result = grip_result.ErrorCode;
    end
end
