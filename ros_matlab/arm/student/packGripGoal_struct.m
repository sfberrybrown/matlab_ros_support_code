function gripGoal = packGripGoal_struct(pos,gripGoal,optns)
    %----------------------------------------------------------------------
    % packGripGoal
    %----------------------------------------------------------------------
    % This function will populate gripGoal with all information necessary
    % in a control_msgs/FollowJointTrajectoryGoal message.
    %
    % This functions assumes a single waypoint for the gripper, i.e. open
    % or close. 
    %
    % The function will accomplish 3 steps:
    % 1) Set Point, name and duration. 
    % 2) Set Goal Tolerance via a control_msgs/JointTolerance message type
    % 3) Set a trajectory_msgs/JointTrajectoryPoint with time/pos/vel/accel/effort
    %
    % Input Arguments:
    % pos (double): a position where 0 is open and 
    % gripGoal (control_msgs/FollowJointTrajectoryGoal)
    %
    % Output
    % A populated gripGoal message
    %----------------------------------------------------------------------
    r = optns{'rHandle'};
    
    
    % 1. Set Point, name and Duration
    jointWaypoints = pos;                % Set position of way points
    jointWaypointTimes = 0.01;             % Time at which action is launched
    numJoints = size(jointWaypoints,1);    % Only 1 joint for gripper

    % TODO: Fill name of left finger 
    gripGoal.Trajectory.JointNames = 
    
    % Time Stamp
    if numJoints == 1

        % Set duration to 1 sec (or faster)
        r.trajPts.TimeFromStart   = 
    
    else
        r.trajPts.TimeFromStart   = rosduration(jointWaypointTimes,'DataFormat', 'struct');
    end
    
    % TODO: Position - set to relevant position
    r.trajPts.Positions       = 

    % Velocities
    r.trajPts.Velocities      = zeros(size(jointWaypoints));

    % Accelerations
    r.trajPts.Accelerations   = zeros(size(jointWaypoints));
    
    % Copy trajPts --> gripGoal.Trajectory.Points
    gripGoal.Trajectory.Points = r.trajPts;
end