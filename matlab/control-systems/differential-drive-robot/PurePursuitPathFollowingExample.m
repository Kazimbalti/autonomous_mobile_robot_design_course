%% Pure Pursuit Path Following for Differential Drive Robot
%
%   In this example we examine the performance of a Pure Pursuit Controller
%   responsible to guide a differential drive robot across a path with
%   fixed waypoints. 

clear;

%%  Define Desired Waypoints

path = [2.00    1.00;
        1.5    1.25;
        4.5    7.5;
        8.5    9.5;
        10.5   12.00;
        14.00   10.00;
        8.00    8.00;
        6.00    4.50;
        2.50    2.00];
    

%%  Set the Robot Current and Goal Location

robotCurrentLocation = path(1,:);
robotGoal = path(end,:);

%%  Set Initial Robot Orientation 

initialOrientation = 0;

%%  Set Initial Conditions Vector 
%
%   A differential 2D robot is considered - planning in [x,y, heading]

robotCurrentPose = [robotCurrentLocation initialOrientation];

%%  Initialize Aimplw Robot Simulator
%   The considered simulator only accounts for the kinematic equations of a
%   two-wheeled differential drive robot. 

robotRadius = 0.5;
robot = ExampleHelperRobotSimulator('emptyMap',2);
robot.enableLaser(false);
robot.setRobotSize(robotRadius);
robot.showTrajectory(true);
robot.setRobotPose(robotCurrentPose);

%%  Visualization of Desired Path 

plot(path(:,1), path(:,2),'k--d','LineWidth',2)
xlim([0 15])
ylim([0 15])

%%  Define the Path Following Pure Pusuit Controller

controller = robotics.PurePursuit;

%%  Inform the Controller for the Waypoints

controller.Waypoints = path;

%%  Define Desired Linear Velocity

controller.DesiredLinearVelocity = 0.4;

%%  Define Maximum Angular Velocity in rad/s

controller.MaxAngularVelocity = 2.5;

%%  Define the Look-ahead Distance

controller.LookaheadDistance = 0.5;

%% Define Radius of Acceptance for Goal Destination

goalRadius = 0.1;

%%  Define Distance to Goal

distanceToGoal = norm(robotCurrentLocation - robotGoal);

%%  Run the Simulation 
%
%   The controller runs at 10Hz

controlRate = robotics.Rate(10);
pause;
while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs
    [v, omega] = controller(robot.getRobotPose);
    
    % Simulate the robot using the controller actions
    drive(robot, v, omega);
     
    % Extract current robot pose
    robotCurrentPose = robot.getRobotPose;
    
    % Evaluate the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);
    
    waitfor(controlRate);
    
end

delete(robot)


