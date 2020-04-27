close all
set(0,'DefaultFigureWindowStyle','docked')
clc
clf
clear

startup_rvc;                                                                % Ensuring robotics toolbox is active and functional
floorOffset = (-1.0896/2);                                                  % measured height or table in body0.ply
objectOffset = transl(0, 0 , -0.1);                                         % offset to lower endeffector onto object 
workspace = [-2.5 2.5 -2.5 2.5 (2*floorOffset) 1];
% workspace = [-1.5 1.5 -1.5 1.5 0 1.5];

% define Robot coordinates
x1 = input('Robot 1 base X coordinate: ');
y1 = input('Robot 1 base Y coordinate: ');
z1 = input('Robot 1 base Z coordinate: ');
robotLoc1 = transl(x1, y1, z1);
x2 = input('Robot 2 base X coordinate: ');
y2 = input('Robot 2 base Y coordinate: ');
z2 = input('Robot 2 base Z coordinate: ');
robotLoc2 = transl(x2, y2, z2);

% define object coordinates
topHousingPose      = transl(0.2,-0.3,0);                                     
bottomHousingPose   = transl(0.2,0,0);                                        
pcbPose             = transl(0.2,0.3,0);
boxPose             = transl(-0.15,0,0)* trotz(pi/2);

% ------------------------------------------

% PLY files for the UR3 have been face/vertex decimated heavily for 
% resposive animations and reduced computational time. 
% The imperfections in the surfaces allow this response time.

% Making robot 1 and calculating safety locations
robot1          = UR3Model(workspace, 0, robotLoc1);
robot1.maxRadius();                                                                         % Mathematical calc of max radius and volume reach
safeBarrierDistance = robot1.radius + 1.5;                                                  % Added 1.5m on to calculated radius for factor of safety

% Making robot 2 and environment
robot2          = UR3Model(workspace, 1, robotLoc2);
robot2.maxRadius();
table           = body(workspace, 0, transl(0,0,floorOffset));                              % Dimensions of the table (x, y, z) = (1.4880, 2.3383, 1.0896)
barrier1        = body(workspace, 1, transl(0,safeBarrierDistance,floorOffset));            % Dimensions of the barrier (x, y, z) = (3.3951, 0.7129, 1.7555)
barrier2        = body(workspace, 2, transl(safeBarrierDistance,0,floorOffset));
barrier3        = body(workspace, 3, transl(0,-safeBarrierDistance,floorOffset));
barrier4        = body(workspace, 4, transl(-safeBarrierDistance,0,floorOffset));
topHousing      = body(workspace, 5, topHousingPose);                                       % Top Housing
bottomHousing   = body(workspace, 6, bottomHousingPose);                                    % Bottom Housing
pcb             = body(workspace, 7, pcbPose);                                              % PCB
box             = body(workspace, 8, boxPose);
motion = move();                                                                            % Initialising motion class

% ------------------------------------------
% Calculating midpoint between the two robots
midpoint = (robot1.model.base * robot2.model.base)/2;
midpoint = midpoint(1:3,4);
if (max(midpoint)/2) < 0.25
    midpoint = transl(midpoint) * transl(0,0,0.25);
else
    midpoint = transl(midpoint) * transl(0,0,max(midpoint)/2);
end

% ------------------------------------------
% Max volume and radius information
robot1.calcPointCloud();                                                                   % Point cloud calc of max radius and volume reach.                                                                               % Setting target figure back to figure 1
robot1.view(0);                                                                            % Top view
robot1.view(1);                                                                            % Side view

% ------------------------------------------
% Quizk way to flip which robot picks up what
% temp = robot1;
% robot1 = robot2;
% robot2 = temp;
% delete(temp);

disp('Picking up top housing...');
motion.moveRobot(robot1, topHousingPose * trotx(pi) * objectOffset);
motion.moveRobot(robot1, topHousingPose * trotx(pi));

disp('Picking up PCB...');
motion.moveRobot(robot2, pcbPose * trotx(pi) * objectOffset);
motion.moveRobot(robot2, pcbPose * trotx(pi));

disp('Bringing PCB and top housing to midpoint...');
motion.moveRobotAndPart(robot1, midpoint * trotx(pi), topHousing);
motion.moveRobotAndPart(robot2, midpoint * trotx(pi), pcb);

disp('Picking up bottom housing...');
motion.moveRobot(robot2, bottomHousingPose * trotx(pi) * objectOffset);
motion.moveRobot(robot2, bottomHousingPose * trotx(pi));

disp('Bringing bottom housing to midpoint...');
motion.moveRobotAndPart(robot2, midpoint * trotx(pi), bottomHousing);

disp('Delivering components...');
motion.moveRobotAndAllParts(robot1, boxPose, topHousing, pcb, bottomHousing);

% ------------------------------------------
disp('Demonstration complete! :) ');
robot1.model.teach();
