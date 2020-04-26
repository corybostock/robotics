close all
set(0,'DefaultFigureWindowStyle','docked')
clc
clf
clear

% startup_rvc;                                                              % Ensuring robotics toolbox is active and functional
floorOffset = (-1.0896/2);                                                  % measured height or table in body0.ply
% workspace = [-3 3 -3 3 (2*floorOffset) 1.5];
workspace = [-1.5 1.5 -1.5 1.5 0 1.5];

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
topHousingPose      = transl(0.1,-0.5,0);                                     
bottomHousingPose   = transl(0.1,0,0);                                        
pcbPose             = transl(0.1,0.5,0);

% ------------------------------------------

% PLY files for the UR3 have been face/vertex decimated heavily for 
% resposive animations and reduced computational time. 
% The imperfections in the surfaces allow this response time.

% Making robot 1 and calculating safety locations
figure(1);
robot1          = UR3Model(workspace, 0, robotLoc1);
robot1.maxRadius();                                                                         % Mathematical calc of max radius and volume reach
safeBarrierDistance = robot1.radius + 1.5;                                                  % Added 1.5m on to calculated radius for factor of safety

% Making robot 2 and environment
% robot2          = UR3Model(workspace, 1, robotLoc2);
% robot2.maxRadius();
% table           = body(workspace, 0, transl(0,0,floorOffset));                              % Dimensions of the table (x, y, z) = (1.4880, 2.3383, 1.0896)
% barrier1        = body(workspace, 1, transl(0,safeBarrierDistance,floorOffset));            % Dimensions of the barrier (x, y, z) = (3.3951, 0.7129, 1.7555)
% barrier2        = body(workspace, 2, transl(safeBarrierDistance,0,floorOffset));
% barrier3        = body(workspace, 3, transl(0,-safeBarrierDistance,floorOffset));
% barrier4        = body(workspace, 4, transl(-safeBarrierDistance,0,floorOffset));
topHousing      = body(workspace, 5, topHousingPose);                                       % Top Housing
bottomHousing   = body(workspace, 6, bottomHousingPose);                                    % Bottom Housing
pcb             = body(workspace, 7, pcbPose);                                              % PCB
motion = move();                                                                            % Initialising motion class

% ------------------------------------------
% robot1.calcPointCloud(2);                                                                   % Point cloud calc of max radius and volume reach. Shown on Figure 2
figure(1);                                                                                  % Setting target figure back to figure 1
robot1.view(0);                                                                             % Top view
% robot1.view(1);                                                                             % Side view

% ------------------------------------------
motion.moveRobot(robot1, bottomHousingPose);
