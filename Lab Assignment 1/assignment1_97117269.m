close all
set(0,'DefaultFigureWindowStyle','docked')
clc
clf
clear

% startup_rvc;                                                              % Ensuring robotics toolbox is active and functional
floorOffset = (-1.0896/2);                                                  % measured height or table in body0.ply
workspace = [-3 3 -3 3 (2*floorOffset) 1.5];
% workspace = [-1.5 1.5 -1.5 1.5 0 1.5];

% define Robot coordinates
x1 = input('Robot 1 base X coordinate: ');
y1 = input('Robot 1 base Y coordinate: ');
z1 = input('Robot 1 base Z coordinate: ');
robotLoc1 = transl(x1, y1, z1);
% x2 = input('Robot 2 base X coordinate: ');
% y2 = input('Robot 2 base Y coordinate: ');
% z2 = input('Robot 2 base Z coordinate: ');
% robotLoc2 = transl(x2, y2, z2);

% ------------------------------------------

% PLY files for the UR3 have been face/vertex decimated heavily for 
% resposive animations and reduced computational time. 
% The imperfections in the surfaces allow this response time.

% Making robot and environment objects
robot1          = UR3Model(workspace, 0, robotLoc1);
% robot2          = UR3Model(workspace, 1, robotLoc2);
% table           = body(workspace, 0, transl(0,0,floorOffset));              % Dimensions of the table (x, y, z) = (1.4880, 2.3383, 1.0896)
% barrier1        = body(workspace, 1, transl(0,2,floorOffset));              % Dimensions of the barrier (x, y, z) = (3.3951, 0.7129, 1.7555)
% barrier2        = body(workspace, 2, transl(2,0,floorOffset));
% barrier3        = body(workspace, 3, transl(0,-2,floorOffset));
% barrier4        = body(workspace, 4, transl(-2,0,floorOffset));
% topHousing      = body(workspace, 5, transl(-0.5,0,0));                     % Top Housing
% bottomHousing   = body(workspace, 6, transl(0,0,0));                        % Bottom Housing
% pcb             = body(workspace, 7, transl(0.5,0,0));                      % PCB

% ------------------------------------------
robot1.maxRadius();                                                         % Mathematical calc of max radius and volume reach
robot1.calcPointCloud();                                                    % Point cloud calc of max radius and volume reach
robot1.view(0);                                                             % Top view
robot1.view(1);                                                             % Side view
