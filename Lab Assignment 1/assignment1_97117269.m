close all
set(0,'DefaultFigureWindowStyle','docked')
clc
clf
clear

% startup_rvc;
floorOffset = (-1.8453/2);
workspace = [-2 2 -2 2 floorOffset 2];


x1 = input('Robot 1 base X coordinate: ');
y1 = input('Robot 1 base Y coordinate: ');
z1 = input('Robot 1 base Z coordinate: ');
robotLoc1 = transl(x1, y1, z1);
x2 = input('Robot 2 base X coordinate: ');
y2 = input('Robot 2 base Y coordinate: ');
z2 = input('Robot 2 base Z coordinate: ');
robotLoc2 = transl(x2, y2, z2);

robot1 = UR3Model(workspace, robotLoc1, 1);
hold on;
robot2 = UR3Model(workspace, robotLoc2, 1);
table = body(workspace, 0, transl(0,0,floorOffset));







% % get pose
% tempq = UR3_1.ikine(housing_bottom_pose)
% UR3_1q = zeros(size(linkList)); 
% UR3_1.plot(UR3_1q)
% 
% % animate 1 
% jointTrajectory = jtraj(UR3_1.getpos(), tempq,60)
% 
% for trajStep = 1:size(jointTrajectory,1)
%     q = jointTrajectory(trajStep,:);
%     UR3_1.animate(q);
%     pause(0.001);
%     trajStep
% end

















% figure(1);
% robot1.plot(zeros(1,6));
% hold on;
% robot2.plot(zeros(1,6));
% 
% stepRads = deg2rad(30);
% qlim = robot1.qlim;
% % Don't need to worry about joint 6
% pointCloudeSize = prod(floor((qlim(1:5,2)-qlim(1:5,1))/stepRads + 1));
% pointCloud = zeros(pointCloudeSize,3);
% counter = 1;
% for q1 = qlim(1,1):stepRads:qlim(1,2)
%     for q2 = qlim(2,1):stepRads:qlim(2,2)
%         for q3 = qlim(3,1):stepRads:qlim(3,2)
%             for q4 = qlim(4,1):stepRads:qlim(4,2)
%                 for q5 = qlim(5,1):stepRads:qlim(5,2)
%                     % Don't need to worry about joint 6, just assume it=0
%                     q6 = 0;
% %                     for q6 = qlim(6,1):stepRads:qlim(6,2)
%                         q = [q1,q2,q3,q4,q5,q6];
%                         tr = robot1.fkine(q);                        
%                         pointCloud(counter,:) = tr(1:3,4)';
%                         counter = counter + 1; 
%                         if mod(counter/pointCloudeSize * 100,1) == 0
%                             display(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudeSize * 100),'% of poses']);
%                         end
% %                     end
%                 end
%             end
%         end
%     end
% end
% plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');
     
