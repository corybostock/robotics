classdef move < handle % Movement of robots or bodies
    properties
    end
    
    methods
        function self = move()
            disp('it worked yo');
        end     
        
        function moveRobot(self, robot, targetPos)
            tempq = robot.model.ikcon(targetPos, robot.model.getpos());
            jointTrajectory = jtraj(robot.model.getpos(), tempq ,100);
            
            for trajStep = 1:size(jointTrajectory,1)
                q = jointTrajectory(trajStep,:);
                robot.model.animate(q);
                pause(0.01);
            end
            pause(0.01);
        end
        
    end
end
