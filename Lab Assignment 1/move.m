classdef move < handle % Movement of robots or bodies
    properties
    end
    
    methods
        function self = move()
           
        end     
        
        function moveRobot(self, robot, targetPos)
            tempq = robot.model.ikcon(targetPos, robot.model.getpos());
            trajectory = jtraj(robot.model.getpos(), tempq ,50);
            
            for step = 1:size(trajectory,1)
                q = trajectory(step,:);
                robot.model.animate(q);
                pause(0.001);
            end
        end
        
        function moveRobotAndPart(self, robot, targetPos, part)
            tempq = robot.model.ikcon(targetPos, robot.model.getpos());
            trajectory = jtraj(robot.model.getpos(), tempq ,50);
            
            for step = 1:size(trajectory,1)
                q = trajectory(step,:);
                robot.model.animate(q);
                endefect = robot.model.fkine(q);
                part.model.base = transl(endefect(1:3,4)');
                part.model.animate(0);
                pause(0.001);
            end
        end
    end
end
