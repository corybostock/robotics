classdef UR3Model < handle % setup and move the UR3 robot, as well as log its transforms
    properties
        model;
        currentJoints;
        location;
        workspace;
        plyData;
        radius;
    end
    
    methods
        function self = UR3Model(workspace,roboNum, location)
            self.workspace = workspace;
            self.GetRobot(roboNum);
            self.currentJoints = zeros(1,6);
            self.model.base = location;
            self.PlotAndColour(self.location);
            
        end
        function PlotAndColour(self,location)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['link',num2str(linkIndex),'.ply'],'tri');
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end
            
            % Display robot
            self.model.plot3d(self.currentJoints,'workspace',self.workspace,'floorlevel', 0);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight;
            end
            
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                        , plyData{linkIndex+1}.vertex.green ...
                        , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end    
        end
        
        function GetRobot(self, roboNum) % Setup Robot Parameters
            pause(0.001);
            L1 = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]));
            L2 = Link('d',0,'a',-0.24365,'alpha',0,'qlim',deg2rad([-360 360]));
            L3 = Link('d',0,'a',-0.21325,'alpha',0,'qlim',deg2rad([-360 360]));
            L4 = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]));
            L5 = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]));
            L6 = Link('d',0.0819,'a',0,'alpha',0,'qlim',deg2rad([-360 360]));
            
            pause(0.0001)
            name = ['UR_3_',roboNum];
            self.model = SerialLink([L1 L2 L3 L4 L5 L6], 'name', name);             
        end
        
        function maxRadius(self)
            disp('Checking arm reach... ');
            pause(0.5);
            % Robot arm extended to max upwards pose
            q = [0,-90,0,-90,0,0];
            self.model.animate(deg2rad(q));
            endefect = self.model.fkine(deg2rad(q));
            maxLengthPos = abs( endefect(1:3,4)' - [0,0,0.1519] );          % subtracting base link
            pause(0.5);

            % Robot arm extended to max downward pose
            q = [0,90,0,-90,0,0];
            self.model.animate(deg2rad(q));
            endefect = self.model.fkine(deg2rad(q));
            maxLengthNeg = endefect(1:3,4)' - [0,0,0.1519];
            pause(0.5);
            
            % Perfect sphere max volume
            disp('Volume in metres cubed of unrestricted robot reach: ');
            maxVol = ( 4 * pi * maxLengthPos(3)^3 ) / 3

            % https://mathworld.wolfram.com/SphericalCap.html?fbclid=IwAR13igaa2iavUjBd3vyHoOVTVOrUEAFUITtciWRmLwOi0T3nV4ReMzjtfS8
%             h = maxLengthPos(3) - maxLengthNeg(3);
%             disp('Volume in metres cubed of spherical cap: ');
%             maxVol = (pi * h^2 * (3 * maxLengthPos(3) - h)) / 3

            % 0.24365 is only for the mdl arm replace with math from link to work out
            % variable for first arm
            redundant_vol = pi * 0.24365^2 *  abs(maxLengthNeg(3));
            disp('Volume in metres cubed the robot can navigate to: ');
            maxVol = maxVol - redundant_vol
            
            q = [90,0,0,-90,0,0];
            self.model.animate(deg2rad(q));
            endefect = self.model.fkine(deg2rad(q));
            self.radius = abs( endefect(2,4));
        end
        
        function calcPointCloud(self, instance)
            loadFile = input('Load Point cloud from file? 0 - No, 1 - Yes: ');
            
            if loadFile == 0
                disp('Calculating Point Cloud... ');
                stepRads = deg2rad(30);
                qlim = self.model.qlim;
                pointCloudeSize = prod(floor((qlim(1:5,2)-qlim(1:5,1))/stepRads + 1));
                pointCloud = zeros(pointCloudeSize,3);
                counter = 1;
                tic;
                for q1 = qlim(1,1):stepRads:qlim(1,2)
                    for q2 = qlim(2,1):stepRads:qlim(2,2)
                        for q3 = qlim(3,1):stepRads:qlim(3,2)
                            for q4 = qlim(4,1):stepRads:qlim(4,2)
                                for q5 = qlim(5,1):stepRads:qlim(5,2)
                                    % Don't need to worry about joint 6, just assume it=0
                                    q6 = 0;
                                    q = [q1,q2,q3,q4,q5,q6];
                                    if (self.limitCheck(q) == 1)
                                        tr = self.model.fkine(q);                        
                                        pointCloud(counter,:) = tr(1:3,4)';
                                        counter = counter + 1
                                    end
                                 end
                             end
                         end
                     end
                end
                save('pCloud', 'pointCloud');
            end
            
            if loadFile == 1
                load('pCloud');
            end
            
            figure(instance);
            plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');
            [k, Max_Vol] = convhull(pointCloud);
            Max_Vol
            
        end
        
        function [t] = limitCheck(self, jointAngles)
            joints = jointAngles;
            t = 1;
            [rows, columns] = size(joints);
            currentLink = transl(0,0,0);                                    % assuming robot is at the origin
            for joint = 1:columns
                currentLink = currentLink * self.model.A(joint,joints);
                if(currentLink(3,4) < 0)
                    t = 0;
                    return
                end
            end
        end
        
        function view(self, type)
            switch type
                case 0 % top view
                    disp('Maximum reach upwards');
                    q = [0,-90,0,-90,0,0];
                    self.model.animate(deg2rad(q));
                case 1 % side view
                    disp('Maximum reach sideways');
                    q = [90,0,0,-90,0,0];
                    self.model.animate(deg2rad(q));
            end
            
            input('Observe the robot arm position, press enter to continue: ');
            
        end        
    end
end