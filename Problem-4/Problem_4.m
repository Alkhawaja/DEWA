clear
Start_p=[0, 0];
End_p=[ceil(rand*100),ceil(rand*100)];
x_max = 100;
y_max = 100;
plot(Start_p(1),Start_p(2),'o')
hold on
plot(End_p(1),End_p(2),'x')
axis([0 x_max 0 y_max])
x_dev=x_max/10;
y_dev=y_max/10;
xblock(1,1)=End_p(1);
yblock(1,2)=End_p(1);
x=linspace(0,x_max);
y=linspace(0,y_max);
[X,Y]=meshgrid(x,y);
plot(X,Y,'k',Y,X,'k');
cost=zeros(100);

%right down
for i=End_p(1):x_max
 for k=End_p(2):y_max
cost(i,k)=(i+k)-(End_p(1)+End_p(2));
end
end
%Left up
for i=1:End_p(1)
 for k=1:End_p(2)
cost(i,k)=End_p(1)+End_p(2)-(i+k);
end
end
%right up
for i=End_p(1):x_max
 for k=1:End_p(2)
cost(i,k)=(i-k)-End_p(1)+End_p(2);end
end
%left down
for i=1:End_p(1)
 for k=End_p(2):y_max
cost(i,k)=End_p(1)-End_p(2)-(i-k);end
end
%cost=[cost2 cost4;cost3 cost1];
    %xblock(i,k)=i+k-1;
%     obstacle = [(i-1)*x_dev,(k-1)*y_dev,x_dev,y_dev];
% a=100;
% a_d=100;
% b=100;
% b_d=100;
%     % rectangle('Position',[a,b,a_d,b_d],'FaceColor',[2 0 0])
% %     hold on
% for i=a:a+a_d
% for k=b:b+b_d
% cost(i,k)=200;
% end
% end
    figure 
    for i=1:100
     for k=1:100
%          if cost(i,k)>30
%         plot(i,k,'xr')
%         hold on
%          else
    plot(i,k,'x','color',[abs(200-cost(i,k))/200 abs(200-cost(i,k))/200 0] );
    hold on
%          end
     end
    end
    
    %% EXAMPLE: Multi-Robot Swarm Behavior
% Copyright 2018 The MathWorks, Inc.


%% Create a multi-robot environment
numRobots = 6;
env = MultiRobotEnv(numRobots);
env.robotRadius = 1.05;
env.showTrajectory = false;
map= occupancyMap(100,100);
env.mapName = 'map';
%% Create robot detectors for all robots
detectors = cell(1,numRobots);
for rIdx = 1:numRobots
    detector = RobotDetector(env,rIdx);
    detector.maxDetections = numRobots;
    detector.maxRange = 1000;
    detector.fieldOfView = pi/2;
    detectors{rIdx} = detector;
end
env.plotSensorLines = false; % So the sensor lines don't dominate the visuals

%% Initialization
% Number of robot teams
numTeams = 1;  
env.robotColors = repmat(hsv(numTeams),[ceil(numRobots/numTeams) 1]);
env.robotColors = env.robotColors(1:numRobots,:); % Truncate colors in case there are unequal teams
env.cost=cost;
sampleTime = 0.5;                % Sample time [s]
tVec = 0:sampleTime:2700;        % Time array                

% Initialize poses randomly, and add bias to each "team"
poses = [70*(rand(2,numRobots))+6; ...
         pi*rand(1,numRobots)];
% angleBias = 20*pi*(1:numRobots)/numTeams;
% poses(1:2,:) = poses(1:2,:) + 25.5*[sin(angleBias);cos(angleBias)];

%% Simulation loop
vel = zeros(3,numRobots);
for idx = 2:numel(tVec)
    
    % Update the environment
    env(1:numRobots, poses);
    xlim([0 100]);   % Without this, axis resizing can slow things down
    ylim([0 100]); 
    
    % Read the sensor and execute the controller for each robot
    for rIdx = 1:numRobots
       detections = step(detectors{rIdx}); 
       vel(:,rIdx) = swarmTeamController(poses,rIdx,detections,numTeams,idx,floor(idx/300));
    end
    
    % Discrete integration of pose
    poses = poses + vel*sampleTime;

end

%% Helper function: Robot Controller Logic
function vel = swarmTeamController(poses,rIdx,detections,numTeams,ma,ii)
    
    % Unpack the robot's pose and team index
    pose = poses(:,rIdx);
    teamIdx = mod(rIdx,numTeams);

    % If there are no detections, turn in place
    v = 0;
    w = 60;
    x=1;
    m=300;
    ii;
    i=(mod(ii,2)>0)*(ii*m)+(mod(ii,2)==0)*(1+ii)*m;
    i+m;
    ma;
    % Else, turn towards the average of detected robots on the same team
    if ~isempty(detections)
        validInds = find(mod(detections(:,3),numTeams) == teamIdx);
        if ~isempty(validInds)
            
            % Take the average range and angle
            range = mean(detections(validInds,1));
            angle = mean(detections(validInds,2));
            
            %Move linearly to maintain a range to the nearest robot
            if range > 5
                v = -0.9*sign((ma>i&&ma<i+500)-0.5);
            end
            if range < 5
                v = -0.9;
                x=-x;
            end
            
            % Turn to maintain a heading to the nearest robot
            if angle > pi/12
                w = 2*((ma>i&&ma<i+300)*0.2);
            elseif angle < -pi/12
                w = -2;
            end
            
        end
        
    end


for i=1:numTeams
    if v<0
    if pose(1,i)<5 || pose(1,i)>95
       % fprintf( "hey")
          v=-v;
    elseif pose(2,i)<5 || pose(2,i)>95
        %fprintf("bye")
        v=-v;
    end
    end
end   
        


    % Convert to global velocity
    vel = bodyToWorld([v;0;w],pose);

end
%     map = occupancyMap(100,100,1);
% 
% for i=1:100
%     for k=1:100
% updateOccupancy(map,[i k], cost(i,k)/100);
% 
% end
% end
% figure
% show(map)
% axis([0 x_max 0 y_max])
%  pause(0.0001)
%     grid on
%     current_cost=cost(Start_p(1,1)+1,Start_p(1,2)+1)
%     loc_current_cost=[Start_p(1,1)+2,Start_p(1,2)+2]
%     m=loc_current_cost
% while(current_cost>0)
% for i=-1:1
% [ii,ll]=min(cost((loc_current_cost(1,1)-1:(loc_current_cost(1,1)+1)),(loc_current_cost(1,2)-1:loc_current_cost(1,2)+i)));
% [iii,lll]=min(ii);
% 
% end
% loc_current_cost=[loc_current_cost(1,1)+max(ll)-2,loc_current_cost(1,1)+lll-2];
% m=[m;loc_current_cost];
% end
%plot((i*10),(k*10),'*','color',[((i+k)*((i+k)<80))/200 ((i+k)*((i+k)>80)*((i+k)<120))/200 ((i+k)*((i+k)>120))/200]);
