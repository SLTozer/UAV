function sim_start
%
% simulation example for use of cloud dispersion model
%
% Arthur Richards, Nov 2014
%

% load cloud data
% choose a scenario
% load 'cloud1.mat'
load 'cloud2.mat'

mapRect = [min(cloud.x),min(cloud.y);max(cloud.x),max(cloud.y)];

% time and time step
t = 0;
dt = 1.0;
% UAVs have flight time of 30 mins only
maxTime = 1800.0;

% number of UAVs
num = 20;

uavBodies(num,1) = UavBody();
uavBrains = UavBrain.empty(num,0);
for i = 1:num
    uavBrains(i) = UavBrain(uavBodies(i), mapRect);
end

% i-th message is sent from the i-th uav, and is formatted as:
% [x y bearing concentration sent], where:
%   x & y are the perceived [x,y] coordinates of the UAV
%   bearing is the perceived bearing of the UAV
%   concentration is the measured cloud concentration of the UAV
%   sent indicates if the UAV sent a message - this is not an actual part
%       of the  message sent by the UAV, but an indicator variable (used
%       for efficiency)
uavMessages(num,1) = Message();
if dt ~= 1.0
    error(['Message logic has not been set to deal with timesteps '...
           'other than 1.0']);
end

% open new figure window
figure
hold on % so each plot doesn't wipe the predecessor

% main simulation loop
while t < maxTime,
    %% Decision step
    % Make decisions at time t
    for i = 1:num
        uavBrains(i).decisionStep(cloud, t, dt, uavMessages, i);
    end
    
    %% Physical timestep
    % Timestep [t -> t + dt]
    t = t + dt;
    for i = 1:num
        uavMessages(i,:) = uavBrains(i).getMessage();
        uavBodies(i).move(dt);
    end
    
    %% Display
    
    % clear the axes for fresh plotting
    cla
    % put information in the title
    title(sprintf('t=%.1f secs',t))
    % plot robot location
    for i = 1:num
        uavBodies(i).plot();
    end
    % plot the cloud contours
    cloudplot(cloud,t)
    
    % pause ensures that the plots update
    pause(0.1)
    
end
