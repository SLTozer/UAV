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

% time and time step
t = 0;
dt = 1.0;

% number of UAVs
num = 1;

uavBodies(num,1) = UavBody();
uavBrains = UavBrain.empty(num,0);
for i = 1:num
    uavBrains(i) = UavBrain(uavBodies(i));
end

% i-th message is sent from the i-th uav, and is formatted as:
% [x y bearing concentration]
if dt ~= 1.0
    error(['Message logic has not been set to deal with timesteps '...
           'other than 1.0']);
end
uavMessages = zeros(num,4);

% open new figure window
figure
hold on % so each plot doesn't wipte the predecessor

% main simulation loop
for kk=1:1000,
    %% Decision step
    % Make decisions from time [t -> t + dt]
    for i = 1:num
        uavBrains(i).decisionStep(cloud, t, dt, uavMessages);
    end
    
    %% Physical timestep
    % Timestep [t -> t + dt]
    t = t + dt;
    for i = 1:num
        uavBodies(i).move(dt);
        uavMessages(i,:) = uavBrains(i).getMessage();
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
