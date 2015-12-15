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

uavBody = UavBody;
uavBrain = UavBrain(uavBody);

% open new figure window
figure
hold on % so each plot doesn't wipte the predecessor

% main simulation loop
for kk=1:1000,
    
    % Make decisions from time [t -> t + dt]
    uavBrain.decisionStep(cloud, t, dt);
    
    % Timestep [t -> t + dt]
    t = t + dt;
    uavBody.move(dt);
    
    
    % Get measurements for display
    p = uavBody.sensorReading(cloud, t);
    x = uavBody.pos;
    y = uavBrain.targetPos;
    b = uavBrain.bearingEstimate;
    
    % clear the axes for fresh plotting
    cla
    
    % put information in the title
    title(sprintf('t=%.1f secs pos=(%.1f, %.1f)  Concentration=%.2f  estHead=%.2f',t, x(1),x(2),p,b))
        
    % plot robot location
    plot(x(1),x(2),'o')
    plot(y(1),y(2),'x')
    
    % plot the cloud contours
    cloudplot(cloud,t)
    
    % pause ensures that the plots update
    pause(0.1)
    
end