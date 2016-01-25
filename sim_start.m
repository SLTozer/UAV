function sim_start
%
% simulation example for use of cloud dispersion model
%
% Arthur Richards, Nov 2014
%

% load cloud data
% choose a scenario
load 'cloud1.mat'
% load 'cloud2.mat'

displayAI = true;

mapRect = [min(cloud.x),min(cloud.y);max(cloud.x),max(cloud.y)];
% Slightly shrunken map
aiMapRect = mapRect * 0.9;

% time and time step
t = 0;
dt = 1.0;
% UAVs have flight time of 30 mins only
maxTime = 1800.0;

% number of UAVs
num = 10;

% starting state of UAVs
startCirc = num * 5;
startRad = startCirc / (2*pi);

uavBodies = UavBody.empty(num,0);
uavBrains = UavBrain.empty(num,0);
for i = 1:num
    % position the uavs in a circle facing outwards from [0,0], such that
    % each UAV starts 4m (subject to error) apart
    startAng = ((i-1)/num) * 2 * pi;
    [flippedStartVec(1),flippedStartVec(2)] = pol2cart(startAng, startRad);
    startPos = fliplr(flippedStartVec);
    % include sig = 25cm positional error and sig = 5 degrees angular error
    errPos = startPos + [randn*0.25, randn*0.25];
    errAng = startAng + (randn*pi/36);
    % create objects 
    uavBodies(i) = UavBody(errPos, errAng);
    uavBrains(i) = UavBrain(uavBodies(i), i, aiMapRect);
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
if dt < 1.0
    error(['Message logic has not been set to deal with timesteps '...
           'lower than 1.0']);
end

% open new figure window
figure
hold on % so each plot doesn't wipe the predecessor

% main simulation loop
while t < maxTime,
    %% Decision step
    % Make decisions at time t
    for i = 1:num
        % UAV brain receives messages, takes measurements, and decides the
        % next action for the physical UAV to take
        uavBrains(i).decisionStep(cloud, t, dt, uavMessages);
    end
    
    %% Physical timestep
    % Timestep [t -> t + dt]
    t = t + dt;
    for i = 1:num
        % Messages enter transit and arrive at the next timestep
        uavMessages(i) = uavBrains(i).getMessage();
        % UAVs physically move
        uavBodies(i).move(dt);
    end
    
    for i = 1:num
        for j = i+1:num
            %if uavBrains(i).currentState > UavState.Calibration && ...
            %        uavBrains(j).currentState > UavState.Calibration
            if t > 30
                if sqrLen(uavBodies(i).pos-uavBodies(j).pos) <= 225
                    disp('collision!');
                elseif sqrLen(uavBodies(i).pos-uavBodies(j).pos) <= 2500
                    disp('near collision!');
                end
            end
        end
    end
    
    %% Display
    
    % clear the axes for fresh plotting
    cla
    % put information in the title
    title(sprintf('t=%.1f secs',t))
    % plot robot location
    for i = 1:num
        uavBodies(i).plot();
        if displayAI == true
            uavBrains(i).draw();
        end
    end
    % plot the cloud contours
    cloudplot(cloud,t)
    
    % pause ensures that the plots update
    %pause(0.1)
    
end
