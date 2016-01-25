function sim_start
%
% simulation example for use of cloud dispersion model
%
% Arthur Richards, Nov 2014
%

%% Drawing instructions

% Must be set true for anything to be drawn
drawAny = true;
% Draw the cloud
drawCloud = true;
% Draw the UAVs
drawUavs = true;
% Draw control info used by the AIs
drawAI = true;
% Draws the pheremones used by AIs - EXTREMELY performance intensive
drawAIExtra = false;





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

%% UAV initial state

% number of UAVs
uavCount = 12;

% starting state of UAVs
startCirc = uavCount * 50;
startRad = startCirc / (2*pi);

uavBodies = UavBody.empty(uavCount,0);
uavBrains = UavBrain.empty(uavCount,0);
for i = 1:uavCount
    % position the uavs in a circle facing outwards from [0,0], such that
    % each UAV starts 4m (subject to error) apart
    startAng = ((i-1)/uavCount) * 2 * pi;
    [flippedStartVec(1),flippedStartVec(2)] = pol2cart(startAng, startRad);
    startPos = fliplr(flippedStartVec);
    % include sig = 25cm positional error and sig = 5 degrees angular error
    errPos = startPos + [randn*0.25, randn*0.25];
    errAng = startAng + (randn*pi/36);
    % include random battery failure - some UAVs may not be fully charged
    % 10% of batteries have reduced life, lasting 100-1800 seconds
    if rand < 0.1
        batteryLife = rand * (maxTime - 100) + 100;
    else
        batteryLife = maxTime;
    end
    % create objects 
    uavBodies(i) = UavBody(errPos, errAng, batteryLife);
    uavBrains(i) = UavBrain(uavBodies(i), i, aiMapRect);
end

% i-th message is sent from the i-th uav
% Each row of messages represents messages sent from a particular timestep.
% The j-th row of messages will arrive in j-1 timesteps.
transitMessageCount = ceil(1/dt);
uavMessages(uavCount,transitMessageCount) = Message();
if dt < 1.0
    error(['Message logic has not been set to deal with timesteps '...
           'lower than 1.0']);
end

if drawAny
    % open new figure window
    figure
    hold on % so each plot doesn't wipe the predecessor
end

crashedUavs = [];
outOfBoundsUavs = [];
deadUavs = [];

% main simulation loop
while t < maxTime,
    %% Decision step
    % Make decisions at time t
    for i = 1:uavCount
        % UAV brain receives messages, takes measurements, and decides the
        % next action for the physical UAV to take
        uavBrains(i).decisionStep(cloud, t, dt, uavMessages);
    end
    
    %% Physical timestep
    % Timestep [t -> t + dt]
    t = t + dt;
    % Messages in transit move forward to arrive in 1 second
    for i = 1:transitMessageCount-1
        uavMessages(:,i) = uavMessages(:,i+1);
    end
    for i = 1:uavCount
        % Messages enter transit and arrive after 1 second has passed
        uavMessages(i,transitMessageCount) = uavBrains(i).getMessage();
        % UAVs physically move
        uavBodies(i).move(dt);
    end
    
    % Handle crashes, assorted failure, and intentional landing
    for i = 1:uavCount
        if uavBodies(i).operational
            % UAVs that crash into each other
            for j = i+1:uavCount
                % Allow UAVs 30 seconds for take-off and calibration
                if t > 30 && uavBodies(j).operational
                    if UavBody.collision(uavBodies(i), uavBodies(j), 15) %sqrLen(uavBodies(i).pos-uavBodies(j).pos) <= 225
                        disp('collision!');
                        uavBodies(i).disable;
                        uavBodies(j).disable;
                        crashedUavs = [crashedUavs i j];
                    elseif UavBody.collision(uavBodies(i), uavBodies(j), 50)%sqrLen(uavBodies(i).pos-uavBodies(j).pos) <= 2500
                        disp('near collision!');
                    end
                end
            end
            % UAVs that leave the bounds of the map
            if uavBodies(i).outsideMap(mapRect)
                uavBodies(i).disable;
                outOfBoundsUavs = [outOfBoundsUavs i];
            end
            % UAVs that run out of battery and crash
            if uavBodies(i).batteryLife <= 0
                uavBodies(i).disable;
                deadUavs = [deadUavs i];
            end
        end
    end
    
    %% Display
    
    if drawAny
        % clear the axes for fresh plotting
        cla
        % put information in the title
        title(sprintf('t=%.1f secs',t))
        for i = 1:uavCount
            % plot robot location
            if drawUavs
                uavBodies(i).plot();
            end
            % plot robot AI data
            if drawAI
                uavBrains(i).draw(drawAIExtra);
            end
        end
        % plot the cloud contours
        if drawCloud
            cloudplot(cloud,t)
        else
            pause(0.01)
            axis equal
            axis([min(cloud.x) max(cloud.x) min(cloud.y) max(cloud.y)])
        end
    end
    
end

fprintf('%d UAVs failed to return to their launch spot before landing.\n', length(deadUavs));
fprintf('%d UAVs crashed into other UAVs.\n', length(crashedUavs));
fprintf('%d UAVs left the bounds of the map.\n', length(outOfBoundsUavs));
