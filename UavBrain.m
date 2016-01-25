classdef UavBrain < handle
    
    % UAV behavioural constants
    properties (Constant)
        AvoidanceRadius = 135;
        DangerRadius = 65;
        RandomWalkMaxDistance = 600.0;
        FirstTarget = [0 100];
        PheremoneDetectionRange = 800.0;
        PheremoneAvoidanceRange = 300.0;
        MaxPheremoneCount = 100;
        ApproachTimeoutTime = 100.0;
        CalibrationTime = 30.0;
    end
    
    % UAV physical state
    % Objective physical properties of the UAV and the world
    properties (SetAccess = public, GetAccess = public)
        uavBody;
        nextTurnRate;
        nextVelocity;
        mapRect;
    end
    % UAV estimated state
    % Updated automatically during each timestep
    properties (SetAccess = public, GetAccess = public)
        % UAV physical state
        posEstimate;
        lastPosEstimate;
        bearingEstimate;
        % Search data
        conc;
        lastConc;
        % Avoidance data
        closestUav;
        closestSqrDist;
        closeUavCount;
    end
    % UAV AI state
    % Used to control the actions taken by the UAVs AI
    properties (SetAccess = public, GetAccess = public)
        id
        currentState;
        lastState;
        targetPos;
        calibrated;
        % Tracking data
        localPheremones;
        localPheremoneCount;
        nextPheremone;
        % State data
        calibrationTimer;
        approachTimeout;
        searchTimeout;
        retreating;
        home;
    end
    
    methods
        function uavBrain = UavBrain(uavBody, id, mapRect)
            uavBrain.uavBody = uavBody;
            uavBrain.id = id;
            uavBrain.currentState = UavState.Start;
            uavBrain.posEstimate = uavBody.getGpsPos();
            uavBrain.bearingEstimate = 0;
            uavBrain.targetPos = [0,0];
            uavBrain.conc = 0;
            uavBrain.mapRect = mapRect;
            uavBrain.calibrated = false;
            %uavBrain.localPheremones(1,UavBrain.MaxPheremoneCount) = Pheremone([0 0], PheremoneType.None);
            uavBrain.localPheremones = Pheremone([0 0], PheremoneType.None);
            for i = 1:UavBrain.MaxPheremoneCount
                uavBrain.localPheremones(i) = Pheremone([0 0], PheremoneType.None);
            end
            uavBrain.nextPheremone = PheremoneType.None;
            uavBrain.approachTimeout = 0;
            uavBrain.retreating = false;
            uavBrain.home = [0 0];
            uavBrain.localPheremoneCount = 0;
            uavBrain.closeUavCount = 0;
        end
        
        function leaving = leavingMap(uavBrain)
            moved = uavBrain.posEstimate - uavBrain.lastPosEstimate;
            if uavBrain.posEstimate(1) < uavBrain.mapRect(1,1) && moved(1) < 0
                leaving = true;
            elseif uavBrain.posEstimate(1) > uavBrain.mapRect(2,1) && moved(1) > 0
                leaving = true;
            elseif uavBrain.posEstimate(2) < uavBrain.mapRect(1,2) && moved(2) < 0
                leaving = true;
            elseif uavBrain.posEstimate(2) > uavBrain.mapRect(2,2) && moved(2) > 0
                leaving = true;
            else
                leaving = false;
            end
        end
        
        % Returns true if this UAV has priority over another UAV with the
        % given state and ID
        function hasPriority = uavPriority(uavBrain, otherID, otherState)
            statePriority = UavState.priority(uavBrain.currentState, otherState);
            if statePriority > 0
                hasPriority = true;
            elseif statePriority < 0
                hasPriority = false;
            else
                if uavBrain.id > otherID
                    hasPriority = true;
                else
                    hasPriority = false;
                end
            end
        end
        
        function [closestUav, closestSqrDist, closeUavCount] = findClosestUav(uavBrain, messages)
            closestSqrDist = inf;
            closestUav = 0;
            closeUavCount = 0;
            messageCount = size(messages(:,1));
            for i = 1:messageCount
                if messages(i).id == uavBrain.id || messages(i).state == UavState.Inactive
                    continue
                end
                differenceVector = uavBrain.posEstimate - messages(i).pos;
                squaredDistance = sqrLen(differenceVector);
                if squaredDistance <= UavBrain.AvoidanceRadius^2
                    closeUavCount = closeUavCount + 1;
                end
                if squaredDistance < closestSqrDist
                    closestSqrDist = squaredDistance;
                    closestUav = i;
                end
            end
        end
        
        % Returns the list of all UAVs within the Avoidance Radius based on
        % received messages. 
        function [closeUavs, sqrDists] = findCloseUavs(uavBrain, messages)
            sqrDists = zeros(1,length(messages));
            closeUavs = zeros(1,length(messages));
            uavCount = 0;
            messageCount = size(messages(:,1));
            for i = 1:messageCount
                if messages(i).id == uavBrain.id || messages(i).state == UavState.Inactive
                    continue
                end
                differenceVector = uavBrain.posEstimate - messages(i).pos;
                squaredDistance = sqrLen(differenceVector);
                if squaredDistance <= UavBrain.AvoidanceRadius^2
                    uavCount = uavCount + 1;
                    sqrDists(uavCount) = squaredDistance;
                    closeUavs(uavCount) = i;
                end
            end
            sqrDists = sqrDists(1:uavCount);
            closeUavs = closeUavs(1:uavCount);
        end
        
        function message = getMessage(uavBrain)
            message = Message();
            if uavBrain.uavBody.operational
                message.pos = uavBrain.posEstimate;
                message.state = uavBrain.currentState;
                message.bearing = uavBrain.bearingEstimate;
                message.pheremoneType = uavBrain.nextPheremone;
                message.id = uint8(uavBrain.id);
            end
        end
        
        % Sets the UAVs velocity and turnrate to the ideal values for
        % reaching the current target point
        function flyToTarget(uavBrain, dt)
            targetDiff = uavBrain.targetPos - uavBrain.posEstimate;
            targetAng = atan2(targetDiff(1), targetDiff(2));
            angDiff = wrapToPi(targetAng - uavBrain.bearingEstimate);
            % If not facing towards the target point (within 5 degrees),
            % rotate until we are
            if abs(angDiff) > pi/60
                uavBrain.nextVelocity = UavBody.MinVelocity;
                requiredTurnRate = angDiff / (uavBrain.nextVelocity * dt);
                % Clamp the turn rate to the correct value
                uavBrain.nextTurnRate = max(min(requiredTurnRate,UavBody.MaxTurnRate),-UavBody.MaxTurnRate);
            % Otherwise, full speed ahead
            else
                uavBrain.nextTurnRate = 0;
                uavBrain.nextVelocity = UavBody.MaxVelocity;
            end
        end
        
        function [vel, turn] = avoidanceTrajectory(uavBrain, pos, bearing, dt)
            avoidVec = pos - uavBrain.posEstimate;
            avoidAng = atan2(avoidVec(1),avoidVec(2));
            angDiff = wrapToPi(avoidAng - uavBrain.bearingEstimate);
            right = angDiff > 0;
            front = abs(angDiff) < pi/2;
            if right 
                turn = -UavBody.MaxTurnRate/2;
            else
                turn = UavBody.MaxTurnRate/2;
            end
            if front
                turn = turn * 1.5;
                vel = UavBody.MinVelocity;
            else
                vel = UavBody.MaxVelocity;
            end
        end
        
        % Extract all pheremone data from messages sent within the
        % Pheremone Detection Range, and store in the local pheremone
        % database
        function getPheremoneData(uavBrain, messages)
            for i = 1:length(messages)
                if messages(i).pheremoneType ~= PheremoneType.None ...
                        && uavBrain.localPheremoneCount < UavBrain.MaxPheremoneCount
                    uavBrain.localPheremoneCount = uavBrain.localPheremoneCount + 1;
                    pheremoneVec = messages(i).pos - uavBrain.posEstimate;
                    sqrPheremoneDist = pheremoneVec(1)^2 + pheremoneVec(2)^2;
                    if sqrPheremoneDist <= UavBrain.PheremoneDetectionRange^2
                        pheremone = Pheremone(messages(i).pos, messages(i).pheremoneType);
                        uavBrain.localPheremones(uavBrain.localPheremoneCount) = pheremone;
                    end
                end
            end
        end
        
        % Decrements the remaining duration of all stored pheremones, and
        % removes pheremones that have expired
        % TODO: Inefficient use of array resizing, especially as entries
        % being deleted will typically be at the start of the array -
        % consider using some kind of sorting and/or preallocation.
        function updatePheremoneData(uavBrain, dt)
            removePheremoneCount = 0;
            for i = 1:uavBrain.localPheremoneCount
                uavBrain.localPheremones(i).duration = ...
                    uavBrain.localPheremones(i).duration - dt;
                if uavBrain.localPheremones(i).duration <= 0
                    removePheremoneCount = removePheremoneCount + 1;
                end
            end
            if removePheremoneCount > 0
                uavBrain.localPheremoneCount = uavBrain.localPheremoneCount - removePheremoneCount;
                for i = 1:uavBrain.localPheremoneCount
                    uavBrain.localPheremones(i) = uavBrain.localPheremones(i+removePheremoneCount);
                end
            end
        end
        
        % Finds a random target location within the given radius that is
        % suitably distant from any "Searched" pheremones
        function target = getRandomTarget(uavBrain, radius)
            valid = false;
            currentRadius = radius;
            sqrPheremoneRadius = UavBrain.PheremoneAvoidanceRange^2;
            attemptCount = 0;
            while valid == false
                ang = (rand * 2 * pi) - pi;
                dist = (rand * (currentRadius - UavBrain.PheremoneAvoidanceRange)) ...
                    + UavBrain.PheremoneAvoidanceRange;
                [flippedTargetVec(1),flippedTargetVec(2)] = pol2cart(ang,dist);
                target = uavBrain.posEstimate + fliplr(flippedTargetVec);
                valid = ( ...
                    target(1) >= uavBrain.mapRect(1,1) & ...
                    target(1) <= uavBrain.mapRect(2,1) & ...
                    target(2) >= uavBrain.mapRect(1,2) & ...
                    target(2) <= uavBrain.mapRect(2,2));
                i = 1;
                % To prevent near-infinite loops
                if attemptCount < 20
                    while valid == true && i <= length(uavBrain.localPheremones)
                        if uavBrain.localPheremones(i).type == PheremoneType.Searched
                            pheremoneVec = uavBrain.localPheremones(i).pos - target;
                            sqrPheremoneDist = sqrLen(pheremoneVec);
                            if sqrPheremoneDist <= sqrPheremoneRadius
                                valid = false;
                            end
                        end
                        i = i + 1;
                    end
                end
                if currentRadius < (uavBrain.mapRect(2,1) - uavBrain.mapRect(1,1))/2
                    currentRadius = currentRadius + 100.0;
                end
                attemptCount = attemptCount + 1;
            end
        end
        
        % Make decisions at time t
        function decisionStep(uavBrain, cloud, t, dt, messages)
            % Do nothing if the UAV is no longer active
            if uavBrain.uavBody.operational == false
                return;
            end
            %% Update UAV data
            uavBrain.lastPosEstimate = uavBrain.posEstimate;
            uavBrain.lastConc = uavBrain.conc;
            uavBrain.posEstimate = uavBrain.uavBody.getGpsPos();
            uavBrain.conc = uavBrain.uavBody.sensorReading(cloud, t);
            %[uavBrain.closeUavs, uavBrain.closeSqrDists] = ...
            %    uavBrain.findCloseUavs(messages);
            [uavBrain.closestUav, uavBrain.closestSqrDist, uavBrain.closeUavCount] ...
                = uavBrain.findClosestUav(messages);
            uavBrain.getPheremoneData(messages);
            uavBrain.nextPheremone = PheremoneType.None;
            %% Perform state actions
            switch uavBrain.currentState
                case UavState.Start
                    nextState = start(uavBrain, dt, messages);
                case UavState.Calibration
                    nextState = calibration(uavBrain, dt, messages);
                case UavState.Search
                    nextState = search(uavBrain, dt, messages);
                case UavState.Approach
                    nextState = approach(uavBrain, dt, messages);
                case UavState.Track
                    nextState = track(uavBrain, dt, messages);
                case UavState.Land
                    nextState = land(uavBrain, dt, messages);
                otherwise
                    error('UAV brain has invalid state.');
            end
            %% Update AI state and send commands
            uavBrain.lastState = uavBrain.currentState;
            % Allow 2 minutes to return to the landing position
            if uavBrain.uavBody.batteryLife < 120
                uavBrain.currentState = UavState.Land;
            else
                uavBrain.currentState = nextState;
            end
            uavBrain.uavBody.setVelocity(uavBrain.nextVelocity);
            uavBrain.uavBody.setTurnRate(uavBrain.nextTurnRate);
            uavBrain.bearingEstimate = wrapToPi(uavBrain.bearingEstimate + ...
                                       (uavBrain.nextTurnRate * uavBrain.nextVelocity * dt));
            uavBrain.updatePheremoneData(dt);
        end
        
        function draw(uavBrain, extra)
            if uavBrain.uavBody.operational
                if uavBrain.currentState == UavState.Search
                    plot(uavBrain.targetPos(1),uavBrain.targetPos(2),'x');
                end
                if extra
                    for i = 1:length(uavBrain.localPheremones)
                        if uavBrain.localPheremones(i).type == PheremoneType.Searched
                            pos = uavBrain.localPheremones(i).pos;
                            plot(pos(1),pos(2),'+b'); 
                        end
                    end
                end    
            end
        end
        
        %% State functions
        % Start state
        function nextState = start(uavBrain, dt, messages)
            uavBrain.nextVelocity = UavBody.MinVelocity;
            uavBrain.nextTurnRate = 0;
            uavBrain.home = uavBrain.posEstimate;
            if uavBrain.uavBody.operational
                nextState = UavState.Calibration;
            else
                nextState = UavState.Start;
            end
        end
                
        % Calibration state
        function nextState = calibration(uavBrain, dt, messages)
            %% State Actions
            if uavBrain.lastState ~= UavState.Calibration
                uavBrain.calibrationTimer = UavBrain.CalibrationTime;
            end
            uavBrain.calibrationTimer = uavBrain.calibrationTimer - dt;
            % Update bearing estimate
            estimatedMove = uavBrain.posEstimate - uavBrain.lastPosEstimate;
            
            calculatedMove = [sin(uavBrain.bearingEstimate), ...
                              cos(uavBrain.bearingEstimate)];
                          
            combinedMove = estimatedMove + calculatedMove;
            uavBrain.bearingEstimate = atan2(combinedMove(1), combinedMove(2));
            % Set movement values
            uavBrain.nextVelocity = 10;
            uavBrain.nextTurnRate = 0;
            
            %% State Transition
            % Transition state if enough total time has elapsed
            %if uavBrain.closestSqrDistance < UavBrain.AvoidanceRadius^2
            %    nextState = UavState.Avoidance;
            if (uavBrain.calibrationTimer <= 0)
                uavBrain.calibrated = true;
                nextState = UavState.Search;
            else
                nextState = UavState.Calibration;
            end
        end
        
        % Search state
        % UAV will continually travel on a random walk, preferring to
        % travel short distances.
        % Each time a step of the walk is completed, a pheremone will be
        % issued, preventing the area from being searched again by any UAV
        % for a while.
        % Avoidance behaviour is integrated into the search state by
        % finding a new random walk whenever the current path would lead to
        % a collision.
        function nextState = search(uavBrain, dt, messages)
            %% State Actions
            if uavBrain.lastState ~= UavState.Search
                uavBrain.targetPos = getRandomTarget(uavBrain, UavBrain.RandomWalkMaxDistance);
                uavBrain.nextPheremone = PheremoneType.Searched;
            end
            if uavBrain.retreating && uavBrain.conc < 0.05
                uavBrain.retreating = false;
            end
            targetDiff = uavBrain.targetPos - uavBrain.posEstimate;
            % If the target location has been reached or the target point
            % is invalid, find a new target location
            while sqrLen(targetDiff) <= (UavBody.MaxVelocity * dt)^2
                uavBrain.targetPos = getRandomTarget(uavBrain, UavBrain.RandomWalkMaxDistance);
                targetDiff = uavBrain.targetPos - uavBrain.posEstimate;
                uavBrain.nextPheremone = PheremoneType.Searched;
            end
            
            if uavBrain.closeUavCount > 0
                closestUavPos = messages(uavBrain.closestUav).pos;
                closestUavBearing = messages(uavBrain.closestUav).bearing;

                if uavBrain.uavPriority(messages(uavBrain.closestUav).id, messages(uavBrain.closestUav).state) ...
                        && uavBrain.closestSqrDist > UavBrain.DangerRadius^2
                    % Fly as usual, with minimum speed
                    uavBrain.flyToTarget(dt);
                    uavBrain.nextVelocity = UavBody.MinVelocity;
                else
                    % Find the avoidance trajectory and use
                    [uavBrain.nextVelocity, uavBrain.nextTurnRate] = ...
                        uavBrain.avoidanceTrajectory(closestUavPos, closestUavBearing, dt);
                end
            else
                uavBrain.flyToTarget(dt);
            end
            
            %% State Transition
            if uavBrain.conc > 0.05 && ~uavBrain.retreating
                nextState = UavState.Approach;
            else
                nextState = UavState.Search;
            end
        end
        
        % Approach state
        % UAV will circle into the cloud until it reaches the 1ppm contour
        % While approaching, the UAV will attempt to avoid other
        % approaching and orbiting UAVs.
        % Avoidance behaviour is similar to the orbiting behaviour - UAVs
        % will accelerate or decelerate depending on the relative positions
        % and trajectories of them and the approaching UAV.
        function nextState = approach(uavBrain, dt, messages)
            %% State Actions
            if uavBrain.lastState ~= UavState.Approach
                uavBrain.approachTimeout = UavBrain.ApproachTimeoutTime;
            else
                %uavBrain.approachTimeout = uavBrain.approachTimeout - dt;
            end
            concDiff = uavBrain.conc - uavBrain.lastConc;
            if uavBrain.closeUavCount > 0
                closestUavPos = messages(uavBrain.closestUav).pos;
                closestUavBearing = messages(uavBrain.closestUav).bearing;

                if uavBrain.uavPriority(messages(uavBrain.closestUav).id, messages(uavBrain.closestUav).state) ...
                        && uavBrain.closestSqrDist > UavBrain.DangerRadius^2
                    uavBrain.nextVelocity = UavBody.MinVelocity;
                    % If we are flying into the cloud and are within the map
                    % bounds, go straight ahead
                    if concDiff > 0 && ~uavBrain.leavingMap
                        uavBrain.nextTurnRate = 0;
                    % Otherwise, steer right
                    else
                        uavBrain.nextTurnRate = UavBody.MaxTurnRate * 0.5;
                    end
                else
                    % Find the avoidance trajectory and use
                    [uavBrain.nextVelocity, uavBrain.nextTurnRate] = ...
                        uavBrain.avoidanceTrajectory(closestUavPos, closestUavBearing, dt);
                    uavBrain.nextVelocity = UavBody.MaxVelocity;
                    % If the other UAV is tracking and travelling the same
                    % direction, use the Track avoidance method instead
                    if messages(uavBrain.closestUav).state == UavState.Track && ...
                            uavBrain.closestSqrDist > (UavBrain.DangerRadius+30)^2
                        angDiff = wrapToPi(closestUavBearing - uavBrain.bearingEstimate);
                        if abs(angDiff) < pi/4
                            avoidVec = closestUavPos - uavBrain.posEstimate;
                            avoidAng = atan2(avoidVec(1),avoidVec(2));
                            angDiff = wrapToPi(avoidAng - uavBrain.bearingEstimate);
                            front = abs(angDiff) < pi/2;
                            if front
                                uavBrain.nextVelocity = UavBody.MinVelocity;
                            else
                                uavBrain.nextVelocity = UavBody.MinVelocity + (UavBody.MaxVelocity - UavBody.MinVelocity)*2/3;
                            end
                            % If we are flying into the cloud, go straight ahead
                            if concDiff > 0
                                uavBrain.nextVelocity = UavBody.MaxVelocity;
                                uavBrain.nextTurnRate = 0;
                            % Otherwise, steer right
                            else
                                uavBrain.nextVelocity = UavBody.MinVelocity;
                                % TODO: This value is kind of dependent on dt for its
                                % sanity, realistically you only want a small angle change
                                % on each timestep to prevent it from getting stuck
                                uavBrain.nextTurnRate = UavBody.MaxTurnRate * 0.5;
                            end
                        end
                    end
                end
            else
                % If we are flying into the cloud and are within the map
                % bounds, go straight ahead
                if concDiff > 0 && ~uavBrain.leavingMap
                    uavBrain.nextVelocity = UavBody.MaxVelocity;
                    uavBrain.nextTurnRate = 0;
                % Otherwise, steer right
                else
                    uavBrain.nextVelocity = UavBody.MinVelocity;
                    % TODO: This value is kind of dependent on dt for its
                    % sanity, realistically you only want a small angle change
                    % on each timestep to prevent it from getting stuck
                    uavBrain.nextTurnRate = UavBody.MaxTurnRate * 0.5;
                end
            end
            
            %% State Transition
            % Transition state back to searching if concentration gets too
            % low, or on to tracking if it crosses the threshold
            if uavBrain.conc < 0.025;
                nextState = UavState.Search;
            elseif uavBrain.conc > 0.9
                nextState = UavState.Track;
            elseif uavBrain.approachTimeout <= 0
                uavBrain.retreating = true;
                nextState = UavState.Search;
            else
                nextState = UavState.Approach;
            end
        end
        
        % Track state
        function nextState = track(uavBrain, dt, messages)
            %% State Actions
            concDiff = uavBrain.conc - uavBrain.lastConc;
            % Used to calculate the turning rate/direction
            contourDistance = 1.0 - uavBrain.conc;
            if sign(concDiff) == sign(contourDistance)
                uavBrain.nextTurnRate = 0;
            else
                desiredTurnRate = contourDistance * 5 * UavBody.MaxTurnRate;
                uavBrain.nextTurnRate = max(min(desiredTurnRate,UavBody.MaxTurnRate/3),-UavBody.MaxTurnRate/5);
            end
            if uavBrain.closeUavCount > 0
                if uavBrain.closeUavCount > 1
                    if rand < dt*(1/30)
                        uavBrain.nextPheremone = PheremoneType.Searched;
                    end
                end
                closestUavPos = messages(uavBrain.closestUav).pos;
                closestUavBearing = messages(uavBrain.closestUav).bearing;
                if uavBrain.uavPriority(messages(uavBrain.closestUav).id, messages(uavBrain.closestUav).state) ...
                        && uavBrain.closestSqrDist > UavBrain.DangerRadius^2
                    avoidVec = closestUavPos - uavBrain.posEstimate;
                    avoidAng = atan2(avoidVec(1),avoidVec(2));
                    angDiff = wrapToPi(avoidAng - uavBrain.bearingEstimate);
                    front = abs(angDiff) < pi/2;
                    if front
                        uavBrain.nextVelocity = UavBody.MinVelocity;
                    else
                        uavBrain.nextVelocity = UavBody.MinVelocity + (UavBody.MaxVelocity - UavBody.MinVelocity)*2/3;
                    end
                else
                    angDiff = wrapToPi(closestUavBearing - uavBrain.bearingEstimate);
                    % If we are at serious risk of collision, take evasive maneuvers
                    if uavBrain.closestSqrDist < UavBrain.DangerRadius^2 ...
                            || abs(angDiff) > pi/4
                        if uavBrain.id < messages(uavBrain.closestUav).id
                            [uavBrain.nextVelocity, uavBrain.nextTurnRate] = ...
                                uavBrain.avoidanceTrajectory(closestUavPos, closestUavBearing, dt);
                            uavBrain.nextVelocity = UavBody.MaxVelocity;
                        else
                            uavBrain.nextVelocity = UavBody.MinVelocity;
                        end
                    % Otherwise, adjust speed to increase distance and carry on
                    else
                        avoidVec = closestUavPos - uavBrain.posEstimate;
                        avoidAng = atan2(avoidVec(1),avoidVec(2));
                        angDiff = wrapToPi(avoidAng - uavBrain.bearingEstimate);
                        front = abs(angDiff) < pi/2;
                        if front
                            uavBrain.nextVelocity = UavBody.MinVelocity;
                        else
                            uavBrain.nextVelocity = UavBody.MinVelocity + (UavBody.MaxVelocity - UavBody.MinVelocity)*2/3;
                        end
                    end
                end
            else
                uavBrain.nextVelocity = UavBody.MinVelocity + (UavBody.MaxVelocity - UavBody.MinVelocity)/3;
            end
            
            %% State Transition
            if uavBrain.conc < 0.25;
                nextState = UavState.Approach;
            else
                nextState = UavState.Track;
            end
        end
        
        function nextState = land(uavBrain, dt, messages)
            %% State Actions
            uavBrain.targetPos = uavBrain.home;
            
            if uavBrain.closeUavCount > 0
                closestUavPos = messages(uavBrain.closestUav).pos;
                closestUavBearing = messages(uavBrain.closestUav).bearing;

                if uavBrain.uavPriority(messages(uavBrain.closestUav).id, messages(uavBrain.closestUav).state) ...
                        && uavBrain.closestSqrDist > UavBrain.DangerRadius^2
                    % Fly to home steadily
                    uavBrain.flyToTarget(dt);
                    uavBrain.nextVelocity = UavBody.MinVelocity;
                else
                    % Find the avoidance trajectory and use
                    [uavBrain.nextVelocity, uavBrain.nextTurnRate] = ...
                        uavBrain.avoidanceTrajectory(closestUavPos, closestUavBearing, dt);
                end
            else
                uavBrain.flyToTarget(dt);
            end
            
            targetDiff = uavBrain.targetPos - uavBrain.posEstimate;
            % If the target location has been reached, land the UAV
            if sqrLen(targetDiff) <= (UavBody.MaxVelocity * dt)^2
                uavBrain.uavBody.disable;
            end
            
            %% State Transition
            nextState = UavState.Land;
        end
    end
    
end

