classdef UavBrain < handle
    
    % UAV behavioural constants
    properties (Constant)
        AvoidanceRadius = 135;
        RandomWalkMaxDistance = 600.0;
        FirstTarget = [0 100];
        PheremoneDetectionRange = 800.0;
        PheremoneAvoidanceRange = 300.0;
        ApproachTimeoutTime = 100;
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
        closeUavs;
        closeSqrDists;
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
        nextPheremone;
        % State data
        approachTimeout;
        searchTimeout;
        retreating;
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
            uavBrain.localPheremones = [];
            uavBrain.nextPheremone = PheremoneType.None;
            uavBrain.approachTimeout = 0;
            uavBrain.retreating = false;
        end
        
        function [closestUav, closestSqrDist] = findClosestUav(uavBrain, messages)
            closestSqrDist = inf;
            closestUav = 0;
            messageCount = size(messages(:,1));
            for i = 1:messageCount
                if messages(i).id == uavBrain.id || messages(i).state ~= UavState.Inactive
                    continue
                end
                differenceVector = uavBrain.posEstimate - messages(i).pos;
                squaredDistance = ...
                    (differenceVector(1)*differenceVector(1)) ...
                    * (differenceVector(2)*differenceVector(2));
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
                message.id = uavBrain.id;
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
                if messages(i).pheremoneType ~= PheremoneType.None
                    pheremoneVec = messages(i).pos - uavBrain.posEstimate;
                    sqrPheremoneDist = pheremoneVec(1)^2 + pheremoneVec(2)^2;
                    if sqrPheremoneDist <= UavBrain.PheremoneDetectionRange^2
                        pheremone = Pheremone(messages(i).pos, messages(i).pheremoneType);
                        uavBrain.localPheremones = [uavBrain.localPheremones pheremone];
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
            for i = 1:length(uavBrain.localPheremones)
                uavBrain.localPheremones(i).duration = ...
                    uavBrain.localPheremones(i).duration - dt;
            end
            startLen = length(uavBrain.localPheremones);
            for i = 0:startLen-1
                if uavBrain.localPheremones(startLen-i).duration <= 0
                    uavBrain.localPheremones(startLen-i) = [];
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
                if currentRadius < (uavBrain.mapRect(2,1) - uavBrain.mapRect(1,1))/2
                    currentRadius = currentRadius + 100.0;
                end
                attemptCount = attemptCount + 1;
            end
        end
        
        % Make decisions at time t
        function decisionStep(uavBrain, cloud, t, dt, messages)
            if uavBrain.uavBody.operational == false
                return;
            end
            % Update UAV data
            uavBrain.lastPosEstimate = uavBrain.posEstimate;
            uavBrain.lastConc = uavBrain.conc;
            uavBrain.posEstimate = uavBrain.uavBody.getGpsPos();
            uavBrain.conc = uavBrain.uavBody.sensorReading(cloud, t);
            [uavBrain.closeUavs, uavBrain.closeSqrDists] = ...
                uavBrain.findCloseUavs(messages);
%             if uavBrain.currentState ~= UavState.Avoidance && ~isempty(uavBrain.closeUavs)
%                 uavBrain.lastState = uavBrain.currentState;
%                 uavBrain.currentState = UavState.Avoidance;
%             elseif uavBrain.currentState == UavState.Avoidance && isempty(uavBrain.closeUavs)
%                 uavBrain.currentState = uavBrain.lastState;
%                 uavBrain.lastState = UavState.Avoidance;
%             end
            uavBrain.getPheremoneData(messages);
            uavBrain.nextPheremone = PheremoneType.None;
            % Perform state actions
            switch uavBrain.currentState
                case UavState.Start
                    nextState = start(uavBrain, cloud, t, dt, messages);
                case UavState.Avoidance
                    nextState = avoidance(uavBrain, cloud, t, dt, messages);
                case UavState.Calibration
                    nextState = calibration(uavBrain, cloud, t, dt, messages);
                case UavState.Search
                    nextState = search(uavBrain, cloud, t, dt, messages);
                case UavState.Approach
                    nextState = approach(uavBrain, cloud, t, dt, messages);
                case UavState.Track
                    nextState = track(uavBrain, cloud, t, dt, messages);
                otherwise
                    error('UAV brain has invalid state.');
            end
            % Update AI state and send commands
            uavBrain.lastState = uavBrain.currentState;
            uavBrain.currentState = nextState;
            uavBrain.uavBody.setVelocity(uavBrain.nextVelocity);
            uavBrain.uavBody.setTurnRate(uavBrain.nextTurnRate);
            uavBrain.bearingEstimate = wrapToPi(uavBrain.bearingEstimate + ...
                                       (uavBrain.nextTurnRate * uavBrain.nextVelocity * dt));
            uavBrain.updatePheremoneData(dt);
        end
        
        %% State functions
        % Start state
        function nextState = start(uavBrain, cloud, t, dt, messages)
            uavBrain.nextVelocity = UavBody.MinVelocity;
            uavBrain.nextTurnRate = 0;
            if uavBrain.uavBody.operational
                nextState = UavState.Calibration;
            else
                nextState = UavState.Start;
            end
        end
        
        % Avoidance state
        function nextState = avoidance(uavBrain, cloud, t, dt, messages)
            %% State Actions
            [~, closestUavInd] = min(uavBrain.closeSqrDists);
            closestUav = uavBrain.closeUavs(closestUavInd);
            closestUavPos = messages(closestUav).pos;
            closestUavBearing = messages(closestUav).bearing;
            
            % Find the avoidance trajectory and use
            [uavBrain.nextVelocity, uavBrain.nextTurnRate] = ...
                uavBrain.avoidanceTrajectory(closestUavPos, closestUavBearing, dt);
            
            %% State Transition
        end
        
        % Calibration state
        function nextState = calibration(uavBrain, cloud, t, dt, messages)
            %% State Actions
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
            if (t > 30)
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
        function nextState = search(uavBrain, cloud, t, dt, messages)
            %% State Actions
            if uavBrain.lastState ~= UavState.Search && ...
                    uavBrain.lastState ~= UavState.Avoidance
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
            
            %uavBrain.targetPos = [0 0];
            
            if ~isempty(uavBrain.closeUavs)
                [~, closestUavInd] = min(uavBrain.closeSqrDists);
                closestUav = uavBrain.closeUavs(closestUavInd);
                closestUavPos = messages(closestUav).pos;
                closestUavBearing = messages(closestUav).bearing;

                % Find the avoidance trajectory and use
                [uavBrain.nextVelocity, uavBrain.nextTurnRate] = ...
                    uavBrain.avoidanceTrajectory(closestUavPos, closestUavBearing, dt);
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
        function nextState = approach(uavBrain, cloud, t, dt, messages)
            %% State Actions
            if uavBrain.lastState ~= UavState.Approach
                uavBrain.approachTimeout = UavBrain.ApproachTimeoutTime;
            else
                %uavBrain.approachTimeout = uavBrain.approachTimeout - dt;
            end
            concDiff = uavBrain.conc - uavBrain.lastConc;
            if ~isempty(uavBrain.closeUavs)
                [~, closestUavInd] = min(uavBrain.closeSqrDists);
                closestUav = uavBrain.closeUavs(closestUavInd);
                closestUavPos = messages(closestUav).pos;
                closestUavBearing = messages(closestUav).bearing;

                % Find the avoidance trajectory and use
                [uavBrain.nextVelocity, uavBrain.nextTurnRate] = ...
                    uavBrain.avoidanceTrajectory(closestUavPos, closestUavBearing, dt);
                uavBrain.nextVelocity = UavBody.MaxVelocity;
            else
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
            
            %% State Transition
            % Transition state back to searching if concentration gets too
            % low, or on to tracking if it crosses the threshold
            if uavBrain.conc < 0.025;
                nextState = UavState.Search;
            elseif uavBrain.conc > 1.0
                nextState = UavState.Track;
            elseif uavBrain.approachTimeout <= 0
                uavBrain.retreating = true;
                nextState = UavState.Search;
            else
                nextState = UavState.Approach;
            end
        end
        
        % Track state
        function nextState = track(uavBrain, cloud, t, dt, messages)
            %% State Actions
            concDiff = uavBrain.conc - uavBrain.lastConc;
            % Used to calculate the turning rate/direction
            contourDistance = 1.0 - uavBrain.conc;
%             if sign(contourDistance) == 1
%                 contourDistance = contourDistance - 0.05;
%             else
%                 contourDistance = contourDistance + 0.05;
%             end
            if sign(concDiff) == sign(contourDistance)
                uavBrain.nextTurnRate = 0;
            else
                %desiredTurnRate = contourDistance * 2 * UavBody.MaxTurnRate;%sqrt(abs(contourDistance)) * sign(contourDistance) * UavBody.MaxTurnRate;
                desiredTurnRate = contourDistance * 5 * UavBody.MaxTurnRate;
                %desiredTurnRate = -(concDiff^2 * sign(concDiff)) * 50 * UavBody.MaxTurnRate;
                uavBrain.nextTurnRate = max(min(desiredTurnRate,UavBody.MaxTurnRate/3),-UavBody.MaxTurnRate/5);
            end
            if ~isempty(uavBrain.closeUavs)
                [~, closestUavInd] = min(uavBrain.closeSqrDists);
                closestUav = uavBrain.closeUavs(closestUavInd);
                closestUavPos = messages(closestUav).pos;

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
                uavBrain.nextVelocity = UavBody.MinVelocity + (UavBody.MaxVelocity - UavBody.MinVelocity)/3;
            end
            
            %% State Transition
            if uavBrain.conc < 0.25;
                nextState = UavState.Approach;
            else
                nextState = UavState.Track;
            end
        end
        
        function draw(uavBrain)
            if uavBrain.currentState == UavState.Search
                plot(uavBrain.targetPos(1),uavBrain.targetPos(2),'x');
            end
%             for i = 1:length(uavBrain.localPheremones)
%                 if uavBrain.localPheremones(i).type == PheremoneType.Searched
%                     pos = uavBrain.localPheremones(i).pos;
%                     plot(pos(1),pos(2),'+b'); 
%                 end
%             end
        end
    end
    
end

