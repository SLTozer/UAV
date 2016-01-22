classdef UavBrain < handle
    
    % UAV behavioural constants
    properties (Constant)
        AvoidanceRadius = 70;
        FirstTarget = [0 100];
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
        posEstimate;
        lastPosEstimate;
        bearingEstimate;
        conc;
        lastConc;
        closestUav;
        closestSqrDist;
    end
    % UAV AI state
    % Used to control the actions taken by the UAVs AI
    properties (SetAccess = public, GetAccess = public)
        currentState;
        lastState;
        targetPos;
    end
    
    methods
        function uavBrain = UavBrain(uavBody, mapRect)
            uavBrain.uavBody = uavBody;
            uavBrain.currentState = UavState.Start;
            uavBrain.posEstimate = uavBody.getGpsPos();
            uavBrain.bearingEstimate = 0;
            uavBrain.targetPos = [0,0];
            uavBrain.conc = 0;
            uavBrain.mapRect = mapRect;
        end
        
        function [closestUav, closestSqrDist] = findClosestUav(uavBrain, messages, ownMessage)
            closestSqrDist = inf;
            closestUav = 0;
            messageCount = size(messages(:,1));
            for i = 1:messageCount
                if i == ownMessage || messages(i).state ~= UavState.Inactive
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
        
        function message = getMessage(uavBrain)
            message = Message();
            message.pos = uavBrain.posEstimate;
            message.state = uavBrain.currentState;
            message.conc = uavBrain.conc;
        end
        
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
        
        % Make decisions at time t
        function decisionStep(uavBrain, cloud, t, dt, messages, ownMessage)
            % Update UAV data
            uavBrain.lastPosEstimate = uavBrain.posEstimate;
            uavBrain.lastConc = uavBrain.conc;
            uavBrain.posEstimate = uavBrain.uavBody.getGpsPos();
            uavBrain.conc = uavBrain.uavBody.sensorReading(cloud, t);
            [uavBrain.closestUav, uavBrain.closestSqrDist] = ...
                uavBrain.findClosestUav(messages, ownMessage);
            % Perform state actions
            switch uavBrain.currentState
                case UavState.Start
                    nextState = start(uavBrain, cloud, t, dt);
                case UavState.Calibration
                    nextState = calibration(uavBrain, cloud, t, dt);
                case UavState.Search
                    nextState = search(uavBrain, cloud, t, dt);
                case UavState.Approach
                    nextState = approach(uavBrain, cloud, t, dt);
                case UavState.Track
                    nextState = track(uavBrain, cloud, t, dt);
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
        end
        
        %% State functions
        % Start state
        function nextState = start(uavBrain, cloud, t, dt, messages)
            uavBrain.nextVelocity = UavBody.MinVelocity;
            uavBrain.nextTurnRate = 0;
            if uavBrain.uavBody.operational
                nextState = UavState.Search;
            else
                nextState = UavState.Start;
            end
        end
        
        % Avoidance state
        function nextState = avoidance(uavBrain, cloud, t, dt, messages)
            %% State Actions
            closestUavPos = messages(uavBrain.closestUav,1:2);
            % Simply set a target point far away from the closest UAV
            uavBrain.targetPos = ...
                uavBrain.posEstimate ...
                - ((closestUavPos - uavBrain.posEstimate) * 100);
            uavBrain.flyToTarget(dt);
            
            %% State Transition
            if uavBrain.closestSqrDist > UavBrain.AvoidanceRadius^2
                uavBrain.targetPos = [NaN NaN];
                nextState = UavState.Search;
            else
                nextState = UavState.Avoidance;
            end
        end
        
        % Calibration state
        function nextState = calibration(uavBrain, cloud, t, dt)
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
            if uavBrain.closestSqrDistance < UavBrain.AvoidanceRadius^2
                nextState = UavState.Avoidance;
            elseif (t > 15)
                uavBrain.calibrated = true;
                nextState = UavState.Search;
            else
                nextState = UavState.Calibration;
            end
        end
        
        % Search state
        function nextState = search(uavBrain, cloud, t, dt)
            %% State Actions
            if uavBrain.lastState ~= UavState.Search
                uavBrain.targetPos = ...
                    [(rand * (uavBrain.mapRect(2,1)-uavBrain.mapRect(1,1))) + uavBrain.mapRect(1,1), ...
                    (rand * (uavBrain.mapRect(2,2)-uavBrain.mapRect(1,2))) + uavBrain.mapRect(1,2)];
            end
            targetDiff = uavBrain.targetPos - uavBrain.posEstimate;
            % If the target location has been reached or the target point
            % is invalid, find a new target location
            while norm(targetDiff) <= UavBody.MaxVelocity * dt
                uavBrain.targetPos = ...
                    [(rand * (uavBrain.mapRect(2,1)-uavBrain.mapRect(1,1))) + uavBrain.mapRect(1,1), ...
                    (rand * (uavBrain.mapRect(2,2)-uavBrain.mapRect(1,2))) + uavBrain.mapRect(1,2)];
                targetDiff = uavBrain.targetPos - uavBrain.posEstimate;
            end
            uavBrain.flyToTarget(dt);
            
            %% State Transition
            if uavBrain.conc > 0.05
                nextState = UavState.Approach;
            else
                nextState = UavState.Search;
            end
        end
        
        % Approach state
        function nextState = approach(uavBrain, cloud, t, dt)
            %% State Actions
            concDiff = uavBrain.conc - uavBrain.lastConc;
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
            
            %% State Transition
            % Transition state back to searching if concentration gets too
            % low, or on to tracking if it crosses the threshold
            if uavBrain.conc < 0.025;
                nextState = UavState.Search;
            elseif uavBrain.conc > 1.0
                nextState = UavState.Track;
            else
                nextState = UavState.Approach;
            end
        end
        
        % Track state
        function nextState = track(uavBrain, cloud, t, dt)
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
                desiredTurnRate = sqrt(abs(contourDistance)) * sign(contourDistance) * UavBody.MaxTurnRate;
                uavBrain.nextTurnRate = max(min(desiredTurnRate,UavBody.MaxTurnRate),-UavBody.MaxTurnRate);
            end
            uavBrain.nextVelocity = UavBody.MinVelocity;
            
            %% State Transition
            if uavBrain.conc < 0.25;
                nextState = UavState.Approach;
            else
                nextState = UavState.Track;
            end
        end
        
    end
    
end

