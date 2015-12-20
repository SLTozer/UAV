classdef UavBrain < handle
    
    properties
        currentState;
        uavBody;
        posEstimate;
        bearingEstimate;
        
        nextTurnRate;
        nextVelocity;
        
        targetPos;
        conc;
    end
    
    methods
        function uavBrain = UavBrain(uavBody)
            uavBrain.uavBody = uavBody;
            uavBrain.currentState = UavState.CalibrationState;
            uavBrain.posEstimate = uavBody.getGpsPos();
            uavBrain.bearingEstimate = 0;
            uavBrain.targetPos = [0,0];
            uavBrain.conc = 0;
        end
        
        function decisionStep(uavBrain, cloud, t, dt, messages)
            switch uavBrain.currentState
                case UavState.CalibrationState
                    nextState = calibration(uavBrain, cloud, t, dt);
                case UavState.SearchState
                    nextState = search(uavBrain, cloud, t, dt);
                case UavState.ApproachState
                    nextState = approach(uavBrain, cloud, t, dt);
                case UavState.TrackState
                    nextState = track(uavBrain, cloud, t, dt);
                otherwise
                    error('UAV brain has invalid state.');
            end
            uavBrain.currentState = nextState;
            uavBrain.uavBody.setVelocity(uavBrain.nextVelocity);
            uavBrain.uavBody.setTurnRate(uavBrain.nextTurnRate);
            uavBrain.bearingEstimate = wrapToPi(uavBrain.bearingEstimate + ...
                                       (uavBrain.nextTurnRate * uavBrain.nextVelocity * dt));
        end
        
        function message = getMessage(uavBrain)
            message = zeros(1,4);
            message(1) = uavBrain.posEstimate(1);
            message(2) = uavBrain.posEstimate(2);
            message(3) = uavBrain.bearingEstimate;
            message(4) = uavBrain.conc;
        end
        
        function nextState = calibration(uavBrain, cloud, t, dt)
            % Update bearing estimate
            newPosEstimate = uavBrain.uavBody.getGpsPos();
            estimatedMove = newPosEstimate - uavBrain.posEstimate;
            
            calculatedMove = [sin(uavBrain.bearingEstimate), ...
                              cos(uavBrain.bearingEstimate)];
                          
            combinedMove = estimatedMove + calculatedMove;
            % Update internal information
            uavBrain.posEstimate = newPosEstimate;
            uavBrain.bearingEstimate = atan2(combinedMove(1), combinedMove(2));
            % Set movement values
            uavBrain.nextVelocity = 10;
            uavBrain.nextTurnRate = 0;
            % Transition state if enough total time has elapsed
            if (t > 15)
                nextState = UavState.SearchState;
            else
                nextState = UavState.CalibrationState;
            end
        end
        function nextState = search(uavBrain, cloud, t, dt)
            uavBrain.posEstimate = uavBrain.uavBody.getGpsPos();
            targetDiff = uavBrain.targetPos - uavBrain.posEstimate;
            % If the target location has been reached, find a new target
            % location
            while norm(targetDiff) <= UavBody.MaxVelocity * dt
                uavBrain.targetPos = [(rand * (max(cloud.x)-min(cloud.x))) + min(cloud.x), ...
                                      (rand * (max(cloud.y)-min(cloud.y))) + min(cloud.x)];
            targetDiff = uavBrain.targetPos - uavBrain.posEstimate;
            end
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
            
            uavBrain.conc = uavBrain.uavBody.sensorReading(cloud, t);
            % Transition state if the cloud is detected
            if uavBrain.conc > 0.05
                nextState = UavState.ApproachState;
            else
                nextState = UavState.SearchState;
            end
        end
        function nextState = approach(uavBrain, cloud, t, dt)
            newConc = uavBrain.uavBody.sensorReading(cloud, t);
            concDiff = newConc - uavBrain.conc;
            % If we are flying into the cloud, turn a gentle right
            if concDiff > 0
                uavBrain.nextVelocity = UavBody.MaxVelocity;
                uavBrain.nextTurnRate = pi/180;
            % Otherwise, steer a hard right
            else
                uavBrain.nextVelocity = UavBody.MinVelocity;
                uavBrain.nextTurnRate = UavBody.MaxTurnRate;
            end
            % Assign internal state
            uavBrain.conc = newConc;
            % Transition state back to searching if concentration gets too
            % low, or on to tracking if it crosses the threshold
            if uavBrain.conc < 0.025;
                nextState = UavState.SearchState;
            elseif uavBrain.conc > 1.0
                nextState = UavState.ApproachState;%TrackState;
            else
                nextState = UavState.ApproachState;
            end
        end
        function nextState = track(uavBrain, cloud, t, dt)
            % Yet to be implemented
        end
        
    end
    
end

