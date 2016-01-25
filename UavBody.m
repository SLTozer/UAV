classdef UavBody < handle
    
    properties (Constant)
        MaxTurnRate = pi/30;
        MinVelocity = 10;
        MaxVelocity = 20;
        CrashRadius = 10;
    end
    
    properties (SetAccess = private, GetAccess = public)
        pos
        bearing
    end
    properties (SetAccess = private, GetAccess = public)
        vel
        turnRate
        operational
    end
    
    methods
        function uavBody = UavBody(pos, bearing)
            uavBody.pos = pos;
            uavBody.bearing = bearing;
            uavBody.vel = 15;
            uavBody.turnRate = 0;
            uavBody.operational = true;
        end
        
        function move(uavBody, dt)
            if uavBody.operational
%                 distance = uavBody.vel * dt;
%                 moveVec = [sin(uavBody.bearing),cos(uavBody.bearing)] * distance;
%                 uavBody.pos = uavBody.pos + moveVec;
%                 uavBody.bearing = wrapToPi(uavBody.bearing + (uavBody.turnRate * distance));
                [uavBody.pos, uavBody.bearing] = ...
                    moveRungeKutta( uavBody.pos, uavBody.bearing, uavBody.vel, uavBody.turnRate, dt );
            end
        end
        
        function setVelocity(uavBody, newVel)
            if (newVel > UavBody.MaxVelocity) || (newVel < UavBody.MinVelocity)
                error('Error: UAV velocity out of bounds.');
            end
            uavBody.vel = newVel;
        end
        
        function setTurnRate(uavBody, newTurnRate)
            if (abs(newTurnRate) > UavBody.MaxTurnRate)
                error('Error: UAV turn rate out of bounds.');
            end
            uavBody.turnRate = newTurnRate;
        end
        
        function fuzzPos = getGpsPos(uavBody)
            fuzzAngle = rand * 2 * pi;
            fuzzDist = rand * 3;
            fuzzPos = uavBody.pos + ([sin(fuzzAngle),cos(fuzzAngle)] * fuzzDist);
        end
        
        function conc = sensorReading(uavBody, cloud, t)
            conc = cloudsamp(cloud, uavBody.pos(1), uavBody.pos(2), t);
        end
        
        function plot(uavBody)
            plot(uavBody.pos(1),uavBody.pos(2),'o');
        end
    end
    
end

