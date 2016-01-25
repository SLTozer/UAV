classdef Message
    % Message sent by a UAV; no larger than 32 bytes
    properties
        % 16 bytes
        pos;
        % 8 bytes
        bearing;
        % 1 byte
        state;
        % 1 byte
        pheremoneType;
        % 1 byte
        id;
    end    
    
    methods
        function message = Message()
            message.pos = [0,0];
            message.id = -1;
            message.bearing = 0;
            message.state = UavState.Inactive;
            message.pheremoneType = PheremoneType.None;
        end
    end
end

