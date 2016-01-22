classdef Message
    % Message sent by a UAV; no larger than 32 bytes
    properties
        pos;
        conc;
        state;
    end    
    
    methods
        function message = Message()
            message.pos = [0,0];
            message.state = UavState.Inactive;
            message.conc = 0;
        end
    end
end

