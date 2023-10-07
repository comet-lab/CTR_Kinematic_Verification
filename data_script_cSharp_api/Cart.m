classdef Cart < handle
    
    properties
        lin_min = 0
        lin_max = 0
        rot_min = 0
        rot_max = 0
    end
    
    methods
        function self = Cart(lin_min, lin_max, rot_min, rot_max)
            %CART Construct an instance of this class
            %   Set min and max values
            self.lin_min = lin_min;
            self.lin_max = lin_max;
            self.rot_min = rot_min;
            self.rot_max = rot_max;
        end
        
        function range = get_lin_range(self)
            range = [self.lin_min, self.lin_max];
        end
        
        function range = get_rot_range(self)
            range = [self.rot_min, self.rot_max];
        end
    end
end

