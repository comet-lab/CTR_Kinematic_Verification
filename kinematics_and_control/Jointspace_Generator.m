classdef Jointspace_Generator < handle

    properties

        cart1 = Cart(0, 40, -90, 0);
        cart2 = Cart(0, 40, -90, 0);
        cart3 = Cart(0, 0, 0, 0);


        
        % Lists or the positions in different formats
        positions_list = []
        positions_list_string = []
        positions_gcode_string = ""
        positions_gcode_list = []
    end
    
    methods
        function self = Jointspace_Generator() 
        end
        
        % Creates and returns array of random positions in joint space
        % format; used later for kinematics calculations
        function positions_list = get_new_random_positions(self,num_positions)
            
            positions_list = [Pose(0,0,0,0,0,0)];
%             rng(0,'twister');  % This makes positions random and not repeatable
            rng('shuffle'); % This makes positions random but will return the same random position each time
            randi([-10 10],1,1000);
            for i = 1:num_positions
                lin1 = randi(self.cart1.get_lin_range(), 1, 1);
                lin2 = randi(self.cart2.get_lin_range(), 1, 1);
                lin3 = randi(self.cart3.get_lin_range(), 1, 1);
                rot1 = randi(self.cart1.get_rot_range(), 1, 1);
                rot2 = randi(self.cart2.get_rot_range(), 1, 1);
                rot3 = randi(self.cart3.get_rot_range(), 1, 1);

                new_pose = Pose(lin1, (lin1+lin2), (lin1+lin2+lin3), rot1, rot2, rot3);
                positions_list(i+1,1) = new_pose;

                self.positions_list = positions_list;
                self.positions_list_string = self.positions_list_to_string();
                self.positions_gcode_string = self.positions_list_to_gcode_string();
                self.positions_gcode_list = self.positions_list_to_gcode_list();
            end
        end
        
        
        % Returns a string of joint positions; not used for any code --
        % diagnostic only
        function positions_string_list = positions_list_to_string(self)
            positions_string_list = [""];
            for i = 1:size(self.positions_list, 1)
                pose = self.positions_list(i,1);
                command = pose.get_string_for_pose();
                positions_string_list(i,1) = string + command + "\n";
            end
        end
        
       % Returns string of gcode for positions; not used for any code --
       % diagnostic only
       function string = positions_list_to_gcode_string(self)
            string = "";
            for i = 1:size(self.positions_list, 1)
                pose = self.positions_list(i,1);
                command = pose.get_gcode_for_pose();
                string = string + command + "\n";
            end
       end
        
       % Returns an array of gcode to be used in main script
       function gcode_list = positions_list_to_gcode_list(self)
            gcode_list = [""];
            for i = 1:size(self.positions_list, 1)
                pose = self.positions_list(i,1);
                command = pose.get_gcode_for_pose();
                gcode_list(i,1) = "G0 " +  command + "\n";
            end
       end

       function poses = pose_list(self)
           poses = [];
           for i = 1:size(self.positions_list, 1)
               pose = self.positions_list(i,1);
               pose = pose.get_pose();
               poses(i,:) = pose;
           end
       end

    end
end

