classdef Jointspace_Generator < handle

    properties
        cart1 = Cart(0, 30, -45, 45);
        cart2 = Cart(0, 30, -90, 90);
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
%                 lin3 = 0; % Use this for two tube tests
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
        
        % Creates a list of positions that are equally spaced, then
        % randomizes them. Used later in kinematics
        function positions_list = get_spaced_random_positions(self,num_positions)
            
            
%             rng(0,'twister');
            rng('shuffle');
            randi([-10 10],1,1000);
            
            arr1 = linspace(self.cart1.lin_min,self.cart1.lin_max,(self.cart1.lin_max-self.cart1.lin_min)/5+1);
            arr2 = linspace(self.cart2.lin_min,self.cart2.lin_max,(self.cart2.lin_max-self.cart2.lin_min)/5+1);
            arr3 = linspace(self.cart3.lin_min,self.cart3.lin_max,(self.cart3.lin_max-self.cart3.lin_min)/5+1);
            arr4 = linspace(self.cart1.rot_min,self.cart1.rot_max,(self.cart1.rot_max-self.cart1.rot_min)/10+1);
            arr5 = linspace(self.cart2.rot_min,self.cart2.rot_max,(self.cart2.rot_max-self.cart2.rot_min)/10+1);
            arr6 = linspace(self.cart3.rot_min,self.cart3.rot_max,(self.cart3.rot_max-self.cart3.rot_min)/10+1);
%             size_all = size(arr1, 2) + size(arr2, 2) + size(arr3, 2) + size(arr4, 2) + size(arr5, 2) + size(arr6, 2);

            f_array = [];
            for a = 1:size(arr6,2)
                for b = 1:size(arr5,2)
                    for c = 1:size(arr4,2)
                        for d = 1:size(arr3,2)
                            for e = 1:size(arr2,2)
                                for f = 1:size(arr1,2)
                                    f_array = [f_array; [arr1(1,f), arr2(1,e), arr3(1,d), arr4(1,c), arr5(1,b), arr6(1,a)]] ;
                                end
                            end
                        end
                    end
                end
            end

            disp(size(f_array))


            ctr = 1;
            for i = 1:num_positions
%                 new_pose = Pose(lin1, (lin1+lin2), (lin1+lin2+lin3), rot1, rot2, rot3);
                new_pose = Pose(f_array(ctr,1), (f_array(ctr,1)+f_array(ctr,2)), (f_array(ctr,1)+f_array(ctr,2)+f_array(ctr,3)), ...
                    f_array(ctr,4), f_array(ctr,5), f_array(ctr,6));
                positions_list(i,1) = new_pose;
                ctr = ctr+uint16(size(f_array,1)/num_positions);
%                 ctr = ctr+10;
                
            end

            v = positions_list(randperm(length(positions_list)));
            positions_list = [[Pose(0,0,0,0,0,0)];positions_list];

            self.positions_list = positions_list;
            self.positions_list_string = self.positions_list_to_string();
            self.positions_gcode_string = self.positions_list_to_gcode_string();
            self.positions_gcode_list = self.positions_list_to_gcode_list();
%                 self.positions_as_list = self.get_pose_as_list()
%                 print(self.robot1.fkin([lin1, lin1+lin2, lin1+lin2+lin3, rot1, rot2, rot3]))
        end

        function positions_list = get_all_spaced_positions_2tubes(self,num_positions)
            
            
%             rng(0,'twister');
            rng('shuffle');
            randi([-10 10],1,1000);
            
            lin_arr = linspace(self.cart2.lin_min,self.cart2.lin_max,(self.cart2.lin_max-self.cart2.lin_min)/5+1);
            rot_arr = linspace(self.cart2.rot_min,self.cart2.rot_max,(self.cart2.rot_max-self.cart2.rot_min)/5+1);
            
            f_array = [];
            for e = 1:size(lin_arr,2)
                for f = 1:size(rot_arr,2)
                        f_array = [f_array; [lin_arr(1,e), rot_arr(1,f)]] ;
                end
            end

            for ctr = 1:size(f_array,1)
%                 new_pose = Pose(lin1, (lin1+lin2), (lin1+lin2+lin3), rot1, rot2, rot3);
                new_pose = Pose(0, (f_array(ctr,1)), 0, 0, f_array(ctr,2), 0);
                positions_list(ctr,1) = new_pose;
            end

            self.positions_list = positions_list;
            self.positions_list_string = self.positions_list_to_string();
            self.positions_gcode_string = self.positions_list_to_gcode_string();
            self.positions_gcode_list = self.positions_list_to_gcode_list();
        
        end

        % Returns a string of joint positions; not used for any code --
        % diagnostic only
        function string = positions_list_to_string(self)
            string = "";
            for i = 1:size(self.positions_list, 1)
                pose = self.positions_list(i,1);
                command = pose.get_string_for_pose();
                string = string + command + "\n";
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
                gcode_list(i,1) = command + "\n";
            end
       end
    end
end

