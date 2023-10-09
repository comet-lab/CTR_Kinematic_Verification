clear% Initialize test information
num_tubes = 3; % number of tubes for test
num_pts = 20;  % number of test points
rotation = true; % true if test includes rotation

% Create a random list of joint variables
generator = Jointspace_Generator();

joint_list = generator.get_new_random_positions(num_pts);
% joint_list = generator.get_spaced_random_positions(num_pts);
% joint_list = generator.get_all_spaced_positions_2tubes(num_pts);


gcode_list = generator.positions_gcode_list();
pose_list = generator.pose_list();

comb_list = [gcode_list, string(pose_list)];

size(gcode_list,1)
% Define and open the serial port


% Initialize data collection arrays
data_arr = zeros(num_pts+1, 20);


% Save the data to a folder with the current data and time
date = datestr(now, 'dd/mm/yy-HH:MM');
date(3) = "_";
date(6) = "_";
date(12) = "_";

if rotation
    test_string = "rotate";
else
    test_string = "in-plane-bending";
end

% filename = "data_files/" + date + "_" + int2str(num_tubes) + "-tubes_" + test_string + ".csv";

filename = "data_files/jointvariables.csv";
writematrix(comb_list, filename);