clear% Initialize test information
num_tubes = 2; % number of tubes for test
num_pts = 20;  % number of test points
rotation = true; % true if test includes rotation
COMPort = "COM3";

% Create a random list of joint variables
generator = Jointspace_Generator();

% joint_list = generator.get_new_random_positions(num_pts);
% joint_list = generator.get_spaced_random_positions(num_pts);
joint_list = generator.get_all_spaced_positions_2tubes(num_pts);


gcode_list = generator.positions_gcode_list();
size(gcode_list,1)
% Define and open the serial port
ser = serialport(COMPort, 250000);

% Initialize data collection arrays
data_arr = zeros(1*10^3, 20);
frame_count = 0;

% theClient = NatNetML.NatNetClientML(0); % Input = iConnectionType: 0 = Multicast, 1 = Unicast
% 
% % Connect to an OptiTrack server (Motive)
% HostIP = char('130.215.211.19');
% theClient.Initialize(HostIP, HostIP);

disp('OptiTrack NatNet connected!')

% Iterate thru points to move robot and collect data
for i = 1:size(gcode_list,1)
    fopen(ser);

    % Send the postion command from the list of positions
    writeline(ser, gcode_list(i,1));

    % Tell the actuator to wait for previous command to finish and send the
    % current iteration that it finished
    writeline(ser, 'M400\n');
    writeline(ser, 'M118 '+ string(i) +'\n');
%     echo_str = 'M118 '+ string(i) +'\n';
%     writeline(ser, echo_str);

    % Read the data return for the correct number. Here we only read one
    % character beacuse, well, for lack of better explanation, MATLAB is a
    % pain and there really is not a better way to do this
    data_read = fscanf(ser);
    expc = int2str(i);
    while not(strcmp(data_read(1),expc(1)))
        data_read = fscanf(ser);
    end

    disp(i);  % Display the current joint position

%     % Take 10 data points per position
%     for t = 1:10
%         frame_count = frame_count + 1;
% 
% %         frameOfData = theClient.GetLastFrameOfData();
%         frameOfData = theClient.GetLastFrameOfData();
%         
%         % Here the robot tip MUST be the first rigid body recorded.
%         % Otherwise the data will reflect a different rigid body and be
%         % completely useless. 
% 
%         robot_tip = frameOfData.RigidBodies(1);
%         data_arr(frame_count, 1) = robot_tip.x;
%         data_arr(frame_count, 2) = robot_tip.y;
%         data_arr(frame_count, 3) = robot_tip.z;
%     
%         data_arr(frame_count, 4) = robot_tip.qw;
%         data_arr(frame_count, 5) = robot_tip.qx;
%         data_arr(frame_count, 6) = robot_tip.qy;
%         data_arr(frame_count, 7) = robot_tip.qz;
% 
%         data_arr(frame_count, 8) = joint_list(i,1).lin1;
%         data_arr(frame_count, 9) = joint_list(i,1).lin2;
%         data_arr(frame_count, 10) = joint_list(i,1).lin3;
%         data_arr(frame_count, 11) = joint_list(i,1).rot1;
%         data_arr(frame_count, 12) = joint_list(i,1).rot2;
%         data_arr(frame_count, 13) = joint_list(i,1).rot3;
% 
%         robot_base = frameOfData.RigidBodies(2); % Robot base data
%         data_arr(frame_count, 14) = robot_base.x;
%         data_arr(frame_count, 15) = robot_base.y;
%         data_arr(frame_count, 16) = robot_base.z;
%     
%         data_arr(frame_count, 17) = robot_base.qw;
%         data_arr(frame_count, 18) = robot_base.qx;
%         data_arr(frame_count, 19) = robot_base.qy;
%         data_arr(frame_count, 20) = robot_base.qz;
% 
%         pause(0.1)
%     end
    pause(1);
%     fclose(ser);

end

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

filename = "data_files/" + date + "_" + int2str(num_tubes) + "-tubes_" + test_string + ".csv";

writematrix(data_arr, filename);