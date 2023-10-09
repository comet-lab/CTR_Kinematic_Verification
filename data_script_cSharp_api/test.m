clear; clc;

startPose = Pose(0, 0, 0, 0, 0, 0);
drive_bot = Drive(startPose);

%DEMO

% generator = Jointspace_Generator();
% positions_list = generator.get_new_random_positions(20);
% 
% for i = 1:size(positions_list,1)
%     drive_bot.travel_to(positions_list(i).lin1, positions_list(i).lin2, positions_list(i).lin3, positions_list(i).rot1, positions_list(i).rot2, positions_list(i).rot3)
% end
