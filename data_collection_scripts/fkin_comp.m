% mocap vs FKin

% Initialize test information and flags
num_tubes = 2;      % number of tubes for test
test_points = 21;   % number of test points
rotation = true;   % true if test includes rotation
plotOn = true;     % set true to show plots in the end

fpp = 10;           % number of frames per point

robot = Robot(num_tubes, plotOn);    % robot object (to call forward kinematics of robot)

% translation offsets for ground plane
x_offset = 0.00;
% y_offset = -0.00725;
y_offset = -0.00725+0.00268;
% z_offset = 0.110;
z_offset = 0.1127;


% translation offsets for end effector

x_ee = 0.0;
y_ee = 0.0;
z_ee = 0.005;




% enter filename / filepath if different folder
filename = 'data_files/02_05_23-18_05_2-tubes_in-plane-bending';
% filename = 'data_files/18_04_23-15_27_2-tubes_in-plane-bending';  
% filename = 'data_files/14_04_23-16_51_2-tubes_in-plane-bending';

T1 = readtable(filename);

% data frame is lin1, lin2, rot1, rot2 and sampling every fpp'th point
q_2tubes = table2array([T1(1:fpp:fpp*test_points,8),T1(1:fpp:fpp*test_points,9), ...
                        T1(1:fpp:test_points*fpp,11),T1(1:fpp:test_points*fpp,12)]);       % joint variable

% ee -> end effector / tip

% % data frame is x, y, z, qw, qx, qy, qz
ee_mocap_data = table2array([T1(1:test_points*fpp,1),T1(1:test_points*fpp,2),T1(1:test_points*fpp,3), ...
                    T1(1:test_points*fpp,4), T1(1:test_points*fpp,5),T1(1:test_points*fpp,6), T1(1:test_points*fpp,7)]);

% data frame is x, y, z, qw, qx, qy, qz
% ee_mocap_data = table2array([T1(1:test_points*fpp,14),T1(1:test_points*fpp,15),T1(1:test_points*fpp,16), ...
%                     T1(1:test_points*fpp,17), T1(1:test_points*fpp,18),T1(1:test_points*fpp,19), T1(1:test_points*fpp,20)]);



for i = 1:test_points   
    ee_mocap_data_avg(i,:) = mean(ee_mocap_data(((i-1)*fpp)+1:i*fpp,:));
    % the average data has average quaternions that may not be normalised

    ee_mocap_pos(i,:) = ee_mocap_data_avg(i,1:3); 
    ee_mocap_quat(i,:) = normalize(ee_mocap_data_avg(i,4:7), "norm");
    ee_mocap_eul(i,:) = quat2eul(ee_mocap_quat(i,:));
    ee_mocap_rotm(:,:,i) = quat2rotm(ee_mocap_quat(i,:));
    
    % transformtion of robot tip from mocap origin
    ee_mocap_tf(:,:,i) = [[ee_mocap_rotm(:,:,i),ee_mocap_pos(i,:)'];[0,0,0,1]];

    mocap_fk_rot = [[rotx(-90)*rotz(180),[0;0;0]];[0,0,0,1]];
    mocap_fk_trl = [[eye(3),[x_offset;y_offset;z_offset]];[0,0,0,1]];

    mocap_fk_tf = mocap_fk_trl*mocap_fk_rot;
    ee_att_tf = [[rotx(-90)*rotz(180),[x_ee;y_ee;z_ee]];[0,0,0,1]];


    ee_rb_tf(:,:,i) = inv(mocap_fk_tf)*ee_mocap_tf(:,:,i)*inv(ee_att_tf);

    ee_rb_pos(i,:) = ee_rb_tf(1:3,4,i)';
    ee_rb_quat(i,:) = rotm2quat(ee_rb_tf(1:3,1:3,i));
    ee_rb_rotm(:,:,i) = ee_rb_tf(1:3,1:3,i);
    ee_rb_eul(i,:) = quat2eul(ee_rb_quat(i,:));


    % TT is a 4x4x3xn
    % 4x 4           -> tranformation matrix
    % x 3            -> three transformations for each joint pose
    % x n            -> iterating trhough the test points
%     TT(:,:,:,i) = robot.fkin(q_2tubes(i,:));
    TT(:,:,:,i) = robot.fkin(q_2tubes(i,:));

%     theta(:,i) = robot.Theta;
%     psi(:,i) = robot.Psi;

    
    f1_f0 = [[eye(3),[0,0,q_2tubes(i,1)*10^-3]'];[0,0,0,1]];

    % contains the tranformations for the end-effector for all testpoints
    % (4 x 4 x n)
    ee_fk_tf(:,:,i) = f1_f0*TT(:,:,1,i)*TT(:,:,2,i)*TT(:,:,3,i);

    % contains x,y,z for the end-effector for all testpoints 
    % (n x 3)
    ee_fk_pos(i,:) = [ee_fk_tf(1,4,i); ee_fk_tf(2,4,i); ee_fk_tf(3,4,i)];
    ee_fk_quat(i,:) = rotm2quat(ee_fk_tf(1:3,1:3,i));
    ee_fk_rotm(:,:,i) = ee_fk_tf(1:3,1:3,i);
    ee_fk_eul(i,:) = quat2eul(ee_fk_quat(i,:));

    pos_error(i,:) = norm(ee_fk_pos(i,:)) - norm(ee_rb_pos(i,:));
    pos_c_error(i,:) = (ee_fk_pos(i,:) - ee_rb_pos(i,:))*10^3;

end

% pos_error_t = rmse(ee_fk_pos, ee_mocap_pos);
pos_error_t = rmse(ee_fk_pos, ee_rb_pos);
