clear;
% mocap vs FKin

% Initialize test information and flags
num_tubes = 2;      % number of tubes for test

% tube parameter: (id, od, curvature, straight section length, arc length, young's modulus)


tube1 = Tube(2.792*10^-3, 3.3*10^-3, 1/13.64, 90*10^-3, 50*10^-3, 1935*10^6);
tube2 = Tube(2.132*10^-3, 2.64*10^-3, 1/12.73, 170*10^-3, 50*10^-3, 1935*10^6);
tube3 = Tube(1.472*10^-3, 1.98*10^-3, 1/16.65, 250*10^-3, 50*10^-3, 193*10^6);

test_points = 16;   % number of test points
fpp = 20;           % number of frames per point

if num_tubes == 2
    tubes = [tube1, tube2];
else
    tubes = [tube1, tube2, tube3];
end
psi_prev = [0,0];

robot = Robot(tubes, false);    % robot object (to call forward kinematics of robot)

% choose default method for fkin
% 0 - Piecewise Constant Curvature
% 1 - Analytical Solution
% 2 - Energy Minimisation
m_flag = 0;

% flag to plot psi vs theta
pt_flag = false;



% enter filename / filepath if different folder
% filename = 'data_files/18_04_23-16_13_2-tubes_rotate';
% filename = 'D:/FichStuff/CTR_Fall23/2023-10-6_05-47-00.csv';
% filename = 'D:/FichStuff/CTR_Fall23/2023-10-7_03-06-44.csv';
% filename = 'D:/FichStuff/CTR_Fall23/2023-10-9_02-10-58.csv';
% filename = 'D:/FichStuff/CTR_Fall23/CTR_Kinematic_Verification/data_script_cSharp_api/data_files/output.csv';
% filename = 'D:/FichStuff/CTR_Fall23/2023-10-19_04-18-50.csv';
% filename = 'D:/FichStuff/CTR_Fall23/2023-10-18_02-46-29.csv';
% filename = 'D:/FichStuff/CTR_Fall23/TestData/2023-10-9_02-10-58.csv';
% filename = 'D:/FichStuff/CTR_Fall23/TestData/2023-10-6_05-47-00.csv';
% filename = 'D:/FichStuff/CTR_Fall23/2023-11-20_01-51-26.csv';

filename = 'D:/FichStuff/CTR_Fall23/2023-11-21_01-41-07.csv';

% filename = 'D:/FichStuff/CTR_Fall23/2023-11-20_03-33-12.csv';

T1 = readtable(filename, 'Format','auto');

if (num_tubes == 3)
    % data frame is lin1, lin2, lin3, rot1, rot2, rot3 and sampling every fpp'th point
    q_tubes = table2array([T1(1:fpp:fpp*test_points,2),T1(1:fpp:fpp*test_points,3),T1(1:fpp:fpp*test_points,4), ...
                           T1(1:fpp:test_points*fpp,5),T1(1:fpp:test_points*fpp,6),T1(1:fpp:fpp*test_points,7)]);       % joint variable
else
    % data frame is lin1, lin2, rot1, rot2 and sampling every fpp'th point
    q_tubes = table2array([T1(1:fpp:fpp*test_points,2),T1(1:fpp:fpp*test_points,3), ...
                           T1(1:fpp:test_points*fpp,5),T1(1:fpp:test_points*fpp,6)]);       % joint variable
end


% ee -> end effector / tip
% % data frame is x, y, z, qw, qx, qy, qz
ee_mocap_data = table2array([T1(1:test_points*fpp,9),T1(1:test_points*fpp,10),T1(1:test_points*fpp,11), ...
                    T1(1:test_points*fpp,12), T1(1:test_points*fpp,13),T1(1:test_points*fpp,14), T1(1:test_points*fpp,15)]);



% hf -> home frame
% % data frame is x, y, z, qw, qx, qy, qz
hf_mocap_data = table2array([T1(1:test_points*fpp,18),T1(1:test_points*fpp,19),T1(1:test_points*fpp,20), ...
                    T1(1:test_points*fpp,21), T1(1:test_points*fpp,22),T1(1:test_points*fpp,23), T1(1:test_points*fpp,24)]);


for i = 1:test_points
    
    % Averaging the data across the number of frames per point
    
    % ROBOT TIP / END EFFECTOR
    % averging all the data (x, y, z, qw, qx, qy, qz)
    ee_mocap_data_avg(i,:) = mean(ee_mocap_data(((i-1)*fpp)+1:i*fpp,:));
    
    % obtaining the position data from the combined data frame
    ee_mocap_pos(i,:) = ee_mocap_data_avg(i,1:3)*10^-3; 

    % the average data has average quaternions that may not be normalised
    ee_mocap_quat(i,:) = normalize(ee_mocap_data_avg(i,4:7), "norm");
    
    % converting quaternions to euler angles
    ee_mocap_eul(i,:) = quat2eul(ee_mocap_quat(i,:));

    % converting quaternions to rotation matrix
    ee_mocap_rotm(:,:,i) = quat2rotm(ee_mocap_quat(i,:));

    % transformtion of robot tip from mocap origin
    ee_mocap_tf(:,:,i) = [[ee_mocap_rotm(:,:,i),ee_mocap_pos(i,:)'];[0,0,0,1]];




    % Averaging the data across the number of frames per point
    
    % ROBOT TIP / END EFFECTOR
    % averging all the data (x, y, z, qw, qx, qy, qz)
    hf_mocap_data_avg(i,:) = mean(hf_mocap_data(((i-1)*fpp)+1:i*fpp,:));
    
    % obtaining the position data from the combined data frame
    hf_mocap_pos(i,:) = hf_mocap_data_avg(i,1:3)*10^-3; 
    
    % the average data has average quaternions that may not be normalised
    hf_mocap_quat(i,:) = normalize(hf_mocap_data_avg(i,4:7), "norm");
    
    % converting quaternions to euler angles
    hf_mocap_eul(i,:) = quat2eul(hf_mocap_quat(i,:));
    
    % converting quaternions to rotation matrix
    hf_mocap_rotm(:,:,i) = quat2rotm(hf_mocap_quat(i,:));
    
    % transformtion of home frame from mocap origin
    hf_mocap_tf(:,:,i) = [[hf_mocap_rotm(:,:,i),hf_mocap_pos(i,:)'];[0,0,0,1]];


    % computing the transform of robot tip w.r.t. home frame
    ee_hf_tf(:,:,i) = inv(hf_mocap_tf(:,:,i))*ee_mocap_tf(:,:,i);

    % obtaining the position data from the above transform
    ee_hf_pos(i,:) = ee_hf_tf(1:3,4,i)';

    % obtaining the rotation matrix from the above transform
    ee_hf_rotm(:,:,i) = ee_hf_tf(1:3,1:3,i);
    
    % converting the rotation matrix to quaternions
    ee_hf_quat(i,:) = rotm2quat(ee_hf_tf(1:3,1:3,i));

    % converting quaternions to euler angles
    ee_hf_eul(i,:) = quat2eul(ee_hf_quat(i,:));


    %%% Using the Forawrd kinematics model
    % Data format for the transforms from the model
    % TT is a 4x4xjxn
    % 4x 4           -> tranformation matrix
    % x j            -> j transformations for each joint pose (j = 2*no_of_tubes - 1)
    % x n            -> iterating trhough the test points
   
    


        if m_flag == 0
            TT(:,:,i) = robot.fkin(q_tubes(i,:));
        
        elseif m_flag == 1
            [TT(:,:,i), psi(:,i)] = robot.fkin_tors_as(q_tubes(i,:), psi_prev);
            psi_prev = psi(:,i);
            theta_prev = deg2rad(q_tubes(i, 3:4));
            psi_i = psi(:,i);
            psi_deg = (psi')*(180/3.14159);
            jac(:,:,i) = robot.calc_complete_jacobian(q_tubes(i,:), psi_i);
%             inv_err(i,:) = robot.check_inverseK(ee_hf_pos(i,:)', q_tubes(i,:), psi_i);
        
        elseif m_flag == 2
            [TT(:,:,i), psi(:,i)] = robot.fkin_tors_em(q_tubes(i,:), psi_prev);
            psi_prev = psi(:,i);
            psi_deg = (psi')*(180/3.14159);
        
        end

    
    theta_deg = q_tubes(:, 3:4);
    % Robot_home in the forward kinematics model
    % is attached to the start of the curved section of 
    % the outer most tube

    % translation of the robot home because outer tube can move
    f1_f0 = [[eye(3),[0,0,q_tubes(i,1)*10^-3]'];[0,0,0,1]];

    % computing the tranform from robot home to robot tip using
    % the forward kinematics model by
    % multiplying all the transforms of linklengths
    % (4 x 4 x n)
    ee_fk_tf(:,:,i) = f1_f0*TT(:,:,i);

    % contains x,y,z for the end-effector
    ee_fk_pos(i,:) = [ee_fk_tf(1,4,i); ee_fk_tf(2,4,i); ee_fk_tf(3,4,i)];
    
    % obtaining the rotation matrix from the above transform
    ee_fk_rotm(:,:,i) = ee_fk_tf(1:3,1:3,i);
    
    % converting the rotation matrix to quaternions
    ee_fk_quat(i,:) = rotm2quat(ee_fk_tf(1:3,1:3,i));
    
    % converting quaternions to euler angles
    ee_fk_eul(i,:) = quat2eul(ee_fk_quat(i,:));


    % norm error for each position of robot tip
    % between the model and the mocap
    pos_error_norm(i,:) = norm(ee_fk_pos(i,:) - ee_hf_pos(i,:));
    
    % x,y,z error for each position of robot tip
    % between the model and the mocap
    pos_error(i,:) = (ee_fk_pos(i,:) - ee_hf_pos(i,:))*10^3;


end

% RMSE for individual coordinates of robot tip
pos_error_t = rmse(ee_fk_pos, ee_hf_pos);
disp("RMSE across coordinates: ")
disp(pos_error_t);

% RMSE of the position of robot tip
rmse_t = sqrt(sum(power(pos_error_norm, 2))*(1/test_points));
disp("RMSE: ")
disp(rmse_t);


%%% PLOTTING POSTIONS AND ERROR

range = 1:test_points;
ee_fk_pos_mm = ee_fk_pos*10^3;
ee_hf_pos_mm = ee_hf_pos*10^3;


figure(1)

subplot(3,1,1)
plot(range, [ee_fk_pos_mm(:,1), ee_hf_pos_mm(:,1)]);
grid on;
set(gca,'FontSize',16,'fontWeight','bold')
set(findall(gcf,'type','text'),'FontSize',16,'fontWeight','bold')
ylim([-25, 75]);
xlabel("Test Points")
ylabel("Position [mm]");
title("X Position");
if m_flag == 0
    legend("FK w/o torsion", "Mocap");
elseif m_flag ==1
    legend("FK w torsion (AS)", "Mocap");
else
    legend("FK w torsion (EM)", "Mocap");
end

subplot(3,1,2)
plot(range, [ee_fk_pos_mm(:,2), ee_hf_pos_mm(:,2)]);
grid on;
set(gca,'FontSize',16,'fontWeight','bold')
set(findall(gcf,'type','text'),'FontSize',16,'fontWeight','bold')
ylim([-50, 50]);
xlabel("Test Points")
ylabel("Position [mm]");
title("Y Position");
if m_flag == 0
    legend("FK w/o torsion", "Mocap");
elseif m_flag ==1
    legend("FK w torsion (AS)", "Mocap");
else
    legend("FK w torsion (EM)", "Mocap");
end

subplot(3,1,3)
plot(range, [ee_fk_pos_mm(:,3), ee_hf_pos_mm(:,3)]);
grid on;
set(gca,'FontSize',16,'fontWeight','bold')
set(findall(gcf,'type','text'),'FontSize',16,'fontWeight','bold')
ylim([25, 125]);
xlabel("Test Points")
ylabel("Position [mm]");
title("Z Position");
if m_flag == 0
    legend("FK w/o torsion", "Mocap");
elseif m_flag ==1
    legend("FK w torsion (AS)", "Mocap");
else
    legend("FK w torsion (EM)", "Mocap");
end

sgtitle("XYZ Positions")


figure(2)
boxplot(pos_error_norm*10^3);
grid on;
set(gca,'FontSize',16,'fontWeight','bold')
set(findall(gcf,'type','text'),'FontSize',16,'fontWeight','bold')
xlabel("Error in tip position")
ylabel("[mm]");
title("Error box plot (" + test_points +"datapoints)");


if (m_flag > 0 && pt_flag == 1)
    figure(3)
    subplot(2,1,1)
    plot(range, [psi_deg(:,1), theta_deg(:,1)]);
    grid on;
    set(gca,'FontSize',16,'fontWeight','bold')
    set(findall(gcf,'type','text'),'FontSize',16,'fontWeight','bold')
    xlabel("Test Points");
    ylabel("[deg]");
    title("Psi 1 vs Theta 1")
    legend("Psi", "Theta");
    
    subplot(2,1,2)
    plot(range, [psi_deg(:,2), theta_deg(:,2)]);
    grid on;
    set(gca,'FontSize',16,'fontWeight','bold')
    set(findall(gcf,'type','text'),'FontSize',16,'fontWeight','bold')
    xlabel("Test Points");
    ylabel("[deg]");
    title("Psi 2 vs Theta 2")
    legend("Psi", "Theta");

    sgtitle("PSI vs THETA")
end