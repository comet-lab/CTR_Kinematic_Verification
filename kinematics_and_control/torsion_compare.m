clc; clear; 

% Define our tubes (ID, OD, r (1/K), l, d, E)
tube1 = Tube(2.792*10^-3, 3.3*10^-3, 1/17, 90*10^-3, 50*10^-3, 400*10^6);
tube2 = Tube(2.132*10^-3, 2.64*10^-3, 1/22, 170*10^-3, 50*10^-3, 400*10^6);
tube3 = Tube(1.472*10^-3, 1.98*10^-3, 1/29, 250*10^-3, 50*10^-3, 400*10^6);

% define our array of tubes
tubes = [tube1, tube2];

% create a robot
robot = Robot(tubes, false);

% set the increment [degrees], end point [degrees], and calculate the number of
% points in the set
increment = 1;
end_point = 360;
num_points = floor(end_point / increment) + 1;

% declare array of rotation values for joint 2 and an empty q_var array
rotations = end_point:-increment:0;
q_var = zeros(num_points,4);

% set the rotation for tube 2 to the rotation values
for j = 1:num_points
    q_var(j,2) = 30;
    q_var(j,4) = rotations(j);
end


% Do the kinematics. 
T_pcc = {};             % piecewise constannt curvature transformations
T_em = {};              % energy minimazation transformations
T_as = {};              % analytical solution transformations
psi_prev_em = [0,0];
psi_prev_as = [0,0];
psi_em = zeros(num_points, 2);
psi_as = zeros(num_points, 2);
for i = 1:num_points
    % calculate pcc kinematics
    t_pcc = robot.fkin(q_var(i,:));
    T_pcc{i} = t_pcc;

    % calculate energy minimization kinematics
    [t_em, psi_prev_em] = robot.fkin_tors_em(q_var(i,:), psi_prev_em);
    psi_em(i,:) = psi_prev_em;
    T_em{i} = t_em;

    % calculate analytical solution kinematics
    [t_as, psi_prev_as] = robot.fkin_tors_as(q_var(i,:),psi_prev_as);
    psi_as(i,:) = psi_prev_as;
    T_as{i} = t_as;

    disp("Point " + i);
end

save("torsion_compare.mat", "T_pcc", "T_em", "T_as", "rotations", "psi_em", "psi_as")