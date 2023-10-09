% This script simulates motion of a pair of 2-tube concentric tube robots.
% One robot has tubes that never plastically deform, while the other
% robot's tubes plastically deform according to the piecewise function
% defined by k_initial, k_final, and get_k, where k represents the
% precurvature of the tubes. Both robots travel from their home
% configuration to a target set of joint variables and back three times.
% Travel from home to target takes 5 seconds, and travel from target to
% home takes 5 seconds. The robots are identical during the first 10
% seconds of travel. The second robot's curvature begins to change at t=10
% seconds and stops changing at t=20 seconds, at the end of the second
% travel loop. During the final 10 seconds, the two robots' tubes have 
% different curvatures, but the curvatures do not change.

clear all; close all; clc;

%% Setup

% Create two robots: one to represent the "real" material parameters and
% one to represent the "modeled" material parameters. Each robot will have
% two tubes.
robot_real = Robot(2, false);
robot_modeled = Robot(2, false);

% Generate a time series
t = 0:0.01:30;
num_steps = size(t);
num_steps = num_steps(2);

% The robot will start at the home configuration
q0 = [0 0 0 0];

% A test point (in joint space) to which we will repeatedly move the CTR
% Takes the form [rho1 rho2 theta1 theta2], where rhoi is the translation
% applied to tube i in [mm] and thetai is the rotation applied to the base
% of tube i in [deg]
q_target = [30 50 60 -5];

% The robot starts at these curvatures
k_initial = [11 14];

% The "real" robot's tubes gradually approach these curvatures
k_final = [6 7];

% Initialize matrices to hold all the values of k over time
k_all_real = zeros([2, num_steps]);
k_all_modeled = zeros([2, num_steps]);

% Initialize matrices to hold the error values and the xyz values of both
% robots
error = zeros([1,num_steps]);
all_xyz_real = zeros([3, num_steps]);
all_xyz_modeled = zeros([3, num_steps]);

%% Simulate the "real" robot, with tubes with dynamic precurvatures.

% Simulate 30 seconds of robot motion.
% During each of three 10-second periods, the robot will spend the first 5
% seconds moving from the home configuration (q0) to the target
% configuration (q_target) and the remaining 5 seconds moving back to the
% home configuration
for i=1:num_steps

    % Calculate the input joint variables for this step
    q = get_q(t(i), q_target);

    % Calculate the precurvatures to apply to the real robot at this time
    % step
    k = get_k(t(i), k_initial, k_final);

    % Apply the calculated precurvatures to the real robot
    robot_real.tube1.k = k(1);
    robot_real.tube2.k = k(2);

    % Add these k values to the storage matrix
    k_all_real(:, i) = k';

    % if t(i)==5
    %     robot_real.plotOn = true;
    % else
    %     robot_real.plotOn = false;
    % end
    
    % Calculate the forward kinematics for the robot at this step
    T_real = robot_real.fkin(q);

    % Multiply the transformation matrices together to get the overall
    % transformation matrix
    T_real = T_real(:,:,1) * T_real(:,:,2) * T_real(:,:,3);

    % Extract the tip position from the transformation matrix
    xyz_real = T_real(1:3,4);
    all_xyz_real(:,i) = xyz_real;

end

disp("done with robot 1")

%% Simulate the "modeled" robot, with static tube curvatures.

for i=1:num_steps

    % Calculate the input joint variables for this step
    q = get_q(t(i), q_target);

    % The precurvature of the modeled robot remains constant
    robot_modeled.tube1.k = k_initial(1);
    robot_modeled.tube2.k = k_initial(2);

    % Add the k values to the storage matrix
    k_all_modeled(:, i) = k_initial';
    
    % Calculate the forward kinematics for the robot at this step
    T_modeled = robot_modeled.fkin(q);

    % Multiply the transformation matrices together to get the overall
    % transformation matrix
    T_modeled = T_modeled(:,:,1) * T_modeled(:,:,2) * T_modeled(:,:,3);

    % Extract the tip position from the transformation matrix
    xyz_modeled = T_modeled(1:3,4);
    all_xyz_modeled(:,i) = xyz_modeled;

end

disp("done with robot 2")

%% Calculate and plot the errors.

% Calculate the error between the two trajectories
for i=1:num_steps
    error(i) = norm(all_xyz_modeled(:,i) - all_xyz_real(:,i));
end

% Convert [m] to [mm]
error_scaled = error*1000;

% Plot the error over time
figure
plot(t,error_scaled);
xlabel("Time [sec]");
ylabel("Tip Position Error [mm]");
title("Tip Position Error vs Time for Simulated Kinematics with No Curvature Tracking");

% Plot the k values over time
figure
hold on
plot(t,k_all_modeled(1,:));
plot(t,k_all_modeled(2,:));
plot(t,k_all_real(1,:));
plot(t,k_all_real(2,:));
xlabel("Time[sec]");
ylabel("Tube Curvatures [1/m]");
title("CTR Tube Curvatures vs Time for Both Real and Modeled Robots");
legend(["Modeled Robot Tube 1", "Modeled Robot Tube 2", "Real Robot Tube 1", "Real Robot Tube 2"]);


%% Helper Functions to Track Values of q (Joint Variables) and k (Curvatures)


% Define a piecewise function to determine how the "real" robot's tubes
% change curvature over time
function q = get_q(t, q_target)

    t_mod = mod(t, 10);

    if t_mod <= 5
        q = q_target * t_mod/5;
    else 
        q = q_target * (1 - (t_mod-5)/5);
    end
end

% Define a piecewise function to determine how the "real" robot's tubes
% change curvature over time
function k = get_k(t, k_initial, k_final)

    if t <= 10
        k = k_initial;
    elseif t <= 20
        k = k_initial + (k_final - k_initial) * ((t - 10)/10);
    elseif t <= 30
        k = k_final;
    end
end