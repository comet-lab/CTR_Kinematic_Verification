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
addpath('../kinematics_and_control')

%% Setup

% Create two robots: one to represent the "real" material parameters and
% one to represent the "modeled" material parameters. Each robot will have
% two tubes.
robot_real = Robot(2, false);
robot_guess = Robot(2, false);

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
q_target = [40 10 30 80];

% The robot starts at these curvatures
k_real = [6 7];

% The "real" robot's tubes gradually approach these curvatures
k_guess = [10 12];

% We need an initial guess for covariance in the Kalman filter.
covariance_guess = [1 0 0 0.1 0.1;
                    0 1 0 0.1 0.1;
                    0 0 1 0.1 0.1;
                    0 0 0 1 0.1;
                    0 0 0 0.1 1;];

% Initialize matrices to hold all the values of k over time
k_all_real = zeros([2, num_steps]);
k_all_guess = zeros([2, num_steps]);

% Initialize matrices to hold the error values and the xyz values of both
% robots
error = zeros([1,num_steps]);
all_xyz_real = zeros([3, num_steps]);
all_xyz_guess = zeros([3, num_steps]);

%% Simulate the robots.

% Simulate 30 seconds of robot motion.
% During each of three 10-second periods, the robot will spend the first 5
% seconds moving from the home configuration (q0) to the target
% configuration (q_target) and the remaining 5 seconds moving back to the
% home configuration
for i=1:num_steps

    % Calculate the input joint variables for this step
    q = get_q(t(i), q_target);

    % Calculate the forward kinematics of the real robot, with the correct
    % curvature values.

    % Apply the calculated precurvatures to the real robot
    robot_real.tube1.k = k_real(1);
    robot_real.tube2.k = k_real(2);

    % Add these k values to the storage matrix
    k_all_real(:, i) = k_real';
    
    % Calculate the forward kinematics for the real robot at this step
    T_real = robot_real.fkin(q);

    % Multiply the transformation matrices together to get the overall
    % transformation matrix
    T_real = T_real(:,:,1) * T_real(:,:,2) * T_real(:,:,3);

    % Extract the tip position from the transformation matrix
    xyz_real = T_real(1:3,4);
    all_xyz_real(:,i) = xyz_real;

    % Calculate the forward kinematics of the "guess" robot, with the 
    % filtered curvature values.

    % The precurvature of the modeled robot remains constant
    robot_guess.tube1.k = k_guess(1);
    robot_guess.tube2.k = k_guess(2);

    % Add the k values to the storage matrix
    k_all_guess(:, i) = k_guess';
    
    % Calculate the forward kinematics for the robot at this step
    T_modeled = robot_guess.fkin(q);

    % Multiply the transformation matrices together to get the overall
    % transformation matrix
    T_modeled = T_modeled(:,:,1) * T_modeled(:,:,2) * T_modeled(:,:,3);

    % Extract the tip position from the transformation matrix
    xyz_modeled = T_modeled(1:3,4);
    all_xyz_guess(:,i) = xyz_modeled;   

    % Update k_guess using the kalman filter.

    % % Based on our actuation and our (bad) precurvature values, where do we
    % % think we are? 
    % prediction = [xyz_modeled; k_guess'];
    % % We observe the tip position of the real robot
    % observation = xyz_real;
    % 
    % % Run the filter
    % [state_correction, covariance_guess] = kalman_filter(covariance_guess, prediction, observation);
    % % Extract the new k_guess from the corrected state values.
    % k_guess(1) = state_correction(4);
    % k_guess(2) = state_correction(5);

end

disp("Done with robot sim. Plotting errors now")

%% Calculate and plot the errors.

% Calculate the error between the two trajectories
for i=1:num_steps
    error(i) = norm(all_xyz_guess(:,i) - all_xyz_real(:,i));
end

% Convert [m] to [mm]
error_scaled = error*1000;

% Convert curvature to bending radii
k_all_guess = 1 ./ k_all_guess;
k_all_real = 1 ./ k_all_real;

% Plot the error over time
figure
plot(t,error_scaled);
xlabel("Time [sec]", "FontSize",16);
ylabel("Tip Position Error [mm]", "FontSize",16);
ylim([0 10]);
title(["Tip Position Error vs Time for Simulated" "Kinematics with No Filtering"], "FontSize",20);

% Plot the k values over time
figure
hold on
plot(t,k_all_guess(1,:), "Color","b", "LineStyle","-");
plot(t,k_all_guess(2,:), "Color","r", "LineStyle","-");
plot(t,k_all_real(1,:), "Color","b", "LineStyle","--");
plot(t,k_all_real(2,:), "Color","r", "LineStyle","--");
xlabel("Time [sec]", "FontSize",16);
ylim([.05 .2]);
ylabel("Tube Radii [m]", "FontSize",16);
title(["CTR Tube Bending Radii vs Time for" "Both Correct and Incorrect Robots"], "FontSize", 20);
legend(["Incorrect Robot Tube 1", "Incorrect Robot Tube 2", "Correct Robot Tube 1", "Correct Robot Tube 2"], "FontSize",14);


%% Helper Function to Track Values of q (Joint Variables)

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