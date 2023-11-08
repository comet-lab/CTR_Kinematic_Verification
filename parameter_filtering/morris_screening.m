%%% Morris Screening %%%

function [elementary_effects_avg, elementary_effects_stdev] = morris_screening(joint_config)

close all;
addpath('../kinematics_and_control')

%% Define constants

% The number of parameters to test
% In this case, it's the number of tubes
p = 3;

% J is a p+1 x p matrix of ones
J = ones([p+1, p]);

% Per Uncertainty Quantification textbook (p. 333), choose l to be even and
% the step size to be l/(2(l-1))
l = 40;
step_size = l/(2*(l-1));

% B is a lower triangluar matrix of ones (size p+1 x p)
B = tril(J,-1);

% Create the robot object, with placeholder bending radii for now
tube1 = Tube(2.792*10^-3, 3.3*10^-3, 50, 90*10^-3, 50*10^-3, 1935*10^6);
tube2 = Tube(2.132*10^-3, 2.64*10^-3, 50, 170*10^-3, 50*10^-3, 1935*10^6);
tube3 = Tube(1.726*10^-3, 1.98*10^-3, 50, 250*10^-3, 50*10^-3, 1935*10^6);
tubes = [tube1 tube2 tube3];
robot = Robot(tubes, false);

% How many trials to run for each joint configuration? 
num_elementary_effects = 1000;

elementary_effects_all = zeros([num_elementary_effects, p, 3]);

% % Define a joint space configuration for the robot to go to
% joint_config = [0 0 0 0 0 0];

for trial=1:num_elementary_effects

    %% Define B* matrix, which determines the configurations of variables to test

    % q_star is a diagonal matrix describing the initial configuration of the
    % parameters
    q_star = diag(randi([0 9], [1 3]) * 1/9);
    
    % D_star is a diagonal matrix with random elements in the set {-1,1}
    D_star = diag(randi([0 1], [1 3])*2-1);
    
    % P_star is chosen by randomly permuting the columns of a pxp identity
    % matrix
    order = randperm(p);
    P_star = zeros(p);
    for i=1:p
        P_star(order(i), i) = 1;
    end
    
    % From p. 334 in textbook:
    B_star = (J * q_star + step_size/2 * ((2*B - J)*D_star + J)) * P_star;
    % Ensure all values are between 0 and 1
    B_star = mod(B_star,1);
    % Scale B* to the desired curvature values [0 1] -> [5 50]
    B_star = B_star * 45 + 5;

    %% Perform forward kinematics for each of the configurations in B*

    % The results matrix will contain one row for each row in B. It has 3
    % columns, one for each of x, y, and z.
    results = zeros([p+1 3]);
    
    for i=1:p+1
    
        % Set the robot's curvatures using the scaled B*
        for j=1:3
            robot.tubes(j).k = B_star(i,j);
        end
    
        % Solve the forward kinematics
        T = robot.fkin(joint_config);
        xyz = T(1:3,4);
    
        results(i,:) = xyz';
    
    end

    %% Calculate the elementary effect of each curvature

    % Elementary effects matrix: one row for each of the p curvatures, one
    % column for each of xyz
    elementary_effects = zeros(p,3);
    
    for i=1:p
    
        % Which variable did we change in this step?
        which_var_changed = order(i);
    
        % Take the difference between the results before and after taking the
        % step
        if B_star(i+1,which_var_changed) > B_star(i,which_var_changed)
            difference = results(i+1,:) - results(i,:);
        else
            difference = results(i,:) - results(i+1,:);
        end
        
        % Divide by the step size (p. 331 of textbook)
        difference_scaled = difference ./ step_size;
    
        % Store the results in the elementary effects matrix
        elementary_effects(which_var_changed,:) = difference_scaled;
    
    end

    % Store the elementary effects in the massive results matrix
    elementary_effects_all(trial, :, :) = elementary_effects;

end

% Compute the average and standard deviation for each elementary effect

% u* (mean) requires the absolute value of the difference
elementary_effects_avg = squeeze(mean(abs(elementary_effects_all), 1));

% For variance, we calculate mean without taking absolute value
elementary_effects_avg_no_abs = squeeze(mean(elementary_effects_all, 1));

% For variance, calculate the distance between each difference and the mean
variance_sum = zeros([p 3]);
for i=1:num_elementary_effects
    variance_sum = variance_sum + (squeeze(elementary_effects_all(i, :, :)) - elementary_effects_avg_no_abs) .^ 2;
end

elementary_effects_variance = 1/(num_elementary_effects-1) * variance_sum;
elementary_effects_stdev = sqrt(elementary_effects_variance);


% disp("Average elementary effect after " + num_elementary_effects + " trials [px3]: ");
% disp(elementary_effects_avg);
% 
% disp("Elementary effect stdev after " + num_elementary_effects + " trials [px3]: ");
% disp(elementary_effects_stdev);

end




