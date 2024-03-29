function figures = extended_kalman_filter(k_real, k_guess, q0, q_target, simTime, dt, processNoise, measureNoise, jointVarNoise, initialCovariance)

close all;
addpath('../kinematics_and_control')

%% Tube Curvatures

% The robot starts at these curvatures
% k_real = [6 7];

% We guess that the robot has these curvatures
% k_guess = [10 12];

%% Joint Space Target

% The robot will start at the home configuration
% q0 = [0 0 0 0];

% A test point (in joint space) to which we will repeatedly move the CTR
% Takes the form [rho1 rho2 theta1 theta2], where rhoi is the translation
% applied to tube i in [mm] and thetai is the rotation applied to the base
% of tube i in [deg]
% q_target = [40 10 30 80];

%% Robot Setup

k_size = size(k_real);
num_tubes = k_size(2);

tube1 = Tube(2.792*10^-3, 3.3*10^-3, 1/k_real(1), 90*10^-3, 50*10^-3, 1935*10^6);
tube2 = Tube(2.132*10^-3, 2.64*10^-3, 1/k_real(2), 170*10^-3, 50*10^-3, 1935*10^6);
if num_tubes == 3
    tube3 = Tube(1.726*10^-3, 1.98*10^-3, 1/k_real(3), 250*10^-3, 50*10^-3, 1935*10^6);
    tubes = [tube1 tube2 tube3];
else
    tubes = [tube1 tube2];
end
robot = Robot(tubes, false);



%% Initialize Time Series

% rng(2022);    % For repeatable results
% dt = 0.04;     % seconds
% simTime = 50; % seconds
tspan = 0:dt:simTime;

numStateVars = 3 + num_tubes;

numSteps = length(tspan);
trueStates = zeros([numStateVars,numSteps]);
estimateStates = zeros(size(trueStates));
measurements = zeros([3,numSteps]);
alternativeTipEstimate = zeros([3, numSteps]);
covariances = zeros([numStateVars, numStateVars, numSteps]);

%% Calculate Initial State and Measurement

for i = 1:num_tubes
    robot.tubes(i).k = k_real(i);
end

T_initial = robot.fkin(q0);
xyz_initial = T_initial(1:3,4);

trueInitialState = [xyz_initial; k_real'];
trueInitialMeasurement = xyz_initial;

trueStates(:,1) = trueInitialState;
measurements(:,1) = trueInitialMeasurement;

%% Define Noises for True States

% processNoise = diag([1e-6 1e-6 1e-6 25e-4 25e-4]);
% % processNoise(1:3,1:3) = 1e-6;
% processNoise(4,5) = 25e-4;
% processNoise(5,4) = 25e-4;
% measureNoise = diag([0e-6 0e-6 0e-6]);
% measureNoise = measureNoise + 9e-6;
%
% % jointVarNoise = diag([9, 9, 25, 25]);
% jointVarNoise = [9 9 0 0;
%                  9 9 0 0;
%                  0 0 25 25;
%                  0 0 25 25];

%% Simulating True States and Measurements

for i = 2:numSteps

    t = tspan(i);
    q = get_q(t, q_target, jointVarNoise);

    trueStates(:,i) = stateModel(trueStates(:,i-1),dt); %+ sqrt(processNoise)*randn(3+num_tubes,1);
    measurements(:,i) = measureModel(trueStates(:,i)) + sqrt(measureNoise)*randn(3,1);
end

%% Define Noises for Filter

% processNoise = diag([1e-6 1e-6 1e-6 25e-4 25e-4]);
% % processNoise(1:3,1:3) = 1e-6;
% processNoise(4,5) = 25e-4;
% processNoise(5,4) = 25e-4;
% measureNoise = diag([0e-6 0e-6 0e-6]);
% measureNoise = measureNoise + 9e-6;
%
% % jointVarNoise = diag([9, 9, 25, 25]);
% jointVarNoise = [9 9 0 0;
%                  9 9 0 0;
%                  0 0 25 25;
%                  0 0 25 25];

%% Initial Estimation

for i = 1:num_tubes
    robot.tubes(i).k = k_guess(i);
end

T_initial_guess = robot.fkin(q0);
xyz_initial_guess = T_initial_guess(1:3,4);

initial_state_guess = [xyz_initial_guess; k_guess'];

estimateStates(:,1) = initial_state_guess;

% initialCovariance = diag([10; 10; 10; 10; 10]);

alternativeTipEstimate(:,1) = initial_state_guess(1:3);

covariances(:,:,1) = initialCovariance;

%% Create Filter Object

filter = trackingEKF(State=initial_state_guess,StateCovariance=initialCovariance, ...
    StateTransitionFcn=@stateModel,ProcessNoise=processNoise, ...
    MeasurementFcn=@measureModel,MeasurementNoise=measureNoise);

%% Simulate the Filter

disp("Beginning filter simulation");

for i=2:numSteps

    % Find joint variables at current timestep t
    t = tspan(i);
    q = get_q(t, q_target, jointVarNoise);

    % Run the filter's prediction and correction steps
    predict(filter,dt);
    estimateStates(:,i) = correct(filter,measurements(:,i));

    % Using the filtered curvature, where do we think the robot is?
    T_estimate = robot.fkin(q);
    xyz_estimate = T_estimate(1:3,4);

    alternativeTipEstimate(:,i) = xyz_estimate;

    % Save the covariance for plotting later
    covariances(:,:,i) = filter.StateCovariance;

end

disp("done robot sim");

%% Perform Error Calculations

% buffer = 20;
% rollingAvgXYZ = zeros([3,numSteps]);
% for i=1:buffer-1
%     rollingAvgXYZ(:,i) = mean(estimateStates(1:3,1:i), 2);
% end
% for i=buffer:numSteps
%     rollingAvgXYZ(:,i) = mean(estimateStates(1:3,i-buffer+1:i), 2);
% end
%
% errorXYZRolling = rollingAvgXYZ(1:3,:) - trueStates(1:3,:);
%
% errorEuclidRolling = zeros([1,numSteps]);
% for i=1:numSteps
%     errorEuclidRolling(i) = norm(errorXYZRolling(:,i));
% end
%
% errorEuclidRolling = errorEuclidRolling * 1000;


errorXYZ = estimateStates(1:3,:) - trueStates(1:3,:);
% errorXYZAlternate = alternativeTipEstimate(1:3,:) - trueStates(1:3,:);

errorEuclid = zeros([1,numSteps]);
for i=1:numSteps
    errorEuclid(i) = norm(errorXYZ(:,i));
end

% Convert m to mm
errorEuclid = errorEuclid * 1000;

% errorEuclidAlternate = zeros([1,numSteps]);
% for i=1:numSteps
%     errorEuclidAlternate(i) = norm(errorXYZAlternate(:,i));
% end
%
% % Convert m to mm
% errorEuclidAlternate = errorEuclidAlternate * 1000;

%% Perform Variance Calculations

k1_stdevs = sqrt(covariances(4,4,:));
k2_stdevs = sqrt(covariances(5,5,:));

k1_confidence = squeeze(2 * k1_stdevs)';
k2_confidence = squeeze(2 * k2_stdevs)';

if num_tubes == 3
    k3_stdevs = sqrt(covariances(6,6,:));
    k3_confidence = squeeze(3 * k2_stdevs)';
end

%% Plot the Tip Position Error

f1 = figure;
plot(tspan,errorEuclid);
title("Filtered Tip Position Error");
xlabel("Time [sec]");
ylabel("Tip Position Error [mm]");


% figure
% plot(tspan,errorEuclidAlternate);
% title("Tip Position Error Using Filtered Curvatures");
% xlabel("Time [sec]");
% ylabel("Tip Position Error [mm]");
%
% figure
% plot(tspan,errorEuclidRolling);
% title("Tip Position Error Using Rolling Average");
% xlabel("Time [sec]");
% ylabel("Tip Position Error [mm]");

%% Plot the curvatures

f2 = figure;

% subplot(num_tubes,1,1);
% hold on
% plot(tspan, trueStates(4,:), 'Color','black', 'LineStyle','-');
% plot(tspan, estimateStates(4,:), 'Color','blue', 'LineStyle','-');
% plot(tspan, (estimateStates(4,:)+k1_confidence), 'Color','blue', 'LineStyle','--');
% plot(tspan, (estimateStates(4,:)-k1_confidence), 'Color','blue', 'LineStyle','--');
% title("Filtered Estimate of Tube 1 Curvature Over Time");
% xlabel("Time [sec]");
% ylabel("Curvature [1/m]");
% legend(["True Radius", "Filtered Radius", "95% Confidence Boundary"]);
% % ylim([0 0.3]);
% 
% subplot(num_tubes,1,2);
% hold on
% plot(tspan, trueStates(5,:), 'Color','black', 'LineStyle','-');
% plot(tspan, estimateStates(5,:), 'Color','red', 'LineStyle','-');
% plot(tspan, (estimateStates(5,:)+k2_confidence), 'Color','red', 'LineStyle','--');
% plot(tspan, (estimateStates(5,:)-k2_confidence), 'Color','red', 'LineStyle','--');
% title("Filtered Estimate of Tube 2 Curvature Over Time");
% xlabel("Time [sec]");
% ylabel("Curvature [1/m]");
% legend(["True Radius", "Filtered Radius", "95% Confidence Boundary"]);
% % ylim([0 0.3]);
% 
% if num_tubes == 3
% 
% subplot(num_tubes,1,3);
% hold on
% plot(tspan, trueStates(6,:), 'Color','black', 'LineStyle','-');
% plot(tspan, estimateStates(6,:), 'Color','green', 'LineStyle','-');
% plot(tspan, (estimateStates(6,:)+k3_confidence), 'Color','green', 'LineStyle','--');
% plot(tspan, (estimateStates(6,:)-k3_confidence), 'Color','green', 'LineStyle','--');
% title("Filtered Estimate of Tube 3 Curvature Over Time");
% xlabel("Time [sec]");
% ylabel("Curvature [1/m]");
% legend(["True Radius", "Filtered Radius", "95% Confidence Boundary"]);
% % ylim([0 0.3]);
% 
% end

subplot(num_tubes,1,1);
hold on
plot(tspan, 1 ./ trueStates(4,:), 'Color','black', 'LineStyle','-');
plot(tspan, 1 ./ estimateStates(4,:), 'Color','blue', 'LineStyle','-');
plot(tspan, 1 ./ (estimateStates(4,:)+k1_confidence), 'Color','blue', 'LineStyle','--');
plot(tspan, 1 ./ (estimateStates(4,:)-k1_confidence), 'Color','blue', 'LineStyle','--');
title("Filtered Estimate of Tube 1 Bending Radius Over Time");
xlabel("Time [sec]");
ylabel("Bending Radius [m]");
legend(["True Radius", "Filtered Radius", "95% Confidence Boundary"]);
ylim([0 0.3]);

subplot(num_tubes,1,2);
hold on
plot(tspan, 1 ./ trueStates(5,:), 'Color','black', 'LineStyle','-');
plot(tspan, 1 ./ estimateStates(5,:), 'Color','red', 'LineStyle','-');
plot(tspan, 1 ./ (estimateStates(5,:)+k2_confidence), 'Color','red', 'LineStyle','--');
plot(tspan, 1 ./ (estimateStates(5,:)-k2_confidence), 'Color','red', 'LineStyle','--');
title("Filtered Estimate of Tube 2 Bending Radius Over Time");
xlabel("Time [sec]");
ylabel("Bending Radius[m]");
legend(["True Radius", "Filtered Radius", "95% Confidence Boundary"]);
ylim([0 0.3]);

if num_tubes==3

subplot(num_tubes,1,3);
hold on
plot(tspan, 1 ./ trueStates(6,:), 'Color','black', 'LineStyle','-');
plot(tspan, 1 ./ estimateStates(6,:), 'Color','green', 'LineStyle','-');
plot(tspan, 1 ./ (estimateStates(6,:)+k3_confidence), 'Color','green', 'LineStyle','--');
plot(tspan, 1 ./ (estimateStates(6,:)-k3_confidence), 'Color','green', 'LineStyle','--');
title("Filtered Estimate of Tube 3 Bending Radius Over Time");
xlabel("Time [sec]");
ylabel("Bending Radius [m]");
legend(["True Radius", "Filtered Radius", "95% Confidence Boundary"]);
ylim([0 0.3]);

end



figures = [f1; f2];

%% Plot tip position

f3 = figure;

subplot(3,1,1);
plot(tspan, trueStates(1,:));
title("True X Position");
xlabel("Time [sec]");
ylabel("Position [m]");

subplot(3,1,2);
plot(tspan, trueStates(2,:));
title("True Y Position");
xlabel("Time [sec]");
ylabel("Position [m]");

subplot(3,1,3);
plot(tspan, trueStates(3,:));
title("True Z Position");
xlabel("Time [sec]");
ylabel("Position [m]");


disp("plot done");


%% Helper functions

    function stateNext = stateModel(state,dt)

        for j=1:num_tubes
            robot.tubes(j).k = state(3+j);
        end
        
        T = robot.fkin(q);
        xyz = T(1:3,4);

        stateNext = [xyz; state(4:4+num_tubes-1)];

    end

    function z = measureModel(state)
        z = state(1:3);
    end

% Define a piecewise function to determine how the "real" robot's tubes
% change curvature over time
    function q = get_q(t, q_target, noise_covar)

        t_mod = mod(t, 10);

        if t_mod <= 5
            q = q_target * t_mod/5;
        else
            q = q_target * (1 - (t_mod-5)/5);
        end


        q = q + (sqrt(noise_covar)*randn(num_tubes*2,1))';


    end


end