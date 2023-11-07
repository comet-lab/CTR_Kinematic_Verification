function [x_correct,covariance_correct] = kalman_filter(covariance_prev, prediction, observation)
%KALMAN_FILTER Summary of this function goes here
%   Detailed explanation goes here

%% Hardcoded Matrices

% Note: I could have used the zeros() command to make this section more
% concise. But explicitly defining the matrices allows for easier edits and
% easier visualization.

% In the prediction step, we gather our predicted values of x, y, and z
% from the forward kinematics, and we assume that the FK is perfect.
% Therefore we do not care about the previous values of x, y, and z; we
% only care about previous curvature values. We assume that curvature does
% not change, so our prediction matches our previous curvature values.
F = [0 0 0 0 0;
     0 0 0 0 0;
     0 0 0 0 0;
     0 0 0 1 0;
     0 0 0 0 1];

% Q represents the process noise. We assume there is no noise in our robot
% motion, but we add some noise to the curvature values.
% Q = [0 0 0 0    0;
%      0 0 0 0    0;
%      0 0 0 0    0;
%      0 0 0 0.01 0.01;
%      0 0 0 0.01 0.01];
Q = 0.1 * ones([5 5]);
Q(4, 1:3) = 10;
Q(4, 4:5) = 1;
Q(5, 1:3) = 10;
Q(5, 4:5) = 3;

% H is the observation function. We assume that we can observe tip position
% but not curvature
H = [1 0 0 0 0;
     0 1 0 0 0;
     0 0 1 0 0];

% R represents the observation noise. We assume that we can observe tip
% position perfectly, so this matrix will be filled with zeros.
R = [0 0 0;
     0 0 0;
     0 0 0];

%% Prediction

x_predict = prediction;

covariance_predict = F * covariance_prev * F' + Q;

%% Correction/Analysis

K = covariance_predict * H' * pinv(H * covariance_predict * H' + R);

x_correct = x_predict + K * (observation - H * x_predict);

covariance_correct = covariance_predict - K * H * covariance_predict;

if max(covariance_correct-covariance_correct') > 1e-3
    disp(covariance_correct);
    disp
    disp(covariance_correct - covariance_correct');
    disp(max(covariance_correct - covariance_correct'));
    error("covar matrix not symmetric");
end

end

