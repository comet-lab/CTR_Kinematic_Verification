%% Experiment Hyperparameters

close all; clear all; clc;

num_trials = 250;

k_real_mean = 25;
k_real_stdev = 10;

k_guess_mean = 20;
k_guess_stdev = 15;

q0 = [0 0 0 0 0 0];
q_target_mean = [50 50 50 0 0 0];
q_target_stdev = [25 25 25 180 180 180];

simTime = 50;
filter_hz = 10;

processNoiseXYZMean = 2e-3;
processNoiseXYZStDev = 0.5e-3;

processNoiseKMean = 10e-3;
processNoiseKStDev = 2.5e-3;

measureNoiseXYZMean = 2e-3;
measureNoiseXYZStDev = 0.5e-3;

jointVarNoiseRhoMean = 2;
jointVarNoiseRhoStDev = 0.5;

jointVarNoiseThetaMean = 3;
jointVarNoiseThetaStDev = 0.5;

initialCovariance = diag([100 100 100 100 100 100]);

all_vars_to_save = [];

for i=1:num_trials

    disp("Beginning trial " + i + " out of " + num_trials + " tests");

    %% Set up the trial configuration

    k_real = abs(randn([1 3]) .* k_real_stdev + k_real_mean);

    for j=1:3
        if k_real(j) < 5
            k_real(j) = k_real(j) + 5;
        end
    end

    k_guess = abs(randn([1 3]) .* k_guess_stdev + k_guess_mean);

    q0 = q0;
    q_target = randn([1,6]) .* q_target_stdev + q_target_mean;

    simTime = simTime;
    dt = 1 / filter_hz;

    processNoiseXYZOffDiagonalRatio = randn() * 0.5 + 0.5;
    if processNoiseXYZOffDiagonalRatio < 0
        processNoiseXYZOffDiagonalRatio  = -processNoiseXYZOffDiagonalRatio;
    end
    while processNoiseXYZOffDiagonalRatio > 1
        processNoiseXYZOffDiagonalRatio  = processNoiseXYZOffDiagonalRatio - 1;
    end

    processNoiseXYZ = randn() * processNoiseXYZStDev + processNoiseXYZMean;
    processNoiseXYZOffDiagonal = processNoiseXYZ * processNoiseXYZOffDiagonalRatio;

    processNoiseKOffDiagonalRatio = randn() * 0.5 + 0.5;
    if processNoiseKOffDiagonalRatio < 0
        processNoiseKOffDiagonalRatio  = -processNoiseKOffDiagonalRatio;
    end
    while processNoiseKOffDiagonalRatio > 1
        processNoiseKOffDiagonalRatio  = processNoiseKOffDiagonalRatio - 1;
    end

    processNoiseK = randn() * processNoiseKStDev + processNoiseKMean;
    processNoiseKOffDiagonal = processNoiseK * processNoiseKOffDiagonalRatio;

    processNoise = ([processNoiseXYZ processNoiseXYZOffDiagonal processNoiseXYZOffDiagonal 0 0 0;
        processNoiseXYZOffDiagonal processNoiseXYZ processNoiseXYZOffDiagonal 0 0 0;
        processNoiseXYZOffDiagonal processNoiseXYZOffDiagonal processNoiseXYZ 0 0 0;
        0 0 0 processNoiseK processNoiseKOffDiagonal processNoiseKOffDiagonal;
        0 0 0 processNoiseKOffDiagonal processNoiseK processNoiseKOffDiagonal;
        0 0 0 processNoiseKOffDiagonal processNoiseKOffDiagonal processNoiseK]) .^ 2;

    measureNoiseXYZOffDiagonalRatio = randn() * 0.5 + 0.5;
    if measureNoiseXYZOffDiagonalRatio < 0
        measureNoiseXYZOffDiagonalRatio  = -measureNoiseXYZOffDiagonalRatio;
    end
    while measureNoiseXYZOffDiagonalRatio > 1
        measureNoiseXYZOffDiagonalRatio  = measureNoiseXYZOffDiagonalRatio - 1;
    end
    

    measureNoiseXYZ = randn() * measureNoiseXYZStDev + measureNoiseXYZMean;
    measureNoiseXYZOffDiagonal = measureNoiseXYZ * measureNoiseXYZOffDiagonalRatio;

    measureNoise = ([measureNoiseXYZ measureNoiseXYZOffDiagonal measureNoiseXYZOffDiagonal;
        measureNoiseXYZOffDiagonal measureNoiseXYZ measureNoiseXYZOffDiagonal;
        measureNoiseXYZOffDiagonal measureNoiseXYZOffDiagonal measureNoiseXYZ]) .^ 2;

    jointVarNoiseRhoOffDiagonalRatio = randn() * 0.5 + 0.5;
    if jointVarNoiseRhoOffDiagonalRatio < 0
        jointVarNoiseRhoOffDiagonalRatio  = -jointVarNoiseRhoOffDiagonalRatio;
    end
    while jointVarNoiseRhoOffDiagonalRatio > 1
        jointVarNoiseRhoOffDiagonalRatio  = jointVarNoiseRhoOffDiagonalRatio - 1;
    end

    jointVarNoiseRho = randn() * jointVarNoiseRhoStDev + jointVarNoiseRhoMean;
    jointVarNoiseRhoOffDiagonal = jointVarNoiseRho * jointVarNoiseRhoOffDiagonalRatio;

    jointVarNoiseThetaOffDiagonalRatio = randn() * 0.5 + 0.5;
    if jointVarNoiseThetaOffDiagonalRatio < 0
        jointVarNoiseThetaOffDiagonalRatio  = -jointVarNoiseThetaOffDiagonalRatio;
    end
    while jointVarNoiseThetaOffDiagonalRatio > 1
        jointVarNoiseThetaOffDiagonalRatio  = jointVarNoiseThetaOffDiagonalRatio - 1;
    end

    jointVarNoiseTheta = randn() * jointVarNoiseThetaStDev + jointVarNoiseThetaMean;
    jointVarNoiseThetaOffDiagonal = jointVarNoiseTheta * jointVarNoiseThetaOffDiagonalRatio;

    jointVarNoise = ([jointVarNoiseRho jointVarNoiseRhoOffDiagonal jointVarNoiseRhoOffDiagonal 0 0 0;
        jointVarNoiseRhoOffDiagonal jointVarNoiseRho jointVarNoiseRhoOffDiagonal 0 0 0;
        jointVarNoiseRhoOffDiagonal jointVarNoiseRhoOffDiagonal jointVarNoiseRho 0 0 0;
        0 0 0 jointVarNoiseTheta jointVarNoiseThetaOffDiagonal jointVarNoiseThetaOffDiagonal;
        0 0 0 jointVarNoiseThetaOffDiagonal jointVarNoiseTheta jointVarNoiseThetaOffDiagonal;
        0 0 0 jointVarNoiseThetaOffDiagonal jointVarNoiseThetaOffDiagonal jointVarNoiseTheta]) .^ 2;

    initialCovariance = initialCovariance;


    %% Save the trial configuration

    clear figures

    save("./experiments-3tubes/"+i+".mat");

    vars_to_save = [i, k_real, k_guess, q0, q_target, simTime, dt, processNoiseXYZ, processNoiseXYZOffDiagonal, processNoiseK, processNoiseKOffDiagonal, measureNoiseXYZ, measureNoiseXYZOffDiagonal, jointVarNoiseRho, jointVarNoiseRhoOffDiagonal, jointVarNoiseTheta, jointVarNoiseThetaOffDiagonal, initialCovariance(1)];
    all_vars_to_save = [all_vars_to_save; vars_to_save];

    %% Execute the trial

    figures = extended_kalman_filter(k_real, k_guess, q0, q_target, simTime, dt, processNoise, measureNoise, jointVarNoise, initialCovariance);

    %% Save the results

    saveas(figures(1), "./experiments-3tubes/"+i+"-tip_pos_error.png");
    saveas(figures(1), "./experiments-3tubes/"+i+"-tip_pos_error.fig");
    saveas(figures(2), "./experiments-3tubes/"+i+"-curvatures.png");
    saveas(figures(2), "./experiments-3tubes/"+i+"-curvatures.fig");


end

writematrix(all_vars_to_save, "./experiments-3tubes/all-experiments.csv");


