close all; clear all; clc;

disp("hello");

load ./experiments/1.mat

disp("redoing 1");


figures = extended_kalman_filter(k_real, k_guess, q0, q_target, simTime, dt, processNoise, measureNoise, jointVarNoise, initialCovariance);