% addpath ../util/

kappa = [1/30e-3; 1/40e-3];
phi = [0;deg2rad(160)];
ell = [50e-3;30e-3];
% kappa = [8.564; 11.66; 14.00];
% phi = [0;0;0];
% ell = [0.0300; 0.0200; 0.0300];
pts_per_seg = 30;

g = robotindependentmapping(kappa,phi,ell,pts_per_seg);
d = 50 * 10^-3;
points = [pts_per_seg 2*pts_per_seg - pts_per_seg * (ell(2) / d)];
draw_ctcr(g,[30 60],[2e-3 1.5e-3]);