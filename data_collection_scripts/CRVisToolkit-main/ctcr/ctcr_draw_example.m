addpath ../util/

kappa = [1/30e-3; 1/40e-3];
phi = [0;deg2rad(160)];
ell = [50e-3;70e-3];
pts_per_seg = 30;

g = robotindependentmapping(kappa,phi,ell,pts_per_seg);

draw_ctcr(g,[30 60],[2e-3 1.5e-3])