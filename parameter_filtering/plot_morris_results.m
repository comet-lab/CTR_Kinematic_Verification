close all; clear all; clc;

load("morris_screen_rho.mat");


for i=1:num_values
    
    x_val = 5 * (i-1);
    x_vals = x_val * squeeze(ones([num_values 1]));
    
    for j=1:num_values

        y_val = 5 * (j-1);
        y_vals = y_val * squeeze(ones([num_values 1]));

        vals_to_plot = sqrt(squeeze(tube3_z_effects(i,j,:)) .^ 2 + squeeze(tube3_x_effects(i,j,:)) .^ 2);
        % vals_to_plot = squeeze(tube3_z_effects(i,j,:));

        scatter3(x_vals, y_vals, 0:5:100, 5, vals_to_plot, "filled");
        hold on
    end
end

colorbar
xlabel("Joint 1 Translation [mm]");
ylabel("Joint 2 Translation [mm]");
zlabel("Joint 3 Translation [mm]");
title("Elementary Effect of Tube 3 Curvature on Tip Position (Norm)");

