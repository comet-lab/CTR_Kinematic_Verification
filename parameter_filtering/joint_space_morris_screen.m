close all; clear all; clc;

joint_values_to_try = 0:10:360;
num_values = size(joint_values_to_try, 2);

tube1_x_effects = zeros([num_values num_values num_values]);
tube1_y_effects = zeros([num_values num_values num_values]);
tube1_z_effects = zeros([num_values num_values num_values]);
tube2_x_effects = zeros([num_values num_values num_values]);
tube2_y_effects = zeros([num_values num_values num_values]);
tube2_z_effects = zeros([num_values num_values num_values]);
tube3_x_effects = zeros([num_values num_values num_values]);
tube3_y_effects = zeros([num_values num_values num_values]);
tube3_z_effects = zeros([num_values num_values num_values]);

for ii=1:num_values

    theta1 = joint_values_to_try(ii);

    for jj=1:num_values

        theta2 = joint_values_to_try(jj);


        for kk=1:num_values

            theta3 = joint_values_to_try(kk);

            q = [0 0 0 theta1 theta2 theta3];

            [avg, stdev] = morris_screening(q);

            tube1_x_effects(ii, jj, kk) = avg(1,1);
            tube1_y_effects(ii, jj, kk) = avg(1,2);
            tube1_z_effects(ii, jj, kk) = avg(1,3);
            tube2_x_effects(ii, jj, kk) = avg(2,1);
            tube2_y_effects(ii, jj, kk) = avg(2,2);
            tube2_z_effects(ii, jj, kk) = avg(2,3);
            tube3_x_effects(ii, jj, kk) = avg(3,1);
            tube3_y_effects(ii, jj, kk) = avg(3,2);
            tube3_z_effects(ii, jj, kk) = avg(3,3);

        end
    disp(q);
    end
end