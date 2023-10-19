%% Load all variables
clear; clc;

load("torsion_compare.mat")

% Get XYZ position values from the transformation matrices
pcc_pos = zeros(size(T_pcc,2),3);
em_pos = zeros(size(T_em,2),3);
as_pos = zeros(size(T_as,2),3);

for i = 1:size(T_pcc,2)
    pcc_pos(i,:) = T_pcc{i}(1:3,4)'*1000;
    em_pos(i,:) = T_em{i}(1:3,4)'*1000;
    as_pos(i,:) = T_as{i}(1:3,4)'*1000;
end


%% Create XYZ position plot of both models
figure(1)

subplot(3,1,1)
plot(rotations, [pcc_pos(:,1), em_pos(:,1), as_pos(:,1)]);
grid on;
set(gca,'FontSize',16,'fontWeight','bold')
set(findall(gcf,'type','text'),'FontSize',16,'fontWeight','bold')
ylim([0, 30]);
xlabel("Theta2 [deg]")
ylabel("Position [mm]");
title("X Position");
legend("PCC", "EM", "AS");

subplot(3,1,2)
plot(rotations, [pcc_pos(:,2), em_pos(:,2), as_pos(:,2)]);
grid on;
set(gca,'FontSize',16,'fontWeight','bold')
set(findall(gcf,'type','text'),'FontSize',16,'fontWeight','bold')
ylim([-30, 30]);
xlabel("Theta2 [deg]")
ylabel("Position [mm]");
title("Y Position");
legend("PCC", "EM", "AS");

subplot(3,1,3)
plot(rotations, [pcc_pos(:,3), em_pos(:,3), as_pos(:,3)]);
grid on;
set(gca,'FontSize',16,'fontWeight','bold')
set(findall(gcf,'type','text'),'FontSize',16,'fontWeight','bold')
ylim([40, 60]);
xlabel("Theta2 [deg]")
ylabel("Position [mm]");
title("Z Position");
legend("PCC", "EM", "AS");

sgtitle("XYZ Positions")

%% Plot psi values compared to alpha values for analytical solution
num_points = size(T_pcc(),2);
alpha = zeros(num_points,2);
alpha(:,2) = deg2rad(rotations)';

figure(2)
range = 1:num_points;

subplot(2,1,1)
plot(range, [alpha(:,1), psi_as(:,1)]);
grid on;
set(gca,'FontSize',16,'fontWeight','bold')
set(findall(gcf,'type','text'),'FontSize',16,'fontWeight','bold')
ylim([-2,10]);
xlabel("Point Num")
ylabel("Angle [rad]");
title("Tube 1");
legend("Alpha1", "Psi1");

subplot(2,1,2)
plot(range, [alpha(:,2), psi_as(:,2)]);
grid on;
set(gca,'FontSize',16,'fontWeight','bold')
set(findall(gcf,'type','text'),'FontSize',16,'fontWeight','bold')
ylim([-2,10]);
xlabel("Point Num")
ylabel("Angle [rad]");
title("Tube 2");
legend("Alpha2", "Psi2");

sgtitle("Analytical solution Alpha and Psi Values")

%% Plot psi values compared to alpha values for energy minimization
num_points = size(T_pcc(),2);
alpha = zeros(num_points,2);
alpha(:,2) = deg2rad(rotations)';

figure(3)
range = 1:num_points;

subplot(2,1,1)
plot(range, [alpha(:,1), psi_em(:,1)]);
grid on;
set(gca,'FontSize',16,'fontWeight','bold')
set(findall(gcf,'type','text'),'FontSize',16,'fontWeight','bold')
ylim([-2,10]);
xlabel("Point Num")
ylabel("Angle [rad]");
title("Tube 1");
legend("Alpha1", "Psi1");

subplot(2,1,2)
plot(range, [alpha(:,2), psi_em(:,2)]);
grid on;
set(gca,'FontSize',16,'fontWeight','bold')
set(findall(gcf,'type','text'),'FontSize',16,'fontWeight','bold')
ylim([-2,10]);
xlabel("Point Num")
ylabel("Angle [rad]");
title("Tube 2");
legend("Alpha2", "Psi2");

sgtitle("Energy Minimization Alpha and Psi Values")