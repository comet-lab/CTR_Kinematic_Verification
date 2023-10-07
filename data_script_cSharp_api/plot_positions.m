%%%%%%%%%%%%%%
% Run fkin_2tubes.m before running this script
% to laod all the required variables into the workspace


range = 1:test_points;
ee_fk_pos_mm = ee_fk_pos*10^3;
ee_hf_pos_mm = ee_hf_pos*10^3;
% ee_fkw_pos_mm = ee_fkw_pos * 10^3;

figure(1)

subplot(3,1,1)
plot(range, [ee_fk_pos_mm(:,1), ee_hf_pos_mm(:,1)]);
grid on;
set(gca,'FontSize',16,'fontWeight','bold')
set(findall(gcf,'type','text'),'FontSize',16,'fontWeight','bold')
ylim([-25, 75]);
xlabel("Test Points")
ylabel("Position [mm]");
title("X Position");
legend("FK w/o torsion", "Mocap");
% legend("FK w/o torsion", "Mocap");

subplot(3,1,2)
plot(range, [ee_fk_pos_mm(:,2), ee_hf_pos_mm(:,2)]);
grid on;
set(gca,'FontSize',16,'fontWeight','bold')
set(findall(gcf,'type','text'),'FontSize',16,'fontWeight','bold')
ylim([-50, 50]);
xlabel("Test Points")
ylabel("Position [mm]");
title("Y Position");
legend("FK w/o torsion", "Mocap");
% legend("FK w/o torsion", "Mocap");

subplot(3,1,3)
plot(range, [ee_fk_pos_mm(:,3), ee_hf_pos_mm(:,3)]);
grid on;
set(gca,'FontSize',16,'fontWeight','bold')
set(findall(gcf,'type','text'),'FontSize',16,'fontWeight','bold')
ylim([25, 125]);

xlabel("Test Points")
ylabel("Position [mm]");
title("Z Position");
legend("FK w/o torsion", "Mocap");
% legend("FK w/o torsion", "Mocap");

sgtitle("XYZ Positions")


figure(2)
boxplot(abs(norm1*10^3 - norm2*10^3));
grid on;
set(gca,'FontSize',16,'fontWeight','bold')
set(findall(gcf,'type','text'),'FontSize',16,'fontWeight','bold')
xlabel("Error in tip position")
ylabel("[mm]");
title("Experiment 3 (21 datapoints)");
