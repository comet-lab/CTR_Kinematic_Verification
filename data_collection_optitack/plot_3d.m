%%%%%%%%%%%%%%
% Run fkin_2tubes.m before running this script
% to laod all the required variables into the workspace


ee_fk_pos_mm = ee_fk_pos * 10^3;
ee_rb_pos_mm = ee_rb_pos * 10^3;

figure();
% hold on
scatter3([ee_fk_pos_mm(:,1), ee_rb_pos_mm(:,1)], [ee_fk_pos_mm(:,2), ee_rb_pos_mm(:,2)], [ee_fk_pos_mm(:,3), ee_rb_pos_mm(:,3)]);
% plot3(ee_fk_pos_mm(:,1), ee_fk_pos_mm(:,2), ee_fk_pos_mm(:,3), ee_rb_pos_mm(:,1), ee_rb_pos_mm(:,2), ee_rb_pos_mm(:,3));
% scatter3(ee_rb_pos_mm(:,1), ee_rb_pos_mm(:,2), ee_rb_pos_mm(:,3));
% hold off
xlim([0,60]);
ylim([-30,30]);
zlim([60,120]);
xlabel("X");
ylabel("Y");
zlabel("Z");
legend("FK", "Mocap");