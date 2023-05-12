%%%%%%%%%%%%%%
% Run fkin_2tubes.m before running this script
% to laod all the required variables into the workspace


ee_fk_pos_mm = ee_fk_pos * 10^3;
ee_rb_pos_mm = ee_rb_pos * 10^3;
f1=[0 0 1];
range = 1:test_points;
figure();
% hold on
% plot3(range, [ee_fk_pos_mm(:,1), ee_rb_pos_mm(:,1)], [ee_fk_pos_mm(:,2), ee_rb_pos_mm(:,2)]);
scatter3([ee_fk_pos_mm(:,1), ee_rb_pos_mm(:,1)], [ee_fk_pos_mm(:,2), ee_rb_pos_mm(:,2)], [ee_fk_pos_mm(:,3), ee_rb_pos_mm(:,3)],'*');
% scatter3(ee_rb_pos_mm(:,1), ee_rb_pos_mm(:,2), ee_rb_pos_mm(:,3));
% hold off
xlim([-20,80]);
ylim([-50,50]);
zlim([40,140]);
set(gca,'FontSize',16,'fontWeight','bold')
set(findall(gcf,'type','text'),'FontSize',16,'fontWeight','bold')
xlabel("X (mm)");
ylabel("Y (mm)");
zlabel("Z (mm)");
legend("FK", "Mocap");

for ii = 1:test_points
    t = text(ee_fk_pos_mm(ii,1),ee_fk_pos_mm(ii,2), ee_fk_pos_mm(ii,3), num2str(ii));
    t.Color = [0 0 1];
end

for ii = 1:test_points
    t = text(ee_rb_pos_mm(ii,1),ee_rb_pos_mm(ii,2), ee_rb_pos_mm(ii,3), num2str(ii));
    t.Color = [1 0 0];
end