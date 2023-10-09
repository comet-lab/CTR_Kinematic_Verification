%%%%%%%%%%%%%%
% Run fkin_2tubes.m before running this script
% to laod all the required variables into the workspace


tp = theaterPlot();
op1 = orientationPlotter(tp,'DisplayName','FK','MarkerSize',5,'MarkerFaceColor','blue','LocalAxesLength',0.005);
op2 = orientationPlotter(tp,'DisplayName','Mocap','MarkerSize',5,'MarkerFaceColor','red','LocalAxesLength',0.005);
% op2 = orientationPlotter(tp,'DisplayName','FK w/ tor','MarkerSize',5,'MarkerFaceColor','red','LocalAxesLength',0.01);

quat1 = quaternion(ee_fk_quat(1:50,:));
quat2 = quaternion(ee_rb_quat(1:50,:));

% plotOrientation(op1,quat1, ee_fk_pos);
% plotOrientation(op2,quat2, ee_rb_pos);

plotOrientation(op1,ee_fk_rotm, ee_fk_pos);
plotOrientation(op2,ee_rb_rotm, ee_rb_pos);
% plotOrientation(op2,ee_fkw_rotm, ee_fkw_pos);



for ii = 1:test_points
    t = text(ee_fk_pos(ii,1),ee_fk_pos(ii,2), ee_fk_pos(ii,3), num2str(ii));
    t.Color = [0 0 1];
end

for ii = 1:test_points
    t = text(ee_rb_pos(ii,1),ee_rb_pos(ii,2), ee_rb_pos(ii,3), num2str(ii));
    t.Color = [1 0 0];
end

% for ii = 1:test_points
%     t = text(ee_fkw_pos(ii,1),ee_fkw_pos(ii,2), ee_fkw_pos(ii,3), num2str(ii));
%     t.Color = [1 0 0];
% end