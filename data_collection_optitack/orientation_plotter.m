%%%%%%%%%%%%%%
% Run fkin_2tubes.m before running this script
% to laod all the required variables into the workspace


tp = theaterPlot("XLimits",[-0.06, 0.06], "YLimits", [-0.06, 0.06], "ZLimits", [0.04, 0.16]);

quat1 = quaternion(ee_fk_quat);
quat2 = quaternion(ee_rb_quat);


for ii = 1:test_points
    op1 = orientationPlotter(tp,'DisplayName',"FK_"+num2str(ii),'MarkerSize',5,'MarkerFaceColor','blue','LocalAxesLength',0.02);
%     op2 = orientationPlotter(tp,'DisplayName',"Mocap_"+num2str(ii),'MarkerSize',5,'MarkerFaceColor','red','LocalAxesLength',0.02);
    op2 = orientationPlotter(tp,'DisplayName','FK w/ tor','MarkerSize',5,'MarkerFaceColor','red','LocalAxesLength',0.01);
    plotOrientation(op1,ee_fk_rotm(:,:,ii), ee_fk_pos(ii,:));
%     plotOrientation(op2,ee_rb_rotm(:,:,ii), ee_rb_pos(ii,:));
    plotOrientation(op2,ee_fkw_rotm(:,:,ii), ee_fkw_pos(ii,:));
    pause(2)
    clearPlotterData(tp)
    delete(op1)
    delete(op2)
end