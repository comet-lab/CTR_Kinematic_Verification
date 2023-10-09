for i =2:test_points
%     axangRB = quat2axang(quatmultiply(ee_rb_quat(i,:),quatinv(ee_rb_quat(i-1,:))));
    d_quat = (quatmultiply(ee_rb_quat(i,:),quatinv(ee_rb_quat(i-1,:))));
    e_quat = (quatmultiply(ee_fk_quat(i,:),quatinv(ee_fk_quat(i-1,:))));
%     ang_rb(i-1,1) = rad2deg(axangRB(4));
    d_ang(i-1,1) = rad2deg(2*acos(d_quat(1)));
    e_ang(i-1,1) = rad2deg(2*acos(e_quat(1)));
    psi_deg = rad2deg(psi);
    theta_deg = rad2deg(theta);
    d_psi(i-1,1) = psi_deg(i,2) - psi_deg(i-1,2);
    d_theta(i-1,1) = theta_deg(i,2) - theta_deg(i-1,2);
end

range_d = 1:test_points-1;

% plot(range_d, [abs(d_psi), abs(ang_rb)]);
figure(1)
boxplot([abs(d_psi), abs(d_theta), abs(d_psi)] - [abs(ang_rb), abs(ang_rb), abs(d_theta)], 'Labels',{'psi vs mocap', 'theta vs mocap', 'psi vs theta'})

set(gca,'FontSize',16,'fontWeight','bold')
set(findall(gcf,'type','text'),'FontSize',16,'fontWeight','bold')
xlabel("error in rotation btw")
ylabel("[deg]");
title("data: May 19 rotation exp");


figure(2)
boxplot([abs(d_ang)] - [abs(e_ang)], 'Labels',{'fkin vs mocap'})

set(gca,'FontSize',16,'fontWeight','bold')
set(findall(gcf,'type','text'),'FontSize',16,'fontWeight','bold')
xlabel("error in rotation btw")
ylabel("[deg]");
title("data: May 19 rotation exp");