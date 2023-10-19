clear; clc;

load("torsion_compare.mat")

pcc_pos = zeros(size(T_pcc,2),3);
em_pos = zeros(size(T_em,2),3);
as_pos = zeros(size(T_as,2),3);

for i = 1:size(T_pcc,2)
    pcc_pos(i,:) = T_pcc{i}(1:3,4)'*1000;
    em_pos(i,:) = T_em{i}(1:3,4)'*1000;
    as_pos(i,:) = T_as{i}(1:3,4)'*1000;
end




figure(1)
range = 1:size(T_pcc,2);
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
ylim([20, 60]);
xlabel("Theta2 [deg]")
ylabel("Position [mm]");
title("Z Position");
legend("PCC", "EM", "AS");

sgtitle("XYZ Positions")