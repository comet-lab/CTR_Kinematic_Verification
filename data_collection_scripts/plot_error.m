range = 1:21;
ee_fk_pos_mm = ee_fk_pos * 10^3;
ee_rb_pos_mm = ee_rb_pos * 10^3;

subplot(3,1,1)
plot(range, abs(ee_fk_pos_mm(:,1) - ee_rb_pos_mm(:,1)));
ylim([0, 20]);
xlabel("Test Points")
ylabel("Error [mm]");
title("X Error");

subplot(3,1,2)
plot(range, abs(ee_fk_pos_mm(:,2) - ee_rb_pos_mm(:,2)));
ylim([0, 5]);
xlabel("Test Points")
ylabel("Error [mm]");
title("Y Error");

subplot(3,1,3)
plot(range, abs(ee_fk_pos_mm(:,3) - ee_rb_pos_mm(:,3)));
ylim([0, 20]);
xlabel("Test Points")
ylabel("Error [mm]");
title("Z Error");

sgtitle("XYZ Error")