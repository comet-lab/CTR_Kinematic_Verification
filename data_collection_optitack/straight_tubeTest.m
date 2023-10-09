filename = 'data_files/26_04_23-14_45_2-tubes_in-plane-bending';
% filename = '26_04_23-14_51_2-tubes_in-plane-bending';
test_points = 21;   % number of test points
fpp = 10;           % number of frames per point


T1 = readtable(filename);

% data frame is lin1, lin2, rot1, rot2 and sampling every fpp'th point
s_tube = table2array([T1(1:fpp:fpp*test_points,8)]);
% ee -> end effector / tip

% data frame is x, y, z, qw, qx, qy, qz
ee_mocap_data = table2array([T1(1:test_points*fpp,1),T1(1:test_points*fpp,2),T1(1:test_points*fpp,3), ...
                    T1(1:test_points*fpp,4), T1(1:test_points*fpp,5),T1(1:test_points*fpp,6), T1(1:test_points*fpp,7)]);


for i = 1:test_points   
    ee_mocap_data_avg(i,:) = mean(ee_mocap_data(((i-1)*fpp)+1:i*fpp,:));
    % the average data has average quaternions that may not be normalised

    ee_mocap_pos_raw(1:10,i) = ee_mocap_data((i-1)*fpp+1:i*fpp,2);
%     ee_mocap_pos_biased(1:10,i) = ee_mocap_data((i-1)*fpp+1:i*fpp,2) - ee_mocap_data_avg(i,2);

    ee_mocap_pos(i,:) = ee_mocap_data_avg(i,1:3); 
    offset = ee_mocap_pos(1,2);
    ee_pos_offset(i,1) = (ee_mocap_pos(i,2) - offset)*10^3; 
    ee_mocap_pos_biased(1:10,i) = ee_mocap_data((i-1)*fpp+1:i*fpp,2) - offset + s_tube(i,1);


end

pos_error = s_tube - ee_pos_offset;
rmse_pos_error = rmse(ee_pos_offset(:), s_tube(:));

range = 1:test_points;

figure (1)
plot(range, [s_tube, ee_pos_offset]);
% plot(range, [ee_fkw_pos_mm(:,3), ee_fk_pos_mm(:,3), ee_rb_pos_mm(:,3)]);
set(gca,'FontSize',16,'fontWeight','bold')
set(findall(gcf,'type','text'),'FontSize',16,'fontWeight','bold')
ylim([-10, 60]);

xlabel("Test Points")
ylabel("Position [mm]");
% legend("FK w/o torsion", "FK w/ torsion", "Mocap");
legend("FK", "Mocap");
title("Positions")

figure(2)
boxplot(s_tube - ee_pos_offset, 'Labels',{'translation of tube'})
set(gca,'FontSize',16,'fontWeight','bold')
set(findall(gcf,'type','text'),'FontSize',16,'fontWeight','bold')
% xlabel("Coordinates axes")
ylabel("[mm]");
title("data: April 26 datataset 2");

figure(3)
boxplot(ee_mocap_pos_raw)
set(gca,'FontSize',16,'fontWeight','bold')
set(findall(gcf,'type','text'),'FontSize',16,'fontWeight','bold')
% xlabel("Coordinates axes")
ylabel("[mm]");
title("4-26 d2: variation at each location");

figure(4)
boxplot(ee_mocap_pos_biased)
set(gca,'FontSize',16,'fontWeight','bold')
set(findall(gcf,'type','text'),'FontSize',16,'fontWeight','bold')
% xlabel("Coordinates axes")
ylabel("[mm]");
title("4-26: error variation at each location");