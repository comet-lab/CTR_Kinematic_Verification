clear;

num_tubes = 2;      % number of tubes for test

% tube parameter: (id, od, curvature, straight section length, arc length, young's modulus)
tube1 = Tube(2.792*10^-3, 3.3*10^-3, 1/17, 90*10^-3, 50*10^-3, 1935*10^6);
tube2 = Tube(2.132*10^-3, 2.64*10^-3, 1/22, 170*10^-3, 50*10^-3, 1935*10^6);
tube3 = Tube(1.472*10^-3, 1.98*10^-3, 1/29, 250*10^-3, 50*10^-3, 193*10^6);

tubes = [tube1, tube2];

% translation joint varibales
rho = [0,0];

% rotation joint variables
theta = deg2rad([60, -30]);

%arc paramters
chi_num = tubes(1).E*tubes(1).I*tubes(1).k*cos(theta(1)) + tubes(2).E*tubes(2).I*tubes(2).k*cos(theta(2));
chi = chi_num/(tubes(1).E*tubes(1).I + tubes(2).E*tubes(2).I);

gamma_num = tubes(1).E*tubes(1).I*tubes(1).k*sin(theta(1)) + tubes(2).E*tubes(2).I*tubes(2).k*sin(theta(2));
gamma = gamma_num/(tubes(1).E*tubes(1).I + tubes(2).E*tubes(2).I);

phi = rad2deg(atan2(gamma, chi))