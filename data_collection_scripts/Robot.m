classdef Robot < handle

    properties
        num_tubes = 2

        tube1 = Tube(4.6*10^-3, 5.8*10^-3, 1/11, 91.21*10^-3, 50*10^-3, 1935*10^6)
        tube2 = Tube(2.8*10^-3, 4.0*10^-3, 1/14, 161.0*10^-3, 50*10^-3, 1935*10^6)

        rot_ee = [] %end effector rotation
        pos_ee = [] % end effector translation

        Psi = []
        Theta = []

        plotOn = false
    end

    methods
        function self = Robot(num_tubes, plotOn)
            self.num_tubes = num_tubes;
            self.plotOn = plotOn;
        end

        function T = fkin(self, q_var)  
            % First we get the link lengths 
            s = get_links(self, q_var);

            % Next we get the values for theta
            theta = get_theta(self, q_var);            

            % Now we calculate the phi and kappa values
            [phi,K] = calculate_phi(self, theta);

            % Finally we calculate the ending transform
            T = calculate_transform(self, s, phi, K);

            if self.plotOn
                plot_tube(self, q_var, s, phi, K)
                w = waitforbuttonpress;
            end
        end


        % Function to find the links
        function s = get_links(self, q_var)
            % Here q_var = [lin1, lin2, rot1, rot2] or
            % q _var = [lin1, lin2, lin3, rot1, rot2, rot3]

            % First define rho values in terms of joint variables. We
            % assume that T1 moves with q_var(1). Since we want rho1 to be 0,
            % we have to subtract q_var(1) from other variables. 

            if(self.num_tubes == 2)
                rho = [0, q_var(2) - q_var(1)]*10^-3;
                d = [self.tube1.d, self.tube2.d];
                T = [rho(1), rho(2), rho(1)+d(1), rho(2)+d(2)]; 

                Ts = sort(T);
                s = [Ts(2)-Ts(1), Ts(3)-Ts(2), Ts(4)-Ts(3)];
            else
                rho = [0, q_var(2) - q_var(1), q_var(3) - q_var(1)]*10^-3;
                d = [self.tube1.d, self.tube2.d, self.tube3.d];
                T = [rho(1), rho(2), rho(3), rho(1)+d(1), rho(2)+d(2), rho(3) + d(3)];

                Ts = sort(T);

                % Not sure I got this line below correct, check when doing
                % three tubes
                s = [Ts(2)-Ts(1), Ts(3)-Ts(2), Ts(4)-Ts(3), Ts(5)-Ts(4), Ts(6)-Ts(5)];
            end
        end

        % Function to get theta values
        function theta = get_theta(self, q_var)
            % We know thete from the joint variables
                
            if (self.num_tubes == 2)
                theta = [deg2rad(q_var(3)), deg2rad(q_var(4))];
            else
                theta = [deg2rad(q_var(4)), deg2rad(q_var(5)), deg2rad(q_var(6))];
            end
        end


        % Function to calcualte phi values for two or three tube
        % configurations
        function [phi,K] = calculate_phi(self, theta)

            [chi, gamma] = in_plane_param(self, theta);

            if(self.num_tubes == 2)
        
                phi1 = atan2(gamma(1), chi(1));
                phi2 = atan2(gamma(2), chi(2));
                phi3 = atan2(gamma(3), chi(3));
        
                phi = [phi1, phi2, phi3];

                K1 = sqrt(chi(1)*chi(1) + gamma(1)*gamma(1));
                K2 = sqrt(chi(2)*chi(2) + gamma(2)*gamma(2));
                K3 = sqrt(chi(3)*chi(3) + gamma(3)*gamma(3));
        
                K = [K1, K2, K3];
            else

                phi1 = atan2(gamma(1), chi(1));
                phi2 = atan2(gamma(2), chi(2));
                phi3 = atan2(gamma(3), chi(3));
                phi4 = atan2(gamma(4), chi(4));
                phi5 = atan2(gamma(5), chi(5));
                
                phi = [phi1, phi2, phi3, phi4, phi5];

                K1 = sqrt(chi(1)*chi(1) + gamma(1)*gamma(1));
                K2 = sqrt(chi(2)*chi(2) + gamma(2)*gamma(2));
                K3 = sqrt(chi(3)*chi(3) + gamma(3)*gamma(3));
                K4 = sqrt(chi(4)*chi(4) + gamma(4)*gamma(4));
                K5 = sqrt(chi(5)*chi(5) + gamma(5)*gamma(5));
        
                K = [K1, K2, K3, K4, K5];
            end
        end

        function T = calculate_transform(self, s, phi, K)
            T=[];

            if(self.num_tubes ==2)
                for i =1:3    
                    tt = [[(cos(phi(i))*cos(phi(i))*(cos(K(i)*s(i))-1)) + 1, sin(phi(i))*cos(phi(i))*(cos(K(i)*s(i))-1), cos(phi(i))*sin(K(i)*s(i)), cos(phi(i))*(1-cos(K(i)*s(i)))/K(i)];
                        [sin(phi(i))*cos(phi(i))*(cos(K(i)*s(i)) -1), + cos(phi(i))*cos(phi(i))*(1 - cos(K(i)*s(i))) + cos(K(i)*s(i)), sin(phi(i))*sin(K(i)*s(i)), sin(phi(i))*(1-cos(K(i)*s(i)))/K(i)];
                        [-cos(phi(i))*sin(K(i)*s(i)), -sin(phi(i))*sin(K(i)*s(i)), cos(K(i)*s(i)), sin(K(i)*s(i))/K(i)];
                        [0, 0, 0, 1]];
                   
                    
                    T(:,:,i) = tt;
                end
            else
                for i =1:5
                    tt = [[cos(phi(i))*cos(K(i)*s(i)), -sin(phi(i)), cos(phi(i))*sin(K(i)*s(i)), cos(phi(i))*(1-cos(K(i)*s(i)))/K(i)];
                        [sin(phi(i))*cos(K(i)*s(i)), +cos(phi(i)), sin(phi(i))*sin(K(i)*s(i)), sin(phi(i))*(1-cos(K(i)*s(i)))/K(i)];
                        [-sin(K(i)*s(i)), 0, cos(K(i)*s(i)), sin(K(i)*s(i))/K(i)];
                        [0, 0, 0, 1]];
                    
                    T(:,:,i) = tt;
                end
            end
        end

        % This is a function to plot the tube as shown by the forward
        % kinematics
        function plot_tube(self, q_var, s, phi, K)
            addpath CRVisToolkit-main\util\
            addpath CRVisToolkit-main\ctcr\

            pts_per_seg = 80;

            g = robotindependentmapping(K', deg2rad(phi)', s', pts_per_seg);
            
            if(self.num_tubes ==2)
                % calculate where the tubes will end
%                 points = [pts_per_seg 2*pts_per_seg - pts_per_seg* (s(2)/self.tube1.d)];
                points = [pts_per_seg 2*pts_per_seg 3*pts_per_seg];
                try
                    draw_ctcr(g,points,[self.tube1.od self.tube1.od self.tube2.od]);
                catch
                    disp("Invalid position to plot")
                end
            else
                translation_frac = [q_var(1)/self.tube1.d q_var(2)/self.tube2.d; q_var(3)/self.tube3.d];

                points = int16(translation_frac * pts_per_seg);

                draw_ctcr(g,points,[self.tube1.od self.tube2.od]);
            end
        end

        
        function [chi, gamma] = in_plane_param(self, theta)

            if(self.num_tubes == 2)
                x1n = self.tube1.E*self.tube1.I*self.tube1.k*cos(theta(1));
                x1 = x1n/(self.tube1.E*self.tube1.I + self.tube2.E*self.tube2.I);
        
                x2n = self.tube1.E*self.tube1.I*self.tube1.k*cos(theta(1)) + self.tube2.E*self.tube2.I*self.tube2.k*cos(theta(2));
                x2 = x2n/(self.tube1.E*self.tube1.I + self.tube2.E*self.tube2.I);
        
                x3n = self.tube2.E*self.tube2.I*self.tube2.k*cos(theta(2));
                x3 = x3n/(self.tube2.E*self.tube2.I);
        
        
                y1n = self.tube1.E*self.tube1.I*self.tube1.k*sin(theta(1));
                y1 = y1n/(self.tube1.E*self.tube1.I + self.tube2.E*self.tube2.I);
        
                y2n = self.tube1.E*self.tube1.I*self.tube1.k*sin(theta(1)) + self.tube2.E*self.tube2.I*self.tube2.k*sin(theta(2));
                y2 = y2n/(self.tube1.E*self.tube1.I + self.tube2.E*self.tube2.I);
        
                y3n = self.tube2.E*self.tube2.I*self.tube2.k*sin(theta(2));
                y3 = y3n/(self.tube2.E*self.tube2.I);
    
                chi = [x1, x2, x3];
                gamma = [y1, y2, y3];
            
            
            else

                x1n = self.tube1.E*self.tube1.I*self.tube1.k*cos(theta(1));
                x1 = x1n/(self.tube1.E*self.tube1.I + self.tube2.E*self.tube2.I + self.tube3.E*self.tube3.I);
        
                x2n = self.tube1.E*self.tube1.I*self.tube1.k*cos(theta(1)) + self.tube2.E*self.tube2.I*self.tube2.k*cos(theta(2));
                x2 = x2n/(self.tube1.E*self.tube1.I + self.tube2.E*self.tube2.I + self.tube3.E*self.tube3.I);
        
                x3n = self.tube2.E*self.tube2.I*self.tube2.k*cos(theta(2));
                x3 = x3n/(self.tube2.E*self.tube2.I + self.tube3.E*self.tube3.I);
    
                x4n = self.tube2.E*self.tube2.I*self.tube2.k*cos(theta(2)) + self.tube3.E*self.tube3.I*self.tube3.k*cos(theta(3));
                x4 = x4n/(self.tube2.E*self.tube2.I + self.tube3.E*self.tube3.I);
    
                x5n = self.tube3.E*self.tube3.I*self.tube3.k*cos(theta(3));
                x5 = x5n/(self.tube3.E*self.tube3.I);
        
        
                y1n = self.tube1.E*self.tube1.I*self.tube1.k*sin(theta(1));
                y1 = y1n/(self.tube1.E*self.tube1.I + self.tube2.E*self.tube2.I + self.tube3.E*self.tube3.I);
        
                y2n = self.tube1.E*self.tube1.I*self.tube1.k*sin(theta(1)) + self.tube2.E*self.tube2.I*self.tube2.k*sin(theta(2));
                y2 = y2n/(self.tube1.E*self.tube1.I + self.tube2.E*self.tube2.I + self.tube3.E*self.tube3.I);
        
                y3n = self.tube2.E*self.tube2.I*self.tube2.k*sin(theta(2));
                y3 = y3n/(self.tube2.E*self.tube2.I + self.tube3.E*self.tube3.I);
    
                y4n = self.tube2.E*self.tube2.I*self.tube2.k*sin(theta(2)) + self.tube3.E*self.tube3.I*self.tube3.k*sin(theta(3));
                y4 = y4n/(self.tube2.E*self.tube2.I + self.tube3.E*self.tube3.I);
    
                y5n = self.tube3.E*self.tube3.I*self.tube3.k*sin(theta(3));
                y5 = y5n/(self.tube3.E*self.tube3.I);

                chi = [x1, x2, x3, x4, x5];
                gamma = [y1, y2, y3, y4, y5];

            end

        end

        function psi = analytical_soln(self, theta, link_len, psi_prev)
            syms x;

            I = [self.tube1.I, self.tube2.I];
            E = self.tube1.E;
            G = self.tube1.G;
            J = [self.tube1.J, self.tube2.J];
            Ls = [self.tube1.l, self.tube2.l];
            k = [self.tube1.k, self.tube2.k];

            c1 = G*J(1)/Ls(1);
            c2 = G*J(2)/Ls(2);
            c3 = E*E*I(1)*I(2)*k(1)*k(2)/(E*I(1)+E*I(2));

            b1 = c3/c1;
            b2 = c1/c2;

%             disp(theta);
            
            % eqn for psi 1
%             taylor_expansion = taylor(link_len(2)*b1*sin(theta(2) + b2*theta(1) -(1+b2)*x), x)
%             psi_eqn = taylor_expansion + theta(1) - x == 0

            % equation rewritten for psi 2
            taylor_expansion = link_len(2)*b1*b2*sin(x - theta(1) + (1/b2)*(x-theta(2)))        % not this not a expansion, but the eqn itself
            psi_eqn = taylor_expansion + x - theta(2) == 0

%             psi_all = solve(psi_eqn, x,"Real",true, "PrincipalValue",true, "MaxDegree", 4); 
            
            psi_all = vpasolve(psi_eqn, x, psi_prev(2))   % numerical solver for equation and finds a solution near psi_prev(2)
            % the numerical solver also solves the taylor expansion which
            % has 1 real and 4 complex roots

            % the line below solves the non-linear eqn, but does not give explicit solutions for the taylor expansion
%             psi_all = solve(psi_eqn, x)
%             disp(psi1);
            
%             psi2 = interp1(psi_all, psi_all, psi_prev(2), 'nearest')
%             psi2 = c1/c2*(theta(1) - psi1) + theta(2);

            psi = [0, psi2]; %forcing psi 1 as zero, based on the experiment

        end

        function psi = tors_comp(self, alpha, link_len)

            options = optimoptions('fmincon', 'Display', 'off');
            psi = fmincon(@(x) self.energy_eqn(x, alpha, link_len), alpha, ...
                [], [], [], [], [], [], [], options);


        end

        function U = energy_eqn(self, psi, alpha, link_len)

            [chi, gamma] = self.in_plane_param(psi);

            x_u = 0;
            y_u = 0;
            t_u = 0;

            I = [self.tube1.I, self.tube2.I];
            E = self.tube1.E;
            G = self.tube1.G;
            J = [self.tube1.J, self.tube2.J];
            Ls = [self.tube1.l, self.tube2.l];

            k = [self.tube1.k, self.tube1.k, 0;
                 0, self.tube2.k, self.tube2.k];

            for i=1:2
                t_u = t_u + (G*J(i)/(2*Ls(i)))*(alpha(i) - psi(i))^2;
            end

            for j = 1:3
                for i =1:2

                    x_u = x_u + E*I(i)*link_len(j)/2*(chi(j) - k(i,j)*cos(psi(i)))^2;
                    y_u = y_u + E*I(i)*link_len(j)/2*(gamma(j) - k(i,j)*sin(psi(i)))^2;
                end
            end

            U = t_u+x_u+y_u;
            
        end


        function T = fkin_tors(self, q_var, psi_prev)           
            
            % First we get the link lengths
            s = get_links(self, q_var);

            % Next we get the values for theta
            theta = get_theta(self, q_var);
            self.Theta  = theta;
            
            % we calculate psi using energy minimisation
%             psi = tors_comp(self, theta, s);
            
            % we calculate psi using the analytical solution
            psi = analytical_soln(self, theta, s, psi_prev);
            self.Psi = psi;

            % Now we calculate the phi and kappa values
            [phi,K] = calculate_phi(self, psi);

            % Finally we calculate the ending transform
            T = calculate_transform(self, s, phi, K);

            if self.plotOn
                plot_tube(self, q_var, s, phi, K)
                w = waitforbuttonpress;
            end
            
        end

    end

end

