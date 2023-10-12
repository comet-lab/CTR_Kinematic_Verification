classdef Robot < handle

    properties
        num_tubes = 2


        rot_ee = [] % end effector rotation
        pos_ee = [] % end effector translation

        plotOn = false

        lls = []
        phi = []
        kappa = []

        tubes = []
    end

    methods
        function self = Robot(tubes, plotOn)
            self.num_tubes = size(tubes, 2);
            self.plotOn = plotOn;
            self.tubes = tubes;
        end

        % Here we calculate the kinematics of a full CTR
        function T = fkin(self, q_var)  
            % FKIN  Calculate the forward kinematics of a CTR.
            %   T = FKIN(Q_VAR) calculates the full forward kinematics of 
            %       an n-tube concentric tube robot, where Q_VAR is the set
            %       of current joint variables for the robot, taking the
            %       form [rho1 ... rhon theta1 ... thetan], where rhoi is 
            %       the translation of the ith tube and thetai is the input
            %       rotation of the ith tube. rho values are in [mm] and
            %       theta values are in [deg].
            
            % First we get the rho and theta avlues from q_var
            rho = get_rho_values(self, q_var);
            theta = get_theta(self, q_var);

            % Next, we use rho to get the link lengths
            self.lls = get_links(self, rho);

            % Now we calculate the phi and kappa values
            [self.phi,self.kappa] = calculate_phi_and_kappa(self, theta, rho);

            % Finally we calculate the ending transform
            T = calculate_transform(self, self.lls, self.phi, self.kappa);
            % disp(T)

            if self.plotOn

                disp(q_var)
                disp(self.lls)
                disp(self.phi)
                disp(self.kappa)

                plot_tube(self, q_var, self.lls, self.phi, self.kappa)

            end

        end

        % get values for plotting
        function [lls, phi, kappa] = get_lls_phi_kappa(self)
            lls = self.lls;
            phi = self.phi;
            kappa = self.kappa;
        end

        % Get rho values from joint positions
        % Return rho (1 x i vector, i is num tubes)
        function rho = get_rho_values(self, q_var)
            % GET_RHO_VALUES  Extract rho values from a vector of CTR joint
            % variables
            %   RHO = GET_RHO_VALUES(Q_VAR) extracts a vector of rho
            %   (translation) inputs for a CTR from a vector of all joint
            %   inputs. QVAR takes the form 
            %   [rho1 ... rhon theta1 ... thetan], where rhoi is the 
            %   translation of the ith tube and thetai is the input
            %   rotation of the ith tube. rho values are in [mm] and theta 
            %   values are in [deg].

            % First define rho values in terms of joint variables. We
            % assume that T1 moves with q_var(1). Since we want rho1 to be 0,
            % we have to subtract q_var(1) from other variables. 

            % Initialize a vector to hold the result
            rho = zeros([1 self.num_tubes]);

            % Fill in the vector with the values of q_var, taking care to
            % subtract the value of q_var(1) so that all translations are
            % w/r/t the first tube translation and taking care to convert
            % from mm to m
            for i=2:self.num_tubes
                rho(i) = (q_var(i) - q_var(1)) * 10^-3;
            end

            % if(self.num_tubes == 2)
            %     rho = [0, q_var(2) - q_var(1)]*10^-3;
            % else
            %     rho = [0, q_var(2) - q_var(1), q_var(3) - q_var(1)]*10^-3;
            % end

        end

        % Function to find the links
        function s = get_links(self, rho)
            % GET_LINKS Get the length of each link (in order) in a CTR
            %   S = GET_LINKS(RHO) calculates the length of each link of a
            %   CTR in order given a vector RHO of input translations (in
            %   mm). S will take shape [1, 2i-1], where i is the number of
            %   tubes in the CTR.

            % Initialize a vector to hold the transition points
            T = zeros([1, 2*self.num_tubes]);
            % Initialize a vector to hold the resulting link lengths
            s = zeros([1, 2*self.num_tubes-1]);

            % Each tube has 2 transition points
            for i=1:self.num_tubes
                
                % One transition point occurs when the straight section of
                % the tube ends and the curved section begins. Because we
                % take the "zero" or "home" configuration to be when all
                % straight sections end at the same point, this transition
                % point occurs at zero plus whatever translation input (or
                % rho) is applied to the tube.
                T(i) = rho(i);

                % The second and final transition point occurs when the
                % tube ends. The distance between this transition point and
                % the previous transition point is always d, or the arc
                % length of the tube's curved section.
                T(i+self.num_tubes) = rho(i) + self.tubes(i).d;
            end

            % Now that we have all the transition points, sort them from
            % low to high.
            T_sorted = sort(T);

            % The link lengths are then the distances between transition
            % points
            for i=1:2*self.num_tubes-1
                s(i) = T_sorted(i+1) - T_sorted(i);
            end

            % if(self.num_tubes == 2)
            %     d = [self.tubes(1).d, self.tubes(2).d];
            %     T = [rho(1), rho(2), rho(1)+d(1), rho(2)+d(2)]; 
            % 
            %     Ts = sort(T);
            %     s = [Ts(2)-Ts(1), Ts(3)-Ts(2), Ts(4)-Ts(3)];
            % else
            %     d = [self.tubes(1).d, self.tubes(2).d, self.tubes(3).d];
            %     T = [rho(1), rho(2), rho(3), rho(1)+d(1), rho(2)+d(2), rho(3) + d(3)];
            % 
            %     Ts = sort(T);
            % 
            %     % Not sure I got this line below correct, check when doing
            %     % three tubes
            %     s = [Ts(2)-Ts(1), Ts(3)-Ts(2), Ts(4)-Ts(3), Ts(5)-Ts(4), Ts(6)-Ts(5)];
            % end
        end

        % Function to get theta values
        % Returns theta (1 x i vector where i is num tubes)
        function theta = get_theta(self, q_var)
            % GET_THETA_VALUES  Extract thetavalues from a vector of CTR 
            % joint variables
            %   THETA = GET_THETA_VALUES(Q_VAR) extracts a vector of theta
            %   (rotation) inputs for a CTR from a vector of all joint
            %   inputs. QVAR takes the form 
            %   [rho1 ... rhon theta1 ... thetan], where rhoi is the 
            %   translation of the ith tube and thetai is the input
            %   rotation of the ith tube. rho values are in [mm] and theta 
            %   values are in [deg].
                
            % Initialize a vector to hold the result
            theta = zeros([1 self.num_tubes]);

            for i=1:self.num_tubes
                theta(i) = deg2rad(q_var(i+self.num_tubes));
            end
            
            % if (self.num_tubes == 2)
            %     theta = [deg2rad(q_var(3)), deg2rad(q_var(4))];
            % else
            %     theta = [deg2rad(q_var(4)), deg2rad(q_var(5)), deg2rad(q_var(6))];
            % end
        end


        % Function to calcualte phi values for two or three tube
        % configurations

        % Should return phi (1 x j vector, where j is num links)
        % and K (1 x j vector)
        function [phi,K] = calculate_phi_and_kappa(self, theta, rho)

             [chi, gamma] = in_plane_param(self, theta, rho);
             if(self.num_tubes == 2)
        
                phi1_0 = atan2(gamma(1), chi(1));
                phi2_0 = atan2(gamma(2), chi(2));
                phi3_0 = atan2(gamma(3), chi(3));
        
                phi = [phi1_0, phi2_0-phi1_0, phi3_0-phi2_0];

                K1 = sqrt(chi(1)*chi(1) + gamma(1)*gamma(1));
                K2 = sqrt(chi(2)*chi(2) + gamma(2)*gamma(2));
                K3 = sqrt(chi(3)*chi(3) + gamma(3)*gamma(3));
        
                K = [K1, K2, K3];
            else

                phi1_0 = atan2(gamma(1), chi(1));
                phi2_0 = atan2(gamma(2), chi(2));
                phi3_0 = atan2(gamma(3), chi(3));
                phi4_0 = atan2(gamma(4), chi(4));
                phi5_0 = atan2(gamma(5), chi(5));
                
                phi = [phi1_0, phi2_0 - phi1_0, phi3_0 - phi2_0, phi4_0 - phi3_0, phi5_0 -phi4_0];

                K1 = sqrt(chi(1)*chi(1) + gamma(1)*gamma(1));
                K2 = sqrt(chi(2)*chi(2) + gamma(2)*gamma(2));
                K3 = sqrt(chi(3)*chi(3) + gamma(3)*gamma(3));
                K4 = sqrt(chi(4)*chi(4) + gamma(4)*gamma(4));
                K5 = sqrt(chi(5)*chi(5) + gamma(5)*gamma(5));
        
                K = [K1, K2, K3, K4, K5];
            end
        end

        function T = calculate_transform(self, s, phi, K)
            T=eye(4);

%             if(self.num_tubes ==2)
            for i =1:(self.num_tubes*2 - 1)
                tt = [[cos(phi(i))*cos(K(i)*s(i)), -sin(phi(i)), cos(phi(i))*sin(K(i)*s(i)), cos(phi(i))*(1-cos(K(i)*s(i)))/K(i)];
                    [sin(phi(i))*cos(K(i)*s(i)), +cos(phi(i)), sin(phi(i))*sin(K(i)*s(i)), sin(phi(i))*(1-cos(K(i)*s(i)))/K(i)];
                    [-sin(K(i)*s(i)), 0, cos(K(i)*s(i)), sin(K(i)*s(i))/K(i)];
                    [0, 0, 0, 1]];
               
                
                T(:,:) = T*tt;
            end
%             else
%                 for i =1:5
%                     tt = [[cos(phi(i))*cos(K(i)*s(i)), -sin(phi(i)), cos(phi(i))*sin(K(i)*s(i)), cos(phi(i))*(1-cos(K(i)*s(i)))/K(i)];
%                         [sin(phi(i))*cos(K(i)*s(i)), +cos(phi(i)), sin(phi(i))*sin(K(i)*s(i)), sin(phi(i))*(1-cos(K(i)*s(i)))/K(i)];
%                         [-sin(K(i)*s(i)), 0, cos(K(i)*s(i)), sin(K(i)*s(i))/K(i)];
%                         [0, 0, 0, 1]];
%                     
%                     T(:,:) = T*tt;
%                 end
%             end
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
%                 points = [pts_per_seg 2*pts_per_seg - pts_per_seg* (s(2)/self.tubes(1).d)];
                points = [pts_per_seg 2*pts_per_seg 3*pts_per_seg];
                try
                    draw_ctcr(g,points,[self.tubes(1).od self.tubes(1).od self.tubes(2).od]);
                catch
                    disp("Invalid position to plot")
                end
            else
                translation_frac = [q_var(1)/self.tubes(1).d q_var(2)/self.tubes(2).d; q_var(3)/self.tubes(3).d];

                points = int16(translation_frac * pts_per_seg);

                draw_ctcr(g,points,[self.tubes(1).od self.tubes(2).od]);
            end
        end

        
        function [chi, gamma] = in_plane_param(self, theta, rho)

            if(self.num_tubes == 2)
                x1n = self.tubes(1).E*self.tubes(1).I*self.tubes(1).k*cos(theta(1));
                x1 = x1n/(self.tubes(1).E*self.tubes(1).I + self.tubes(2).E*self.tubes(2).I);
        
                x2n = self.tubes(1).E*self.tubes(1).I*self.tubes(1).k*cos(theta(1)) + self.tubes(2).E*self.tubes(2).I*self.tubes(2).k*cos(theta(2));
                x2 = x2n/(self.tubes(1).E*self.tubes(1).I + self.tubes(2).E*self.tubes(2).I);
        
                x3n = self.tubes(2).E*self.tubes(2).I*self.tubes(2).k*cos(theta(2));
                x3 = x3n/(self.tubes(2).E*self.tubes(2).I);
        
        
                y1n = self.tubes(1).E*self.tubes(1).I*self.tubes(1).k*sin(theta(1));
                y1 = y1n/(self.tubes(1).E*self.tubes(1).I + self.tubes(2).E*self.tubes(2).I);
        
                y2n = self.tubes(1).E*self.tubes(1).I*self.tubes(1).k*sin(theta(1)) + self.tubes(2).E*self.tubes(2).I*self.tubes(2).k*sin(theta(2));
                y2 = y2n/(self.tubes(1).E*self.tubes(1).I + self.tubes(2).E*self.tubes(2).I);
        
                y3n = self.tubes(2).E*self.tubes(2).I*self.tubes(2).k*sin(theta(2));
                y3 = y3n/(self.tubes(2).E*self.tubes(2).I);
    
                chi = [x1, x2, x3];
                gamma = [y1, y2, y3];
            
            
            else

                x1n = self.tubes(1).E*self.tubes(1).I*self.tubes(1).k*cos(theta(1));
                x1 = x1n/(self.tubes(1).E*self.tubes(1).I + self.tubes(2).E*self.tubes(2).I + self.tubes(3).E*self.tubes(3).I);
        
                x2n = self.tubes(1).E*self.tubes(1).I*self.tubes(1).k*cos(theta(1)) + self.tubes(2).E*self.tubes(2).I*self.tubes(2).k*cos(theta(2));
                x2 = x2n/(self.tubes(1).E*self.tubes(1).I + self.tubes(2).E*self.tubes(2).I + self.tubes(3).E*self.tubes(3).I);
        
                x4n = self.tubes(2).E*self.tubes(2).I*self.tubes(2).k*cos(theta(2)) + self.tubes(3).E*self.tubes(3).I*self.tubes(3).k*cos(theta(3));
                x4 = x4n/(self.tubes(2).E*self.tubes(2).I + self.tubes(3).E*self.tubes(3).I);
    
                x5n = self.tubes(3).E*self.tubes(3).I*self.tubes(3).k*cos(theta(3));
                x5 = x5n/(self.tubes(3).E*self.tubes(3).I);
        
        
                y1n = self.tubes(1).E*self.tubes(1).I*self.tubes(1).k*sin(theta(1));
                y1 = y1n/(self.tubes(1).E*self.tubes(1).I + self.tubes(2).E*self.tubes(2).I + self.tubes(3).E*self.tubes(3).I);
        
                y2n = self.tubes(1).E*self.tubes(1).I*self.tubes(1).k*sin(theta(1)) + self.tubes(2).E*self.tubes(2).I*self.tubes(2).k*sin(theta(2));
                y2 = y2n/(self.tubes(1).E*self.tubes(1).I + self.tubes(2).E*self.tubes(2).I + self.tubes(3).E*self.tubes(3).I);
        
                y4n = self.tubes(2).E*self.tubes(2).I*self.tubes(2).k*sin(theta(2)) + self.tubes(3).E*self.tubes(3).I*self.tubes(3).k*sin(theta(3));
                y4 = y4n/(self.tubes(2).E*self.tubes(2).I + self.tubes(3).E*self.tubes(3).I);
    
                y5n = self.tubes(3).E*self.tubes(3).I*self.tubes(3).k*sin(theta(3));
                y5 = y5n/(self.tubes(3).E*self.tubes(3).I);


                if (rho(3) > self.tubes(1).d)
                    x3n = self.tubes(2).E*self.tubes(2).I*self.tubes(2).k*cos(theta(2));
                    x3 = x3n/(self.tubes(2).E*self.tubes(2).I + self.tubes(3).E*self.tubes(3).I);

                    y3n = self.tubes(2).E*self.tubes(2).I*self.tubes(2).k*sin(theta(2));
                    y3 = y3n/(self.tubes(2).E*self.tubes(2).I + self.tubes(3).E*self.tubes(3).I);

                else
                    x3n = self.tubes(1).E*self.tubes(1).I*self.tubes(1).k*cos(theta(1)) + self.tubes(2).E*self.tubes(2).I*self.tubes(2).k*cos(theta(2)) + self.tubes(3).E*self.tubes(3).I*self.tubes(3).k*cos(theta(3));
                    x3 = x3n/(self.tubes(1).E*self.tubes(1).I + self.tubes(2).E*self.tubes(2).I + self.tubes(3).E*self.tubes(3).I);

                    y3n = self.tubes(1).E*self.tubes(1).I*self.tubes(1).k*sin(theta(1)) + self.tubes(2).E*self.tubes(2).I*self.tubes(2).k*sin(theta(2)) + self.tubes(3).E*self.tubes(3).I*self.tubes(3).k*sin(theta(3));
                    y3 = y3n/(self.tubes(1).E*self.tubes(1).I + self.tubes(2).E*self.tubes(2).I + self.tubes(3).E*self.tubes(3).I);
                end

                chi = [x1, x2, x3, x4, x5];
                gamma = [y1, y2, y3, y4, y5];

            end

        end

        function psi = analytical_soln(self, theta, link_len, psi_prev)
            syms x;

            I = [self.tubes(1).I, self.tubes(2).I];
            E = self.tubes(1).E;
            G = self.tubes(1).G;
            J = [self.tubes(1).J, self.tubes(2).J];
            Ls = [self.tubes(1).l, self.tubes(2).l];
            k = [self.tubes(1).k, self.tubes(2).k];

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
            taylor_expansion = link_len(2)*b1*b2*sin(x - theta(1) + (1/b2)*(x-theta(2)));        % not this not a expansion, but the eqn itself
            psi_eqn = taylor_expansion + x - theta(2) == 0;

%             psi_all = solve(psi_eqn, x,"Real",true, "PrincipalValue",true, "MaxDegree", 4); 
            
            psi2 = vpasolve(psi_eqn, x, psi_prev(2));   % numerical solver for equation and finds a solution near psi_prev(2)
            % the numerical solver also solves the taylor expansion which
            % has 1 real and 4 complex roots

            % the line below solves the non-linear eqn, but does not give explicit solutions for the taylor expansion
%             psi_all = solve(psi_eqn, x)
%             disp(psi1);
            
%             psi2 = interp1(psi_all, psi_all, psi_prev(2), 'nearest')
%             psi2 = c1/c2*(theta(1) - psi1) + theta(2);

            psi = [0, double(psi2)]; %forcing psi 1 as zero, based on the experiment

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

            I = [self.tubes(1).I, self.tubes(2).I];
            E = self.tubes(1).E;
            G = self.tubes(1).G;
            J = [self.tubes(1).J, self.tubes(2).J];
            Ls = [self.tubes(1).l, self.tubes(2).l];

            k = [self.tubes(1).k, self.tubes(1).k, 0;
                 0, self.tubes(2).k, self.tubes(2).k];

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
            psi = tors_comp(self, theta, s);
            
            % we calculate psi using the analytical solution
%             psi = analytical_soln(self, theta, s, psi_prev);
            self.Psi = psi;

            % Now we calculate the phi and kappa values
            [phi,K] = calculate_phi_and_kappa(self, psi);

            % Finally we calculate the ending transform
            T = calculate_transform(self, s, phi, K);

            if self.plotOn
                plot_tube(self, q_var, s, phi, K)
                w = waitforbuttonpress;
            end
            
        end

    end

end

