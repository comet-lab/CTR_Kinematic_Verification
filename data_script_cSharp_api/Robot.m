classdef Robot < handle

    properties
        num_tubes = 2

        tube1 = Tube(2.792*10^-3, 3.3*10^-3, 1/17, 90*10^-3, 50*10^-3, 1935*10^6);
        tube2 = Tube(2.132*10^-3, 2.64*10^-3, 1/22, 170*10^-3, 50*10^-3, 1935*10^6);
        tube3 = Tube(1.472*10^-3, 1.98*10^-3, 1/29, 250*10^-3, 50*10^-3, 1935*10^6);


        rot_ee = [] % end effector rotation
        pos_ee = [] % end effector translation

        lls = []
        phi = []
        kappa = []
    end

    methods
        function self = Robot(num_tubes)
            self.num_tubes = num_tubes;
            
        end

        % Here we calculate the kinematics of a full CTR
        function T = fkin(self, q_var)  

            % First we get the rho and theta avlues from q_var
            rho = get_rho_values(self, q_var);
            theta = get_theta(self, q_var);


            % Next, we use rho to get the link lengths
            self.lls = get_links(self, rho);

            % Now we calculate the phi and kappa values
            [self.phi,self.kappa] = calculate_phi_and_kappa(self, theta, rho);

            % Finally we calculate the ending transform
            T = calculate_transform(self, self.lls, self.phi, self.kappa);
            

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
            % Here q_var = [lin1, lin2, rot1, rot2] or
            % q _var = [lin1, lin2, lin3, rot1, rot2, rot3]

            % First define rho values in terms of joint variables. We
            % assume that T1 moves with q_var(1). Since we want rho1 to be 0,
            % we have to subtract q_var(1) from other variables. 

            if(self.num_tubes == 2)
                rho = [0, q_var(2) - q_var(1)]*10^-3;
            else
                rho = [0, q_var(2) - q_var(1), q_var(3) - q_var(1)]*10^-3;
            end

        end

        % Function to find the links
        function s = get_links(self, rho)

            if(self.num_tubes == 2)
                d = [self.tube1.d, self.tube2.d];
                T = [rho(1), rho(2), rho(1)+d(1), rho(2)+d(2)]; 

                Ts = sort(T);
                s = [Ts(2)-Ts(1), Ts(3)-Ts(2), Ts(4)-Ts(3)];
            else
                d = [self.tube1.d, self.tube2.d, self.tube3.d];
                T = [rho(1), rho(2), rho(3), rho(1)+d(1), rho(2)+d(2), rho(3) + d(3)];

                Ts = sort(T);

                % Not sure I got this line below correct, check when doing
                % three tubes
                s = [Ts(2)-Ts(1), Ts(3)-Ts(2), Ts(4)-Ts(3), Ts(5)-Ts(4), Ts(6)-Ts(5)];
            end
        end

        % Function to get theta values
        % Returns theta (1 x j vector where j is num links)
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
                          [sin(phi(i))*cos(K(i)*s(i)), cos(phi(i)), sin(phi(i))*sin(K(i)*s(i)), sin(phi(i))*(1-cos(K(i)*s(i)))/K(i)];
                          [-sin(K(i)*s(i)), 0, cos(K(i)*s(i)), sin(K(i)*s(i))/K(i)];
                          [0, 0, 0, 1]];
                    
                    T(:,:,i) = tt;
                end
            end
        end

        
        function [chi, gamma] = in_plane_param(self, theta, rho)

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
        
                x4n = self.tube2.E*self.tube2.I*self.tube2.k*cos(theta(2)) + self.tube3.E*self.tube3.I*self.tube3.k*cos(theta(3));
                x4 = x4n/(self.tube2.E*self.tube2.I + self.tube3.E*self.tube3.I);
    
                x5n = self.tube3.E*self.tube3.I*self.tube3.k*cos(theta(3));
                x5 = x5n/(self.tube3.E*self.tube3.I);
        
        
                y1n = self.tube1.E*self.tube1.I*self.tube1.k*sin(theta(1));
                y1 = y1n/(self.tube1.E*self.tube1.I + self.tube2.E*self.tube2.I + self.tube3.E*self.tube3.I);
        
                y2n = self.tube1.E*self.tube1.I*self.tube1.k*sin(theta(1)) + self.tube2.E*self.tube2.I*self.tube2.k*sin(theta(2));
                y2 = y2n/(self.tube1.E*self.tube1.I + self.tube2.E*self.tube2.I + self.tube3.E*self.tube3.I);
        
                
                y4n = self.tube2.E*self.tube2.I*self.tube2.k*sin(theta(2)) + self.tube3.E*self.tube3.I*self.tube3.k*sin(theta(3));
                y4 = y4n/(self.tube2.E*self.tube2.I + self.tube3.E*self.tube3.I);
    
                y5n = self.tube3.E*self.tube3.I*self.tube3.k*sin(theta(3));
                y5 = y5n/(self.tube3.E*self.tube3.I);


                if (rho(3) > self.tube1.d)
                    x3n = self.tube2.E*self.tube2.I*self.tube2.k*cos(theta(2));
                    x3 = x3n/(self.tube2.E*self.tube2.I + self.tube3.E*self.tube3.I);

                    y3n = self.tube2.E*self.tube2.I*self.tube2.k*sin(theta(2));
                    y3 = y3n/(self.tube2.E*self.tube2.I + self.tube3.E*self.tube3.I);

                else
                    x3n = self.tube1.E*self.tube1.I*self.tube1.k*cos(theta(1)) + self.tube2.E*self.tube2.I*self.tube2.k*cos(theta(2)) + self.tube3.E*self.tube3.I*self.tube3.k*cos(theta(3));
                    x3 = x3n/(self.tube1.E*self.tube1.I + self.tube2.E*self.tube2.I + self.tube3.E*self.tube3.I);

                    y3n = self.tube1.E*self.tube1.I*self.tube1.k*sin(theta(1)) + self.tube2.E*self.tube2.I*self.tube2.k*sin(theta(2)) + self.tube3.E*self.tube3.I*self.tube3.k*sin(theta(3));
                    y3 = y3n/(self.tube1.E*self.tube1.I + self.tube2.E*self.tube2.I + self.tube3.E*self.tube3.I);
                end
                

                chi = [x1, x2, x3, x4, x5];
                gamma = [y1, y2, y3, y4, y5];

            end

        end

    end

end

