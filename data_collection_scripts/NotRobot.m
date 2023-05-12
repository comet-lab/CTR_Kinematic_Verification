classdef NotRobot < handle

    properties
%         tube1 = Tube(1.6, 2.2, 50, 100, 10, 100)
%         tube2 = Tube(3.2, 3.8, 30, 100, 10, 100)
%         tube3 = Tube(4.8, 5.4, 15, 100, 10, 100)

%         tube2 = Tube(3.8*10^-3, 5.0*10^-3, 1/11.65, 100, 50*10^-3, 1935*10^6)
%         tube1 = Tube(5.6*10^-3, 6.8*10^-3, 1/8.570, 100, 50*10^-3, 1935*10^6)

        tube3 = Tube(1.0*10^-3, 2.2*10^-3, 1/25, 247.59, 50*10^-3, 1935*10^6)
        tube2 = Tube(2.8*10^-3, 4.0*10^-3, 1/14, 116.0, 50*10^-3, 1935*10^6)
        tube1 = Tube(4.6*10^-3, 5.8*10^-3, 1/10, 91.21, 50*10^-3, 1935*10^6)

%         tube2 = Tube(2.8*10^-3, 4.0*10^-3, 1/14, 116.0, 50*10^-3, 1935*10^6)
%         tube1 = Tube(4.6*10^-3, 5.8*10^-3, 1/10, 91.21, 50*10^-3, 1935*10^6)
      
      
        n_tubes = 2;
        tubes = [];
        rho = [];
        theta = [];
        curve_flag= [];

        rot_ee = [] %end effector rotation
        pos_ee = [] % end effector translation
    end

    methods
        function self = NotRobot()
            self.tubes = [self.tube1, self. tube2, self.tube3];
        end

        function set_tubes(self, num_tubes)
            self.n_tubes = num_tubes;
        end
        
        function link_len = get_link_len(self, q_var)
            
            link_len = zeros(1,2*self.n_tubes-1);
            self.curve_flag = zeros(2*self.n_tubes-1,self.n_tubes);
            T = zeros(1,2*self.n_tubes);
            for i = 1:self.n_tubes
                self.rho(i) = (q_var(i) - q_var(1))*10^-3;
                self.theta(i) = q_var(i+self.n_tubes);
                T(i) = self.rho(i);
                T(i+self.n_tubes) = self.rho(i) + self.tubes(i).d;
            end

            sorted_len = sort(T);
            
            for i = 1:2*self.n_tubes-1
                link_len(i) = sorted_len(i+1) - sorted_len(i);
            end

            for i = 1:self.n_tubes
                for j = 1:2*self.n_tubes -1
                    ll = sorted_len(j);

                    if ll<=self.rho(i)
                        curve = 0;
                        
                    elseif ll<=self.rho(i)+self.tubes(i).d
                        curve = 1;
                    else
                        curve = -1;
                    end

                    self.curve_flag(j,i) = curve;
 
                end
            end
        end
        
        function T = fkin(self, q_var)           
            % Here q_var = [lin1, lin2, rot1, rot2] or
            % q _var = [lin1, lin2, lin3, rot1, rot2, rot3]

            % Find link lengths 
            % First define rho values in terms of joint variables. We
            % assume that T1 moves with q_var(1). Since we want rho1 to be 0,
            % we have to subtract q_var(1) from other variables. 

            s = get_link_len(self, q_var);


            phi = zeros(1, length(s));
            K = zeros(1, length(s));


            for j = 1:length(s)
                x_num = 0;
                x_den = 0;

                y_num = 0;
                y_den = 0;

                for i = 1:self.n_tubes
                    if self.curve_flag(j,i) == -1
                        continue
                    end

                    x_num = x_num + self.tubes(i).E*self.tubes(i).I*self.curve_flag(j,i)*self.tubes(i).k*cos(self.theta(i));
                    x_den = x_den + self.tubes(i).E*self.tubes(i).I;

                    y_num = y_num + self.tubes(i).E*self.tubes(i).I*self.curve_flag(j,i)*self.tubes(i).k*sin(self.theta(i));
                    y_den = y_den + self.tubes(i).E*self.tubes(i).I;

                end

                kx = x_num/x_den;
                ky = y_num/y_den;

                phi(j) = atan2(ky, kx);
                K(j) = sqrt(kx*kx + ky*ky);

            end
    
            T=[];
    
            for i =1:length(s)
                tt = [[cos(phi(i))*cos(K(i)*s(i)), -sin(phi(i)), cos(phi(i))*sin(K(i)*s(i)), cos(phi(i))*(1-cos(K(i)*s(i)))/K(i)];
                    [sin(phi(i))*cos(K(i)*s(i)), +cos(phi(i)), sin(phi(i))*sin(K(i)*s(i)), sin(phi(i))*(1-cos(K(i)*s(i)))/K(i)];
                    [-sin(K(i)*s(i)), 0, cos(K(i)*s(i)), sin(K(i)*s(i))/K(i)];
                    [0, 0, 0, 1]];
                
                T(:,:,i) = tt;
            end
        end


        
        function [chi, gamma] = in_plane_param(self, theta)

            % Now we calculate the chi and gamma values. Here we assume two
            % tubes and 3 links
        
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


        function T = fkin_tors(self, q_var)           
            % Here q_var = [lin1, lin2, rot1, rot2] or
            % q _var = [lin1, lin2, lin3, rot1, rot2, rot3]

            % Find link lengths 
            % First define rho values in terms of joint variables. We
            % assume that T1 moves with q_var(1). Since we want rho1 to be 0,
            % we have to subtract q_var(1) from other variables. 

            rho = [0, q_var(2) - q_var(1)]*10^-3;
%             rho = [0, q_var(2) - q_var(1), q_var(3) - q_var(1)];
%             d = [self.tube1.d, self.tube2.d, self.tube3.d];
            d = [self.tube1.d, self.tube2.d];

            T = [rho(1), rho(2), rho(1)+d(1), rho(2)+d(2)];
            Ts = sort(T);
            s = [Ts(2)-Ts(1), Ts(3)-Ts(2), Ts(4)-Ts(3)];



            % Now we calculate the chi and gamma values. Here we assume two
            % tubes and 3 links
        
            % Calculate chi
            theta = tors_comp(self, [q_var(3), q_var(4)], s);

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
    
            phi1 = atan2(y1, x1);
            phi2 = atan2(y2, x2);
            phi3 = atan2(y3, x3);
    
            phi = [phi1, phi2, phi3];
    
            K1 = sqrt(x1*x1 + y1*y1);
            K2 = sqrt(x2*x2 + y2*y2);
            K3 = sqrt(x3*x3 + y3*y3);
    
            K = [K1, K2, K3];
    
            T=[];
    
            for i =1:3
                tt = [[cos(phi(i))*cos(K(i)*s(i)), -sin(phi(i)), cos(phi(i))*sin(K(i)*s(i)), cos(phi(i))*(1-cos(K(i)*s(i)))/K(i)];
                    [sin(phi(i))*cos(K(i)*s(i)), +cos(phi(i)), sin(phi(i))*sin(K(i)*s(i)), sin(phi(i))*(1-cos(K(i)*s(i)))/K(i)];
                    [-sin(K(i)*s(i)), 0, cos(K(i)*s(i)), sin(K(i)*s(i))/K(i)];
                    [0, 0, 0, 1]];
                
                T(:,:,i) = tt;
            end
        end

    end
end

