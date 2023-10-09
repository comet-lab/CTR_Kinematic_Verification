classdef Tube < handle

    properties
        id = 0          % inner diameter            [m]
        od = 0          % outer diameter            [m]
        r = 0           % Bending radius            [m]
        k = 0           % curvature                 [m^-1]
        l = 0           % uncut length              [m]
        d = 0           % arc length                [m]
        E = 0           % Young's modulus           [N/m^2]
        I = 0           % 2nd moment of inertia     [m^4]
        J = 0           % polar moment of inertia   [m^4]
        G = 0           % shear modulus             [N/m^2]

    end

    methods
        function self = Tube(id, od, r, l, d, E)
            self.id = id;
            self.od = od;
            self.r = r;
            self.k = 1/r;
            self.l = l;
            self.d = d;
            self.E = E;
            self.I = (pi/64)*(od^4 - id^4);
            self.J = 2*self.I;
            self.G = self.E/(2 * (1 + 0.217));
%             self.G = 3.93*10^9;

        end

        function params = get_tube_params(self)
            params = [self.id, self.od, self.r, self.k, self.d, self.E, self.I];
        end
    end
end