classdef SplineTrajectory < handle

    properties (Access = "public")
        s_;
        x_;
        y_;
        dx_;
        dy_;
        ddx_;
        ddy_;
        len_;
        last_s_;
    end

    methods
        function self = SplineTrajectory(shape="spiral")
            disp ("New instance created.");
            if shape=="spiral"
                % spiral
                tau = [0:0.5:5];
                cx = 5*cos(tau).*tau/2;
                cy = 5*sin(tau).*tau/2;
            else
                % wave
                cx = [0:5:50];
                cy = sin(cx./5.0) .* cx ./2.0;
            end

            pkg load splines
            self.s_ = [0, (cumsum(diff(cx).^2) + cumsum(diff(cy).^2)).^0.5];
            self.x_ = csapi(self.s_, cx);
            self.dx_ = fnder(self.x_);
            self.ddx_ = fnder(self.dx_);
            self.y_ = csapi(self.s_, cy);
            self.dy_ = fnder(self.y_);
            self.ddy_ = fnder(self.dy_);
            self.len_ = self.s_(end);
            self.last_s_ = -1;
        end

        function plot(self, hax)
            s = linspace(0, self.len());
            x = fnval(self.x_, s);
            y = fnval(self.y_, s);
            plot(hax, x, y, "-");
        end

        function x = cx(self, s)
            x = fnval(self.x_, s);
        end

        function y = cy(self, s)
            y = fnval(self.y_, s);
        end

        function len = len(self)
            len = self.len_;
        end

        function s = next(self, s)
            s+=1.0;
        end

        function dist = calc_distance(self, x, y, s)
            dx = x - self.cx(s);
            dy = y - self.cy(s);
            dist = sqrt( dx.^2 + dy.^2 );
        end

        function vec = calc_position_vector(self, s)
            vec = self.cx(s) + 1j*self.cy(s);
        end

        function vec = calc_tangent_vector(self, s)
            vec = fnval(self.dx_, s) + j*fnval(self.dy_, s);
            vec = vec / abs(vec);
        end
        
        function vec = calc_normal_vector(self, s)
            ddv = fnval(self.ddx_, s) + j*fnval(self.ddy_, s);
            dv = fnval(self.dx_, s) + j*fnval(self.dy_, s);
            k = (dv'*ddv - dv*ddv')/2j/abs(dv)^3;
            vec = k * ddv / abs(ddv);
        end

        function s = calc_forward_point(self, x, y, s, distance)
            while distance > self.calc_distance(x, y, s)
                if self.next(s) >= self.len()
                    break;
                end
                s = self.next(s);
            end
        end

        function s = calc_nearest_point(self, x, y)
            pkg load optim

            function [f,g] = rosenbrockwithgrad(s)
                x_ = fnval(self.x_, s);
                y_ = fnval(self.y_, s);
                f = (x_ - x)^2 + (y_ - y)^2;
                if nargout > 1
                    dx_ = fnval(self.dx_, s);
                    dy_ = fnval(self.dy_, s);
                    g = 2*dx_*(x_ - x) + 2*dy_*(y_ - y);
                end
            end

            if self.last_s_ == -1
                s = 0;
                self.last_s_ = s;
            else
                options = optimset("GradObj", "on");
                s0 = self.last_s_;
                fun = @rosenbrockwithgrad;
                [X, FVAL, CVG, OUTP] = fmincon(fun, s0, -1, s0, [], [], [], [], [], options);
                s = X;
                self.last_s_ = s;
            end
        end

    end
end
