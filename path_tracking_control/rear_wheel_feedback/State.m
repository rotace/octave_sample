classdef State < handle
    properties (Access = "public")
        v;
        x;
        y;
        yaw;
    end
    
    methods
        function self = State(x, y, yaw, v)
            disp ("New instance created.");
            global WB
            self.x = x;
            self.y = y;
            self.yaw = yaw;
            self.v = v;
        end

        function disp(self)
            disp (["State(", num2str(self.x), ",", num2str(self.y),")"]);
        end

        function vec = calc_tangent_vector(self)
            vec = exp(self.yaw*j);
        end

        function update(self, a, delta)
            assert(length(a)==1)
            assert(length(delta)==1)
            global WB dt
            self.x += self.v * cos(self.yaw) * dt;
            self.y += self.v * sin(self.yaw) * dt;
            self.yaw += self.v / WB * tan(delta) * dt;
            self.v += a * dt;
        end

        function dist = calc_distance(self, point_x, point_y)
            dx = self.x - point_x;
            dy = self.y - point_y;
            % dist = norm([dx,dy]);
            dist = sqrt( dx.^2 + dy.^2 );

        end
    end
end
