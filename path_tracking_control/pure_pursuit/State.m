classdef State < handle
    properties (Access = "public")
        v;
        x;
        y;
        yaw;
        rear_x;
        rear_y;
    end
    
    methods
        function self = State(x, y, yaw, v)
            disp ("New instance created.");
            global WB
            self.x = x;
            self.y = y;
            self.yaw = yaw;
            self.v = v;
            self.rear_x = self.x - ((WB / 2) * cos(self.yaw));
            self.rear_y = self.y - ((WB / 2) * sin(self.yaw));
        end

        function disp(self)
            disp (["State(", num2str(self.x), ",", num2str(self.y),")"]);
        end

        function update(self, a, delta)
            global WB dt
            self.x += self.v * cos(self.yaw) * dt;
            self.y += self.v * sin(self.yaw) * dt;
            self.yaw += self.v / WB * tan(delta) * dt;
            self.v += a * dt;
            self.rear_x = self.x - ((WB / 2) * cos(self.yaw));
            self.rear_y = self.y - ((WB / 2) * sin(self.yaw));
        end

        function dist = calc_distance(self, point_x, point_y)
            dx = self.rear_x - point_x;
            dy = self.rear_y - point_y;
            % dist = norm([dx,dy]);
            dist = sqrt( dx.^2 + dy.^2 );
        end
    end
end
