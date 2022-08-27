classdef State < handle
    properties (Access = "public")
        v;
        x;
        y;
        yaw;
        rear_x;
        rear_y;
        WB;
    end
    
    methods
        function self = State(x, y, yaw, v, WB)
            disp ("New instance created.");
            self.WB = WB;
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

        function vec = calc_rear_position_vector(self)
            vec = self.rear_x + 1j*self.rear_y;
        end

        function vec = calc_position_vector(self)
            vec = self.x + 1j*self.y;
        end

        function vec = calc_tangent_vector(self)
            vec = exp(self.yaw*j);
        end

        function update(self, a, delta)
            assert(length(a)==1)
            assert(length(delta)==1)
            global dt
            self.x += self.v * cos(self.yaw) * dt;
            self.y += self.v * sin(self.yaw) * dt;
            self.yaw += self.v / self.WB * tan(delta) * dt;
            self.v += a * dt;
            self.rear_x = self.x - ((self.WB / 2) * cos(self.yaw));
            self.rear_y = self.y - ((self.WB / 2) * sin(self.yaw));
        end
    end
end
