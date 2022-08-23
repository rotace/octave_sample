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

        function dist = calc_distance_list(self, trajectory, is_rear_ref=false)
            if is_rear_ref
                dx = self.rear_x - trajectory.cx_;
                dy = self.rear_y - trajectory.cy_;
            else
                dx = self.x - trajectory.cx_;
                dy = self.y - trajectory.cy_;
            end
            % dist = norm([dx,dy]);
            dist = sqrt( dx.^2 + dy.^2 );

        end
        function dist = calc_distance(self, trajectory, idx, is_rear_ref=false)
            if is_rear_ref
                dx = self.rear_x - trajectory.cx(idx);
                dy = self.rear_y - trajectory.cy(idx);
            else
                dx = self.x - trajectory.cx(idx);
                dy = self.y - trajectory.cy(idx);
            end
            % dist = norm([dx,dy]);
            dist = sqrt( dx.^2 + dy.^2 );

        end
    end
end
