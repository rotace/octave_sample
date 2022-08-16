classdef TargetCourse < handle
    properties
        old_nearest_point_index;
        distance;
    end

    properties (Access = "public")
        cx;
        cy;
    end

    methods
        function self = TargetCourse(cx, cy)
            disp ("New instance created.");
            self.cx = cx;
            self.cy = cy;
            self.old_nearest_point_index = nan;
        end

        function disp(self)
            disp (["TargetCourse(", num2str(self.distance), ")"]);
        end

        function idx = search_target_index(self, state)
            global k Lfc Kp
            
            % To speed up nearest point search
            if isnan(self.old_nearest_point_index)
                % dx = state.rear_x .- cx;
                % dy = state.rear_y .- cy;
                % d = sqrt( dx.^2 .+ dy.^2 );
                d = state.calc_distance(self.cx, self.cy);
                [~, idx] = min(d);
                self.old_nearest_point_index = idx;
            else
                idx = self.old_nearest_point_index;
                distance_this = state.calc_distance(self.cx(idx), self.cy(idx));
                
                while true
                    distance_next = state.calc_distance(self.cx(idx+1), self.cy(idx+1));
                    if distance_this < distance_next
                        break;
                    end
                    if idx + 1 < length(self.cx)
                        idx += 1;
                    end
                    distance_this = distance_next;
                end
                self.distance = distance_next;
                self.old_nearest_point_index = idx;
            end
        
            Lf = k * state.v + Lfc;
        
            while Lf > state.calc_distance(self.cx(idx), self.cy(idx))
                if idx + 1 >= length(self.cx)
                    break;
                end
                idx+= 1;
            end
        end
    end
end
