classdef PurePursuitSteerController < handle
    properties
        old_nearest_point_index=nan;
        idx_prev=nan;
        k; % look forward gain
        Lfc; % [m] look-ahead distance
    end
    
    methods
        function self = PurePursuitSteerController(k=0.1, Lfc=2.0)
            disp ("New instance created.");
            self.k=k;
            self.Lfc=Lfc;
        end

        function idx = search_target_index(self, state, trajectory)

            % To speed up nearest point search
            if isnan(self.old_nearest_point_index)
                % dx = state.rear_x .- cx;
                % dy = state.rear_y .- cy;
                % d = sqrt( dx.^2 .+ dy.^2 );
                d = state.calc_distance_list(trajectory, true);
                [~, idx] = min(d);
                self.old_nearest_point_index = idx;
            else
                idx = self.old_nearest_point_index;
                distance_this = state.calc_distance(trajectory, idx, true);
                
                while true
                    distance_next = state.calc_distance(trajectory, idx+1, true);
                    if distance_this < distance_next
                        break;
                    end
                    if idx + 1 < trajectory.len()
                        idx += 1;
                    end
                    distance_this = distance_next;
                end
                self.old_nearest_point_index = idx;
            end
        
            Lf = self.k * state.v + self.Lfc;
        
            while Lf > state.calc_distance(trajectory, idx, true)
                if idx + 1 >= trajectory.len()
                    break;
                end
                idx+= 1;
            end
        end

        function [delta, idx] = steer_control(self, state, trajectory)

            idx = self.search_target_index(state, trajectory);
        
            if self.idx_prev >= idx
                idx = self.idx_prev;
            end
        
            if idx < trajectory.len()
                tx = trajectory.cx(idx);
                ty = trajectory.cy(idx);
            else
                tx = trajectory.cx(trajectory.len());
                ty = trajectory.cy(trajectory.len());
                idx = trajectory.len() - 1;
            end
        
            Lf = self.k * state.v + self.Lfc;
        
            dif_vec = (tx - state.rear_x) + j*(ty - state.rear_y);
            yaw_vec = exp(state.yaw*j);
            alpha = arg(dif_vec/yaw_vec);
            delta = atan( (2.0 * state.WB * sin(alpha))/Lf );

            self.idx_prev=idx;
        end
    end
end
