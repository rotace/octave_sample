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

        function idx = search_target_point(self, state, trajectory)
            idx = trajectory.calc_nearest_point(state.rear_x, state.rear_y);
            Lf = self.k * state.v + self.Lfc;
            idx = trajectory.calc_forward_point(state.rear_x, state.rear_y, idx, Lf);
        end

        function [delta, idx] = steer_control(self, state, trajectory)

            idx = self.search_target_point(state, trajectory);
        
            if self.idx_prev >= idx
                idx = self.idx_prev;
            end
        
            Lf = self.k * state.v + self.Lfc;
            tx = trajectory.cx(idx);
            ty = trajectory.cy(idx);
            dif_vec = (tx - state.rear_x) + j*(ty - state.rear_y);
            yaw_vec = exp(state.yaw*j);
            alpha = arg(dif_vec/yaw_vec);
            delta = atan( (2.0 * state.WB * sin(alpha))/Lf );

            self.idx_prev=idx;
        end
    end
end
