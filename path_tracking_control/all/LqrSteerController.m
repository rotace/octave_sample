classdef LqrSteerController < handle
    properties
        Q;
        R;
        e_prev=0.0;
        th_prev=0.0;
    end
    
    methods
        function self = LqrSteerController(Q=diag([1,1,1,1]), R=eye(1))
            disp ("New instance created.");
            pkg load control
            self.Q=Q;
            self.R=R;
        end

        function idx = search_target_index(self, state, trajectory)

            d = state.calc_distance_list(trajectory);
            [~, idx] = min(d);

        end

        function [delta, idx] = steer_control(self, state, trajectory)
            global dt
            idx = self.search_target_index(state, trajectory);
        
            tx = trajectory.cx(idx);
            ty = trajectory.cy(idx);
            dif_vec = (tx - state.x) + j*(ty - state.y);
            yaw_vec = exp(state.yaw*j);
            e = abs(dif_vec);
            alpha = arg(dif_vec/yaw_vec);
            if alpha > 0
                e*= -1;
            end
        
            th = arg(state.calc_tangent_vector()/trajectory.calc_tangent_vector(idx)); % calc difference yaw
            k = abs(trajectory.calc_normal_vector(idx)); % calc curvature
            v = state.v + 1e-7;
        
            A = [
                1.0,  dt, 0.0, 0.0;
                0.0, 0.0,   v, 0.0;
                0.0, 0.0, 1.0,  dt;
                0.0, 0.0, 0.0, 0.0
            ];
        
            B = [0.0; 0.0; 0.0; v/state.WB];
        
            X = [
                e;
                (e-self.e_prev)/dt;
                th;
                (th-self.th_prev)/dt
            ];
        
            [K, ~, ~] = dlqr(A, B, self.Q, self.R);
        
            ff = atan( state.WB * k );
            fb = ( -K * X );
            delta = ff + fb;
        
            self.e_prev = e;
            self.th_prev = th;
        end
    end
end
