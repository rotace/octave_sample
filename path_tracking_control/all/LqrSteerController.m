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

        function idx = search_target_point(self, state, trajectory)

            idx = trajectory.calc_nearest_point(state.x, state.y);

        end

        function [delta, idx] = steer_control(self, state, trajectory)
            global dt
            idx = self.search_target_point(state, trajectory);
        
            dif_vec = trajectory.calc_position_vector(idx) - state.calc_position_vector();
            yaw_vec = exp(state.yaw*j);
            e = abs(dif_vec);
            alpha = arg(dif_vec/yaw_vec);
            if alpha > 0
                e*= -1;
            end
        
            th = arg(state.calc_tangent_vector()/trajectory.calc_tangent_vector(idx)); % calc difference yaw
            k = abs(trajectory.calc_normal_vector(idx)); % calc curvature
            v = state.v;
        
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
            
            if abs(v) > 0.001
                [K, ~, ~] = dlqr(A, B, self.Q, self.R);
            else
                % dlqr unstable when v~= 0
                K = [0 0 0 0];
            end
        
            ff = atan( state.WB * k );
            fb = ( -K * X );
            delta = ff + fb;
        
            self.e_prev = e;
            self.th_prev = th;
        end
    end
end
