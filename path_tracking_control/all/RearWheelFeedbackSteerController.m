classdef RearWheelFeedbackSteerController < handle
    properties
        KTH;
        KE;
    end
    
    methods
        function self = RearWheelFeedbackSteerController(KTH=1.0, KE=0.5)
            disp ("New instance created.");
            self.KTH=KTH;
            self.KE=KE;
        end

        function idx = search_target_point(self, state, trajectory)

            idx = trajectory.calc_nearest_point(state.x, state.y);

        end

        function [delta, idx] = steer_control(self, state, trajectory)

            idx = self.search_target_point(state, trajectory);

            dif_vec = trajectory.calc_position_vector(idx) - state.calc_position_vector();
            yaw_vec = exp(state.yaw*j);
            alpha = arg(dif_vec/yaw_vec);
            e = abs(dif_vec) * sign(-alpha);
        
            th = arg(state.calc_tangent_vector()/trajectory.calc_tangent_vector(idx)); % calc difference yaw
            k = abs(trajectory.calc_normal_vector(idx)); % calc curvature
            v = state.v;
        
            omega = v * k * cos(th) / (1.0 - k * e);
            omega-= self.KTH * abs(v) * th;
            omega-= self.KE * v * sin(th) * e / th;
        
            if th != 0.0 && omega != 0.0
                delta = atan(state.WB * omega / v);
            else
                delta = 0.0;
            end
        end
    end
end
