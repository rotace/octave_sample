classdef Trajectory < handle

    properties (Access = "public")
        cx;
        cy;
    end

    methods
        function self = Trajectory(cx, cy)
            disp ("New instance created.");
            self.cx = cx;
            self.cy = cy;
        end

        function vec = calc_tangent_vector(self, idx)
            assert(idx>0)
            assert(idx<=length(self.cx))

            if idx == 1
                a = idx;
                b = idx+1;
            elseif idx == length(self.cx)
                a = idx-1;
                b = idx;
            else
                a = idx-1;
                b = idx+1;
            end

            vec = (self.cx(b) - self.cx(a)) + j*(self.cy(b) - self.cy(a));
            vec = vec / abs(vec);
        end
        
        function vec = calc_normal_vector(self, idx)
            assert(idx>0)
            assert(idx<=length(self.cx))

            if idx == 1
                a = idx;
                b = idx+2;
            elseif idx == length(self.cx)
                a = idx-2;
                b = idx;
            else
                a = idx-1;
                b = idx+1;
            end

            vec = self.calc_tangent_vector(b) - self.calc_tangent_vector(a);
            dist = abs((self.cx(b) - self.cx(a)) + j*(self.cy(b) - self.cy(a)));
            vec = vec / dist;
        end

    end
end
