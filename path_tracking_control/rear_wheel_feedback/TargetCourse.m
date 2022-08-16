classdef TargetCourse < handle
    properties
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
        end

        function disp(self)
            disp (["TargetCourse(", num2str(self.distance), ")"]);
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

        function idx = search_target_index(self, state)

            d = state.calc_distance(self.cx, self.cy);
            [self.distance, idx] = min(d);

        end
    end
end
