classdef Trajectory < handle

    properties (Access = "public")
        cx_;
        cy_;
        p_;
        dp_;
        ddp_;
    end

    methods
        function self = Trajectory(cx, cy)
            disp ("New instance created.");
            pkg load splines
            self.cx_ = cx;
            self.cy_ = cy;
        end

        function cx = cx(self, s)
            cx = self.cx_(s);
        end

        function cy = cy(self, s)
            cy = self.cy_(s);
        end

        function len = len(self)
            len = length(self.cx_);
        end

        function vec = calc_tangent_vector(self, idx)
            assert(idx>0)
            assert(idx<=length(self.cx_))

            if idx == 1
                a = idx;
                b = idx+1;
            elseif idx == length(self.cx_)
                a = idx-1;
                b = idx;
            else
                a = idx-1;
                b = idx+1;
            end

            vec = (self.cx_(b) - self.cx_(a)) + j*(self.cy_(b) - self.cy_(a));
            vec = vec / abs(vec);
        end
        
        function vec = calc_normal_vector(self, idx)
            assert(idx>0)
            assert(idx<=length(self.cx_))

            if idx == 1
                a = idx;
                b = idx+2;
            elseif idx == length(self.cx_)
                a = idx-2;
                b = idx;
            else
                a = idx-1;
                b = idx+1;
            end

            vec = self.calc_tangent_vector(b) - self.calc_tangent_vector(a);
            dist = abs((self.cx_(b) - self.cx_(a)) + j*(self.cy_(b) - self.cy_(a)));
            vec = vec / dist;
        end

    end
end
