classdef Trajectory < handle

    properties
        cx_;
        cy_;
        last_idx_=-1;
    end

    methods
        function self = Trajectory(shape="spiral")
            disp ("New instance created.");
            if shape=="spiral"
                % spiral
                tau = [0:0.02:5];
                cx = 5*cos(tau).*tau/2;
                cy = 5*sin(tau).*tau/2;
            else
                % wave
                cx = [0:0.5:50];
                cy = sin(cx./5.0) .* cx ./2.0;
            end
            self.cx_ = cx;
            self.cy_ = cy;
        end

        function plot(self, hax)
            plot(hax, self.cx_, self.cy_, "-");
        end

        function x = cx(self, idx)
            x = self.cx_(idx);
        end

        function y = cy(self, idx)
            y = self.cy_(idx);
        end

        function len = len(self)
            len = length(self.cx_);
        end

        function idx = next(self, idx)
            idx+=1;
            if idx > self.len()
                idx = self.len();
            end
        end

        function dist = calc_distance(self, x, y, idx)
            dx = x - self.cx_(idx);
            dy = y - self.cy_(idx);
            dist = sqrt( dx.^2 + dy.^2 );
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

        function idx = calc_forward_point(self, x, y, idx, distance)
            while distance > self.calc_distance(x, y, idx)
                if self.next(idx) >= self.len()
                    break;
                end
                idx = self.next(idx);
            end
        end

        function idx = calc_nearest_point(self, x, y)
            if self.last_idx_ == -1
                idx = 1;
                self.last_idx_ = idx;
            elseif self.last_idx_ > (self.len()-1)
                idx = self.len();
                self.last_idx_ = idx;
            else
                idx = self.last_idx_;
                distance_this = self.calc_distance(x, y, idx);
                
                while true
                    distance_next = self.calc_distance(x, y, self.next(idx));
                    if distance_this <= distance_next
                        break;
                    end
                    idx = self.next(idx);
                    distance_this = distance_next;
                end
                self.last_idx_ = idx;
            end
        end

    end
end
