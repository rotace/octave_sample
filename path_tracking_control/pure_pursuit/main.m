% ref: https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/pure_pursuit/pure_pursuit.py
function main

    % Parameter
    global k Lfc Kp dt WB
    k = 0.1; % look forward gain
    Lfc = 2.0; % [m] look-ahead distance
    Kp = 1.0; % speed proportional gain
    dt = 0.1; % [s] time tick
    WB = 2.9; % [m] wheel base vehicle
    
    % wave
    cx = [0:0.5:50];
    cy = sin(cx./5.0) .* cx ./2.0;

    % spiral
    tau = [0:0.02:5];
    cx = 5*cos(tau).*tau/2;
    cy = 5*sin(tau).*tau/2;

    target_speed = 10.0 / 3.6; % [m/s]

    T = 100.0; % max simulation time

    % init state
    state = state_init(x=-0.0, y=-3.0, yaw=0.0, v=0.0);

    last_idx = length(cx) - 1;
    t = 0.0;
    target_course = target_course_init(cx, cy);
    [tgt_idx, ~] = target_course_search_target_index(target_course, state);

    hax1=subplot(1,1,1);
    hold on
    plot(hax1, cx, cy, "-");

    h_tgt = plot(hax1, cx(tgt_idx), cy(tgt_idx), "rx");
    while T >= t && last_idx > tgt_idx
        % calc control input
        ai = proportional_control(target_speed, state.v);
        [di, tgt_idx] = pure_pursuit_steer_control(state, target_course, tgt_idx);
        state = state_update(state, ai, di);

        plot(hax1, state.x, state.y, "b.")
        set(h_tgt, "xdata", cx(tgt_idx));
        set(h_tgt, "ydata", cy(tgt_idx));
        pause(0.03);

        t += dt;
    end

    pause;
    close;
    
end


function a = proportional_control(target, current)
    global k Kp
    a = Kp * (target - current);
end


%%%%%%%%%%%%%% State %%%%%%%%%%%%%%%
function self = state_init(x=0.0, y=0.0, yaw=0.0, v=0.0)
    global WB
    self = struct();
    self.x = x;
    self.y = y;
    self.yaw = yaw;
    self.v = v;
    self.rear_x = self.x - ((WB / 2) * cos(self.yaw));
    self.rear_y = self.y - ((WB / 2) * sin(self.yaw));
end

function self = state_update(self, a, delta)
    global WB dt
    self.x += self.v * cos(self.yaw) * dt;
    self.y += self.v * sin(self.yaw) * dt;
    self.yaw += self.v / WB * tan(delta) * dt;
    self.v += a * dt;
    self.rear_x = self.x - ((WB / 2) * cos(self.yaw));
    self.rear_y = self.y - ((WB / 2) * sin(self.yaw));
end

function dist = state_calc_distance(self, point_x, point_y)
    dx = self.rear_x .- point_x;
    dy = self.rear_y .- point_y;
    % dist = norm([dx,dy]);
    dist = sqrt( dx.^2 .+ dy.^2 );
end

%%%%%%%%%%%%%% Target Course %%%%%%%%%%%%%%%
function self = target_course_init(cx, cy)
    self.cx = cx;
    self.cy = cy;
    self.old_nearest_point_index = nan;
end

function [idx, Lf] = target_course_search_target_index(self, state)
    global k Lfc Kp
    
    % To speed up nearest point search
    if isnan(self.old_nearest_point_index)
        % dx = state.rear_x .- cx;
        % dy = state.rear_y .- cy;
        % d = sqrt( dx.^2 .+ dy.^2 );
        d = state_calc_distance(state, self.cx, self.cy);
        [~, idx] = min(d);
        self.old_nearest_point_index = idx;
    else
        idx = self.old_nearest_point_index;
        distance_this = state_calc_distance(self.cx(idx), self.cy(idx));
        
        while true
            distance_next = state_calc_distance(state, self.cx(idx+1), self.cy(idx+1))
            if distance_this < distance_next
                break;
            end
            if idx + 1 < length(self.cx)
                idx += 1;
            end
            distance_this = distance_next;
        end
        self.old_nearest_point_index = idx;
    end

    Lf = k * state.v + Lfc;

    while Lf > state_calc_distance(state, self.cx(idx), self.cy(idx))
        if idx + 1 >= length(self.cx)
            break;
        end
        idx += 1;
    end
end

function [delta, idx] = pure_pursuit_steer_control(state, trajectory, idx_prev)
    global WB
    [idx, Lf] = target_course_search_target_index(trajectory, state);

    if idx_prev >= idx
        idx = idx_prev;
    end

    if idx < length(trajectory.cx)
        tx = trajectory.cx(idx);
        ty = trajectory.cy(idx);
    else
        tx = trajectory.cx(end);
        ty = trajectory.cy(end);
        idx = length(trajectory.cx) - 1;
    end

    dif_vec = (tx - state.rear_x) + j*(ty - state.rear_y);
    yaw_vec = exp(state.yaw*j);
    alpha = arg(dif_vec/yaw_vec);
    delta = atan( (2.0 * WB * sin(alpha))/Lf );
end

