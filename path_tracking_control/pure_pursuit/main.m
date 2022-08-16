% ref: https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/pure_pursuit/pure_pursuit.py
function main
    clear all

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

    % init
    state = State(x=-0.0, y=-3.0, yaw=0.0, v=0.0);
    target_course = TargetCourse(cx, cy);

    last_idx = length(cx) - 1;
    t = 0.0;
    tgt_idx = target_course.search_target_index(state);

    hax1=subplot(1,1,1);
    hold on
    plot(hax1, cx, cy, "-");

    h_tgt = plot(hax1, cx(tgt_idx), cy(tgt_idx), "rx");
    while T >= t && last_idx > tgt_idx
        % calc control input
        ai = proportional_control(target_speed, state.v);
        [di, tgt_idx] = pure_pursuit_steer_control(state, target_course, tgt_idx);
        state.update(ai, di);
        target_course.disp();

        plot(hax1, state.x, state.y, "b.")
        set(h_tgt, "xdata", cx(tgt_idx));
        set(h_tgt, "ydata", cy(tgt_idx));
        pause(0.03);

        t += dt;
    end
    
end


function a = proportional_control(target, current)
    global k Kp
    a = Kp * (target - current);
end


function [delta, idx] = pure_pursuit_steer_control(state, trajectory, idx_prev)
    global WB k Lfc
    idx = trajectory.search_target_index(state);

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

    Lf = k * state.v + Lfc;

    dif_vec = (tx - state.rear_x) + j*(ty - state.rear_y);
    yaw_vec = exp(state.yaw*j);
    alpha = arg(dif_vec/yaw_vec);
    delta = atan( (2.0 * WB * sin(alpha))/Lf );
end

