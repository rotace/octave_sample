% ref: https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/rear_wheel_feedback/rear_wheel_feedback.py
function main
    clear all

    % Parameter
    global Kp dt WB KTH KE
    KTH = 1.0; % steering control paramter
    KE = 0.5; % steering control paramter
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
        [di, tgt_idx] = rear_wheel_feedback_steer_control(state, target_course);
        state.update(ai, di);
        target_course.disp();

        plot(hax1, state.x, state.y, "b.")
        set(h_tgt, "xdata", cx(tgt_idx));
        set(h_tgt, "ydata", cy(tgt_idx));
        pause(0.03);

        t+= dt;
    end
    
end


function a = proportional_control(target, current)
    global k Kp
    a = Kp * (target - current);
end


function [delta, idx] = rear_wheel_feedback_steer_control(state, trajectory)
    global WB KTH KE
    idx = trajectory.search_target_index(state);

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
    v = state.v;

    omega = v * k * cos(th) / (1.0 - k * e);
    omega-= KTH * abs(v) * th;
    omega-= KE * v * sin(th) * e / th;

    if th != 0.0 && omega != 0.0
        delta = atan(WB * omega / v);
    else
        delta = 0.0
    end
end