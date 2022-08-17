% ref: https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/lqr_steer_control/lqr_steer_control.py
function main
    clear all
    pkg load control

    % Parameter
    global Kp dt WB Q R e_prev th_prev
    Q = diag([1,1,1,1]); % LQR paramter
    R = eye(1); % LQR paramter
    Kp = 1.0; % speed proportional gain
    dt = 0.1; % [s] time tick
    WB = 2.9; % [m] wheel base vehicle
    e_prev = 0.0;
    th_prev = 0.0;
    
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
        [di, tgt_idx] = lqr_steer_control(state, target_course);
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


function [delta, idx] = lqr_steer_control(state, trajectory)
    global WB Q R dt e_prev th_prev
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
    v = state.v + 1e-7;

    A = [
        1.0,  dt, 0.0, 0.0;
        0.0, 0.0,   v, 0.0;
        0.0, 0.0, 1.0,  dt;
        0.0, 0.0, 0.0, 0.0
    ];

    B = [0.0; 0.0; 0.0; v/WB];

    X = [
        e;
        (e-e_prev)/dt;
        th;
        (th-th_prev)/dt
    ];

    [K, ~, ~] = dlqr(A, B, Q, R);

    ff = atan( WB * k );
    fb = ( -K * X );
    delta = ff + fb;

    e_prev = e;
    th_prev = th;
end