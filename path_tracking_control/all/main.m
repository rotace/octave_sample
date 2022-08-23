function main
    clear all

    % Parameter
    global Kp dt
    Kp = 1.0; % speed proportional gain
    dt = 0.1; % [s] time tick
    
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
    state = State(x=-0.0, y=-3.0, yaw=0.0, v=0.0, WB=2.9);
    trajectory = Trajectory(cx, cy);
    controller = PurePursuitSteerController(k=0.1, Lfc=2.0);
    % controller = LqrSteerController(Q=diag([1,1,1,1]), R=eye(1));
    % controller = RearWheelFeedbackSteerController(KTH=1.0, KE=0.5);

    last_idx = length(cx) - 1;
    t = 0.0;
    tgt_idx = controller.search_target_index(state, trajectory);

    hax1=subplot(1,1,1);
    hold on
    plot(hax1, cx, cy, "-");

    h_tgt = plot(hax1, cx(tgt_idx), cy(tgt_idx), "rx");
    while T >= t && last_idx > tgt_idx
        % calc control input
        ai = accel_control(target_speed, state.v);
        [di, tgt_idx] = controller.steer_control(state, trajectory);
        state.update(ai, di);

        plot(hax1, state.x, state.y, "b.")
        set(h_tgt, "xdata", cx(tgt_idx));
        set(h_tgt, "ydata", cy(tgt_idx));
        pause(0.03);

        t+= dt;
    end
    
end

function a = accel_control(target, current)
    global Kp
    a = Kp * (target - current);
end


