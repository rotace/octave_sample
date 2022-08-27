function main(ttype=1, ctype=1)
    % clear all
    % ttype=1;
    % ctype=1;

    % Parameter
    global Kp dt
    Kp = 1.0; % speed proportional gain
    dt = 0.1; % [s] time tick
    
    target_speed = 10.0 / 3.6; % [m/s]

    % init
    state = State(x=-0.0, y=-3.0, yaw=0.0, v=0.0, WB=2.9);

    if     ttype==1
        trajectory = Trajectory();
    else
        trajectory = SplineTrajectory();
    end

    if     ctype==1
        controller = PurePursuitSteerController(k=0.1, Lfc=2.0);
    elseif ctype==2
        controller = LqrSteerController(Q=diag([10,0.01,1,0.01]), R=eye(1));
    else
        controller = RearWheelFeedbackSteerController(KTH=1.0, KE=0.5);
    end

    hax1=subplot(1,1,1);
    hold on
    % hax1=subplot(3,1,1);
    % hold on
    % hax2=subplot(3,1,2);
    % hold on
    % hax3=subplot(3,1,3);
    % hold on
    trajectory.plot(hax1);

    T = 100.0; % max simulation time
    S = trajectory.len();
    t = 0.0;
    s = controller.search_target_point(state, trajectory);

    h_tgt = plot(hax1, trajectory.cx(s), trajectory.cy(s), "rx");
    while T >= t && S > s
        % calc control input
        ai = accel_control(target_speed, state.v);
        [di, s] = controller.steer_control(state, trajectory);
        state.update(ai, di);

        plot(hax1, state.x, state.y, "b.")
        set(h_tgt, "xdata", trajectory.cx(s));
        set(h_tgt, "ydata", trajectory.cy(s));

        % tvec = trajectory.calc_tangent_vector(s);
        % nvec = trajectory.calc_normal_vector(s);
        % plot(hax2, real(tvec), imag(tvec), "b.")
        % plot(hax3, real(nvec), imag(nvec), "b.")

        % e = abs(trajectory.calc_position_vector(s) - state.calc_position_vector());
        % th = arg(state.calc_tangent_vector()/trajectory.calc_tangent_vector(s));
        % plot(hax2, t, e, "b.")
        % plot(hax3, t, th, "b.")

        pause(0.03);

        t+= dt;
    end

end

function a = accel_control(target, current)
    global Kp
    a = Kp * (target - current);
end


