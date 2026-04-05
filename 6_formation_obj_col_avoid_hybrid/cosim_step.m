function odo_all = cosim_step(model_name, u_ref_all, t_start, t_end, sim_dt)
%COSIM_STEP  Run the Simulink unicycle model from t_start to t_end.
%
%  odo_all = cosim_step(model_name, u_ref_all, t_start, t_end, sim_dt)
%
%  This helper is called by the Python bridge via matlab.engine.eval().
%  It writes u_ref_{i} workspace variables, advances the simulation by
%  one planning step, and returns the odometry matrix.
%
%  Parameters
%  ----------
%  model_name : char     – name of the loaded Simulink model
%  u_ref_all  : (N x 2)  – velocity references for all N agents
%  t_start    : double   – simulation start time for this step [s]
%  t_end      : double   – simulation end time for this step [s]
%  sim_dt     : double   – fixed step size (used for From/To Workspace)
%
%  Returns
%  -------
%  odo_all : (N x 5) matrix, each row = [x, y, theta, v, omega] for agent i

    n_agents = size(u_ref_all, 1);
    odo_all  = zeros(n_agents, 5);

    % Write u_ref to workspace
    for i = 1:n_agents
        var_name = sprintf('u_ref_%d', i);
        assignin('base', var_name, u_ref_all(i,:));
    end

    % Configure StopTime and run
    set_param(model_name, 'StopTime', num2str(t_end, '%.6f'));

    if strcmp(get_param(model_name, 'SimulationStatus'), 'stopped')
        % Fresh start
        set_param(model_name, 'SimulationCommand', 'start');
    else
        % Resume from pause
        set_param(model_name, 'SimulationCommand', 'continue');
    end

    % Poll until paused at t_end
    deadline = tic;
    while true
        status = get_param(model_name, 'SimulationStatus');
        if strcmp(status, 'stopped') || strcmp(status, 'paused')
            break;
        end
        cur_t = str2double(get_param(model_name, 'SimulationTime'));
        if cur_t >= t_end - 1e-9
            set_param(model_name, 'SimulationCommand', 'pause');
            break;
        end
        if toc(deadline) > 10
            warning('cosim_step: simulation step timed out.');
            break;
        end
        pause(0.0005);
    end

    % Read odometry from workspace
    for i = 1:n_agents
        var_name = sprintf('odo_%d', i);
        if evalin('base', sprintf('exist(''%s'', ''var'')', var_name))
            raw = evalin('base', var_name);
            % raw is written by To Workspace as (T x 5) time-series;
            % take the last row
            if size(raw, 1) >= 1
                odo_all(i,:) = raw(end,:);
            end
        end
    end
end
