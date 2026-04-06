function odo_all = cosim_step(model_name, u_ref_all, t_start, t_end, sim_dt)
%COSIM_STEP  Advance the Simulink unicycle co-simulation by one planning sub-step.
%
%  Signature
%  ---------
%    odo_all = cosim_step(model_name, u_ref_all, t_start, t_end, sim_dt)
%
%  Returns
%  -------
%    odo_all : (n_agents x 5)  [x, y, theta, v, omega] for each agent
%
%  -----------------------------------------------------------------------
%  BUGS FIXED vs. previous version
%  -----------------------------------------------------------------------
%
%  BUG 1 (FATAL – caused the timeout):
%    Original code:
%      cur_t = str2double(get_param(model_name, 'SimulationTime'));
%      if isnan(cur_t), cur_t = 0.0; end
%      if cur_t >= t_end - 1e-9, break; end
%
%    get_param(model, 'SimulationTime') returns a DOUBLE, NOT a string.
%    str2double(double_value) → NaN  (MATLAB str2double expects char/string).
%    The isnan guard then resets cur_t = 0.0 on EVERY iteration.
%    Since 0.0 < t_end is always true, the break is NEVER reached.
%    stepForward is called indefinitely until the 10-s global timeout fires.
%
%    FIX: replace time-polling with count-based stepping.
%      n_steps = max(1, round((t_end - t_start) / sim_dt))
%    Call stepForward exactly n_steps times. SimulationTime is never read.
%
%  BUG 2 (initialisation race condition):
%    Original code issued 'start' and 'pause' back-to-back without waiting
%    for the model to reach 'running' first. The 'pause' command can be
%    processed before 'start' takes full effect, so the model may run past
%    t=0 unchecked.
%
%    FIX: after 'start', poll until status is 'running' OR 'paused', then
%    issue 'pause', then poll until status is confirmed 'paused'.
%  -----------------------------------------------------------------------

    n_agents = size(u_ref_all, 1);
    odo_all  = zeros(n_agents, 5);

    % ------------------------------------------------------------------
    % 1.  Update velocity commands (Constant block parameter injection)
    % ------------------------------------------------------------------
    for i = 1:n_agents
        u_str = sprintf('[%.15g, %.15g]', ...
                        double(u_ref_all(i, 1)), double(u_ref_all(i, 2)));
        set_param(sprintf('%s/Agent_%d/Constant_u', model_name, i), 'Value', u_str);
    end

    % ------------------------------------------------------------------
    % 2.  First-call initialisation when model is 'stopped'
    % ------------------------------------------------------------------
    status = get_param(model_name, 'SimulationStatus');

    if strcmp(status, 'stopped')
        set_param(model_name, 'StopTime',  'inf');
        set_param(model_name, 'StartTime', '0.0');

        % Issue 'start' and wait until the model is actually running
        % before issuing 'pause'. Issuing both without waiting causes a
        % race: 'pause' may be processed before 'start' takes effect.
        set_param(model_name, 'SimulationCommand', 'start');

        init_deadline = tic;
        while true
            st = get_param(model_name, 'SimulationStatus');
            if strcmp(st, 'running') || strcmp(st, 'paused')
                break;
            end
            if toc(init_deadline) > 30
                error('cosim_step:InitTimeout', ...
                      'Model did not reach running state within 30 s (status: %s).', st);
            end
            pause(0.01);
        end

        % Now pause safely (model is confirmed running)
        set_param(model_name, 'SimulationCommand', 'pause');

        pause_deadline = tic;
        while true
            st = get_param(model_name, 'SimulationStatus');
            if strcmp(st, 'paused')
                break;
            end
            if toc(pause_deadline) > 15
                error('cosim_step:PauseTimeout', ...
                      'Model did not reach paused state within 15 s after start.');
            end
            pause(0.005);
        end
    end

    % ------------------------------------------------------------------
    % 3.  Count-based stepping  (FIX for Bug 1)
    %
    %     In normal operation t_end - t_start == sim_dt, so n_steps == 1.
    %     Using round() handles floating-point noise (e.g. 0.9999999 → 1).
    %     SimulationTime is NEVER read, so the str2double/NaN bug cannot
    %     occur.
    % ------------------------------------------------------------------
    n_steps = max(1, round((t_end - t_start) / sim_dt));

    for step_i = 1:n_steps

        % Guard: model must be paused before every stepForward call
        current_status = get_param(model_name, 'SimulationStatus');
        if ~strcmp(current_status, 'paused')
            error('cosim_step:NotPaused', ...
                  'Expected ''paused'' before stepForward (step %d/%d), got: ''%s''.', ...
                  step_i, n_steps, current_status);
        end

        set_param(model_name, 'SimulationCommand', 'stepForward');

        % Wait for this individual step to complete
        step_deadline = tic;
        while true
            st = get_param(model_name, 'SimulationStatus');

            if strcmp(st, 'paused')
                break;                          % step finished normally

            elseif strcmp(st, 'stopped')
                % Model stopped unexpectedly; retrieve the error message
                try
                    err_arr = sllasterror;
                    if ~isempty(err_arr)
                        msg = err_arr(end).Message;
                    else
                        msg = '(no error message available)';
                    end
                catch
                    msg = '(sllasterror unavailable)';
                end
                error('cosim_step:ModelCrashed', ...
                      'Model stopped unexpectedly on step %d/%d. Reason: %s', ...
                      step_i, n_steps, msg);
            end

            if toc(step_deadline) > 10
                error('cosim_step:StepTimeout', ...
                      'stepForward did not complete within 10 s (step %d of %d).', ...
                      step_i, n_steps);
            end
            pause(0.001);
        end
    end

    % ------------------------------------------------------------------
    % 4.  Read odometry from MATLAB base workspace
    %     To Workspace blocks write arrays with SaveFormat = 'Array'.
    %     Each row is [x, y, theta, v, omega] at one time instant.
    %     We take the last row (most recent values).
    % ------------------------------------------------------------------
    for i = 1:n_agents
        var_name = sprintf('odo_%d', i);
        try
            raw = evalin('base', var_name);
            if isempty(raw)
                continue;                       % no data yet (leave zeros)
            end
            if isstruct(raw) && isfield(raw, 'signals')
                % Legacy timeseries / structure format
                vals = raw.signals.values;
                if size(vals, 1) >= 1 && size(vals, 2) >= 5
                    odo_all(i, :) = vals(end, 1:5);
                end
            elseif isnumeric(raw) && size(raw, 2) >= 5
                % Array format (SaveFormat = 'Array')
                odo_all(i, :) = raw(end, 1:5);
            end
        catch
            % Variable not yet in workspace (first step edge case) — zeros OK
        end
    end

end
