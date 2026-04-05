function build_unicycle_model(model_name, out_dir, n_agents, sim_dt, ...
                              k_heading, k_speed, max_v, max_omega)
%BUILD_UNICYCLE_MODEL  Programmatically create the Simulink co-simulation model.
%
%  build_unicycle_model(model_name, out_dir, n_agents, sim_dt, ...
%                       k_heading, k_speed, max_v, max_omega)
%
%  Creates a fixed-step Simulink model containing:
%    - One Agent_i subsystem per agent (i = 1..n_agents)
%    - Each subsystem contains:
%        * From Workspace block reading [u_x_i, u_y_i] from the MATLAB workspace
%        * Motion Controller subsystem (heading P-controller + speed shaping)
%        * Unicycle Dynamics subsystem (continuous state integrator)
%        * To Workspace block writing [x_i, y_i, theta_i, v_i, omega_i]
%
%  This model is designed to be driven step-by-step from Python via
%  matlab.engine.  Python writes u_ref_{i} to the workspace before each
%  Simulink step and reads odo_{i} back after the step.
%
%  Parameters
%  ----------
%  model_name : char   – name of the .slx file (no extension)
%  out_dir    : char   – directory where the .slx is saved
%  n_agents   : double – number of agents
%  sim_dt     : double – fixed step size [s]
%  k_heading  : double – proportional heading gain [1/rad]
%  k_speed    : double – speed feedforward scaling [-]
%  max_v      : double – linear speed saturation [unit/s]
%  max_omega  : double – angular rate saturation [rad/s]

    if nargin < 8, max_omega = 6.0;   end
    if nargin < 7, max_v     = 2.0;   end
    if nargin < 6, k_speed   = 1.0;   end
    if nargin < 5, k_heading = 4.0;   end
    if nargin < 4, sim_dt    = 0.01;  end
    if nargin < 3, n_agents  = 4;     end
    if nargin < 2, out_dir   = '.';   end
    if nargin < 1, model_name = 'unicycle_cosim'; end

    n_agents = double(n_agents);

    % ----------------------------------------------------------------
    % Create a new model (close silently if already open)
    % ----------------------------------------------------------------
    if bdIsLoaded(model_name)
        close_system(model_name, 0);
    end
    new_system(model_name);
    load_system(model_name);

    % ----------------------------------------------------------------
    % Solver settings
    % ----------------------------------------------------------------
    set_param(model_name, ...
        'SolverType',     'Fixed-step', ...
        'FixedStep',      num2str(sim_dt), ...
        'Solver',         'ode1', ...
        'StopTime',       '10', ...
        'StartTime',      '0', ...
        'SaveTime',       'off', ...
        'SaveOutput',     'off');

    % ----------------------------------------------------------------
    % Layout parameters
    % ----------------------------------------------------------------
    row_h   = 120;   % vertical spacing between agent rows
    x_left  = 60;
    y_top   = 40;

    for i = 1:n_agents
        agent_name = sprintf('Agent_%d', i);
        y_offset   = y_top + (i-1)*row_h;

        % ---- Add top-level Agent subsystem ----
        sys_path = [model_name '/' agent_name];
        add_block('built-in/Subsystem', sys_path, ...
            'Position', [x_left, y_offset, x_left+560, y_offset+90]);

        _build_agent_subsystem(sys_path, i, model_name, ...
            sim_dt, k_heading, k_speed, max_v, max_omega);
    end

    % ----------------------------------------------------------------
    % Save
    % ----------------------------------------------------------------
    slx_path = fullfile(out_dir, [model_name '.slx']);
    save_system(model_name, slx_path);
    fprintf('[build_unicycle_model] Saved: %s\n', slx_path);
end


% ====================================================================
% Internal helper: populate one Agent_i subsystem
% ====================================================================
function _build_agent_subsystem(sys, agent_id, model_name, ...
                                sim_dt, k_heading, k_speed, max_v, max_omega)
% sys         : full Simulink path to the subsystem block
% agent_id    : 1-based integer

    u_var   = sprintf('u_ref_%d',  agent_id);  % Python writes this
    odo_var = sprintf('odo_%d',    agent_id);  % Python reads this

    % ----------------------------------------------------------------
    % Delete default In/Out ports
    % ----------------------------------------------------------------
    delete_block([sys '/In1']);
    delete_block([sys '/Out1']);

    % ----------------------------------------------------------------
    % 1. From Workspace  [u_x, u_y]
    % ----------------------------------------------------------------
    fw = [sys '/FromWS'];
    add_block('simulink/Sources/From Workspace', fw, ...
        'VariableName', u_var, ...
        'SampleTime',   num2str(sim_dt), ...
        'OutputAfterFinalValue', 'Holding final value', ...
        'Position',     [30 30 130 60]);

    % ----------------------------------------------------------------
    % 2. Motion Controller subsystem
    % ----------------------------------------------------------------
    mc = [sys '/MotionCtrl'];
    add_block('built-in/Subsystem', mc, 'Position', [160 20 340 110]);
    _build_motion_ctrl(mc, k_heading, k_speed, max_v, max_omega);

    % ----------------------------------------------------------------
    % 3. Unicycle Dynamics subsystem
    % ----------------------------------------------------------------
    ud = [sys '/Unicycle'];
    add_block('built-in/Subsystem', ud, 'Position', [360 20 520 110]);
    _build_unicycle_dynamics(ud, sim_dt, agent_id);

    % ----------------------------------------------------------------
    % 4. To Workspace  [x, y, theta, v, omega]
    % ----------------------------------------------------------------
    tw = [sys '/ToWS'];
    add_block('simulink/Sinks/To Workspace', tw, ...
        'VariableName',  odo_var, ...
        'SampleTime',    num2str(sim_dt), ...
        'SaveFormat',    'Array', ...
        'MaxDataPoints', 'inf', ...
        'Position',      [550 30 650 60]);

    % ----------------------------------------------------------------
    % Wire: FromWS -> MotionCtrl (port 1 = u_ref)
    % ----------------------------------------------------------------
    add_line(sys, 'FromWS/1', 'MotionCtrl/1');

    % ----------------------------------------------------------------
    % Wire: Unicycle theta feedback -> MotionCtrl (port 2)
    % ----------------------------------------------------------------
    % We use a GoTo/From pair to avoid a visible feedback line
    % GoTo tag: theta_i
    theta_tag = sprintf('theta_%d', agent_id);

    from_blk = [sys '/From_theta'];
    add_block('simulink/Signal Routing/From', from_blk, ...
        'GotoTag',   theta_tag, ...
        'Position',  [30 80 110 100]);
    add_line(sys, 'From_theta/1', 'MotionCtrl/2');

    % ----------------------------------------------------------------
    % Wire: MotionCtrl [v, omega] -> Unicycle [v, omega]
    % ----------------------------------------------------------------
    add_line(sys, 'MotionCtrl/1', 'Unicycle/1');   % v
    add_line(sys, 'MotionCtrl/2', 'Unicycle/2');   % omega

    % ----------------------------------------------------------------
    % Wire: Unicycle [x,y,theta,v,omega] -> ToWS (via Mux)
    % ----------------------------------------------------------------
    mux = [sys '/Mux_odo'];
    add_block('simulink/Signal Routing/Mux', mux, ...
        'Inputs',   '5', ...
        'Position', [540 25 550 115]);
    add_line(sys, 'Unicycle/1', 'Mux_odo/1');   % x
    add_line(sys, 'Unicycle/2', 'Mux_odo/2');   % y
    add_line(sys, 'Unicycle/3', 'Mux_odo/3');   % theta
    add_line(sys, 'Unicycle/4', 'Mux_odo/4');   % v  (pass-through)
    add_line(sys, 'Unicycle/5', 'Mux_odo/5');   % omega (pass-through)
    add_line(sys, 'Mux_odo/1', 'ToWS/1');

    % ----------------------------------------------------------------
    % GoTo for theta (feedback to MotionCtrl)
    % ----------------------------------------------------------------
    goto_blk = [sys '/GoTo_theta'];
    add_block('simulink/Signal Routing/Goto', goto_blk, ...
        'GotoTag',  theta_tag, ...
        'Position', [555 62 635 82]);
    add_line(sys, 'Unicycle/3', 'GoTo_theta/1');
end


% ====================================================================
% Motion Controller subsystem
% ====================================================================
function _build_motion_ctrl(mc, k_heading, k_speed, max_v, max_omega)
% Ports:
%   In1 = [u_x, u_y]  (desired Cartesian velocity from planning)
%   In2 = theta        (current heading from Unicycle, via GoTo/From)
%   Out1 = v           (linear speed command)
%   Out2 = omega       (angular rate command)

    % Demux u_ref
    dmx = [mc '/Demux_u'];
    add_block('simulink/Signal Routing/Demux', dmx, ...
        'Outputs', '2', 'Position', [30 30 40 90]);

    in1 = [mc '/In1'];
    in2 = [mc '/In2'];
    add_block('built-in/Inport', in1, 'Position', [10 50  30 70]);
    add_block('built-in/Inport', in2, 'Position', [10 120 30 140], 'Port','2');

    add_line(mc, 'In1/1', 'Demux_u/1');

    % Norm of u_ref -> v_ref  (with saturation)
    nrm = [mc '/Norm_v'];
    add_block('simulink/Math Operations/Math Function', nrm, ...
        'Operator', 'magnitude^2', 'Position', [60 25 90 55]);
    sqr = [mc '/Sqrt_v'];
    add_block('simulink/Math Operations/Sqrt', sqr, 'Position', [100 25 130 55]);
    add_line(mc, 'Demux_u/1', 'Norm_v/1');   % u_x
    add_line(mc, 'Demux_u/2', 'Norm_v/2');   % u_y — Math Function 'magnitude^2' takes 2 inputs
    add_line(mc, 'Norm_v/1',  'Sqrt_v/1');

    % Scale v_ref by k_speed
    gain_v = [mc '/Gain_v'];
    add_block('simulink/Math Operations/Gain', gain_v, ...
        'Gain', num2str(k_speed), 'Position', [140 25 170 55]);
    add_line(mc, 'Sqrt_v/1', 'Gain_v/1');

    % Saturate v
    sat_v = [mc '/Sat_v'];
    add_block('simulink/Discontinuities/Saturation', sat_v, ...
        'UpperLimit', num2str(max_v), 'LowerLimit', '0', ...
        'Position', [180 25 210 55]);
    add_line(mc, 'Gain_v/1', 'Sat_v/1');

    out1 = [mc '/Out1'];
    add_block('built-in/Outport', out1, 'Position', [240 35 260 55]);
    add_line(mc, 'Sat_v/1', 'Out1/1');

    % theta_ref = atan2(u_y, u_x) via MATLAB Fcn
    atan2_blk = [mc '/atan2_blk'];
    add_block('simulink/User-Defined Functions/MATLAB Function', atan2_blk, ...
        'Position', [60 85 140 115]);
    % Set the MATLAB code inside the MATLAB Function block:
    % We use set_param to inject the function body.
    set_param(atan2_blk, 'MATLABFunctionCode', ...
        'function theta_ref = fcn(ux, uy); theta_ref = atan2(uy, ux);');
    add_line(mc, 'Demux_u/1', 'atan2_blk/1');  % ux
    add_line(mc, 'Demux_u/2', 'atan2_blk/2');  % uy

    % Heading error: e_theta = wrap(theta_ref - theta)
    sub_theta = [mc '/Sub_theta'];
    add_block('simulink/Math Operations/Sum', sub_theta, ...
        'Inputs', '+-', 'Position', [155 90 175 110]);
    add_line(mc, 'atan2_blk/1', 'Sub_theta/1');
    add_line(mc, 'In2/1',       'Sub_theta/2');

    % Angle wrap: use MATLAB Function
    wrap_blk = [mc '/WrapAngle'];
    add_block('simulink/User-Defined Functions/MATLAB Function', wrap_blk, ...
        'Position', [185 85 260 115]);
    set_param(wrap_blk, 'MATLABFunctionCode', ...
        'function e = fcn(raw); e = mod(raw + pi, 2*pi) - pi;');
    add_line(mc, 'Sub_theta/1', 'WrapAngle/1');

    % omega = k_heading * e_theta
    gain_w = [mc '/Gain_omega'];
    add_block('simulink/Math Operations/Gain', gain_w, ...
        'Gain', num2str(k_heading), 'Position', [270 90 300 110]);
    add_line(mc, 'WrapAngle/1', 'Gain_omega/1');

    % Saturate omega
    sat_w = [mc '/Sat_omega'];
    add_block('simulink/Discontinuities/Saturation', sat_w, ...
        'UpperLimit', num2str(max_omega), 'LowerLimit', num2str(-max_omega), ...
        'Position', [310 90 340 110]);
    add_line(mc, 'Gain_omega/1', 'Sat_omega/1');

    out2 = [mc '/Out2'];
    add_block('built-in/Outport', out2, 'Position', [360 95 380 115], 'Port','2');
    add_line(mc, 'Sat_omega/1', 'Out2/1');
end


% ====================================================================
% Unicycle Dynamics subsystem
% ====================================================================
function _build_unicycle_dynamics(ud, sim_dt, agent_id)
% Ports:
%   In1 = v       (linear speed)
%   In2 = omega   (angular rate)
%   Out1 = x
%   Out2 = y
%   Out3 = theta
%   Out4 = v      (pass-through for logging)
%   Out5 = omega  (pass-through for logging)
%
% Dynamics (Euler discretisation, consistent with ode1 solver):
%   x(t+dt)     = x(t)     + dt * v * cos(theta)
%   y(t+dt)     = y(t)     + dt * v * sin(theta)
%   theta(t+dt) = theta(t) + dt * omega

    in1 = [ud '/In1'];
    in2 = [ud '/In2'];
    add_block('built-in/Inport', in1, 'Position', [10 30 30 50]);
    add_block('built-in/Inport', in2, 'Position', [10 90 30 110], 'Port','2');

    % Integrator for x
    int_x = [ud '/Int_x'];
    add_block('simulink/Continuous/Integrator', int_x, ...
        'InitialCondition', '0', 'Position', [200 20 230 50]);

    % Integrator for y
    int_y = [ud '/Int_y'];
    add_block('simulink/Continuous/Integrator', int_y, ...
        'InitialCondition', '0', 'Position', [200 70 230 100]);

    % Integrator for theta
    int_th = [ud '/Int_theta'];
    add_block('simulink/Continuous/Integrator', int_th, ...
        'InitialCondition', '0', 'Position', [200 120 230 150]);

    % omega -> theta integrator
    add_line(ud, 'In2/1', 'Int_theta/1');

    % v * cos(theta)  ->  x integrator
    cos_blk = [ud '/Cos_theta'];
    add_block('simulink/Math Operations/Trigonometric Function', cos_blk, ...
        'Operator', 'cos', 'Position', [100 55 130 75]);
    mul_x = [ud '/Mul_x'];
    add_block('simulink/Math Operations/Product', mul_x, ...
        'Inputs', '**', 'Position', [150 25 180 55]);
    add_line(ud, 'Int_theta/1', 'Cos_theta/1');
    add_line(ud, 'Cos_theta/1', 'Mul_x/2');
    add_line(ud, 'In1/1',       'Mul_x/1');
    add_line(ud, 'Mul_x/1',     'Int_x/1');

    % v * sin(theta)  ->  y integrator
    sin_blk = [ud '/Sin_theta'];
    add_block('simulink/Math Operations/Trigonometric Function', sin_blk, ...
        'Operator', 'sin', 'Position', [100 105 130 125]);
    mul_y = [ud '/Mul_y'];
    add_block('simulink/Math Operations/Product', mul_y, ...
        'Inputs', '**', 'Position', [150 75 180 105]);
    add_line(ud, 'Int_theta/1', 'Sin_theta/1');
    add_line(ud, 'Sin_theta/1', 'Mul_y/2');
    add_line(ud, 'In1/1',       'Mul_y/1');
    add_line(ud, 'Mul_y/1',     'Int_y/1');

    % Outputs
    for (port_num, blk_src, label) in ...
            {1,'Int_x/1','x'; 2,'Int_y/1','y'; 3,'Int_theta/1','theta'; 4,'In1/1','v'; 5,'In2/1','omega'}
        out_blk = [ud sprintf('/Out%d', port_num)];
        add_block('built-in/Outport', out_blk, ...
            'Position', [280 10+50*(port_num-1) 310 30+50*(port_num-1)], ...
            'Port', num2str(port_num));
        add_line(ud, blk_src, sprintf('Out%d/1', port_num));
    end
end
