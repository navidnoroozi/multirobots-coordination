function build_unicycle_model(model_name, out_dir, n_agents, sim_dt, ...
    k_heading, k_speed, max_v, max_omega)
%BUILD_UNICYCLE_MODEL Programmatically create the Simulink co-simulation model.

if nargin < 8, max_omega = 6.0; end
if nargin < 7, max_v = 2.0; end
if nargin < 6, k_speed = 1.0; end
if nargin < 5, k_heading = 4.0; end
if nargin < 4, sim_dt = 0.01; end
if nargin < 3, n_agents = 4; end
if nargin < 2, out_dir = '.'; end
if nargin < 1, model_name = 'unicycle_cosim'; end

n_agents = double(n_agents);

if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
new_system(model_name);
load_system(model_name);

% --- Convert floating-point dt to exact fractional string ---
[num, den] = rat(sim_dt, 1e-12);
sim_dt_str = sprintf('%d/%d', num, den);

% Solver settings
set_param(model_name, ...
    'SolverType', 'Fixed-step', ...
    'FixedStep', sim_dt_str, ... % Exact fraction bypasses the float error
    'Solver', 'ode1', ...
    'StopTime', 'inf', ... 
    'StartTime', '0', ...
    'SaveTime', 'off', ...
    'SaveOutput', 'off', ...
    'ReturnWorkspaceOutputs', 'off');

% Layout
row_h = 120; 
x_left = 60;
y_top = 40;

for i = 1:n_agents
    agent_name = sprintf('Agent_%d', i);
    y_offset = y_top + (i-1)*row_h;

    sys_path = [model_name '/' agent_name];
    add_block('built-in/Subsystem', sys_path, ...
        'Position', [x_left, y_offset, x_left+560, y_offset+90]);

    build_agent_subsystem(sys_path, i, model_name, ...
        sim_dt_str, k_heading, k_speed, max_v, max_omega);
end

slx_path = fullfile(out_dir, [model_name '.slx']);
save_system(model_name, slx_path);
fprintf('[build_unicycle_model] Saved: %s\n', slx_path);
end

function build_agent_subsystem(sys, agent_id, model_name, ...
    sim_dt_str, k_heading, k_speed, max_v, max_omega)

    odo_var = sprintf('odo_%d', agent_id); 

    try; if get_param([sys '/In1'], 'Handle'); delete_block([sys '/In1']); end; catch; end
    try; if get_param([sys '/Out1'], 'Handle'); delete_block([sys '/Out1']); end; catch; end

    const_u = [sys '/Constant_u'];
    add_block('simulink/Sources/Constant', const_u, ...
        'Value', '[0, 0]', ...
        'VectorParams1D', 'on', ... 
        'SampleTime', sim_dt_str, ... % Exact fraction
        'Position', [30 30 130 60]);

    mc = [sys '/MotionCtrl'];
    add_block('built-in/Subsystem', mc, 'Position', [160 20 340 110]);
    build_motion_ctrl(mc, k_heading, k_speed, max_v, max_omega);

    ud = [sys '/Unicycle'];
    add_block('built-in/Subsystem', ud, 'Position', [360 20 520 110]);
    build_unicycle_dynamics(ud);

    tw = [sys '/ToWS'];
    add_block('simulink/Sinks/To Workspace', tw, ...
        'VariableName', odo_var, ...
        'SampleTime', sim_dt_str, ... % Exact fraction
        'SaveFormat', 'Array', ...
        'MaxDataPoints', 'inf', ...
        'Position', [550 30 650 60]);

    add_line(sys, 'Constant_u/1', 'MotionCtrl/1');

    theta_tag = sprintf('theta_%d', agent_id);
    from_blk = [sys '/From_theta'];
    add_block('simulink/Signal Routing/From', from_blk, ...
        'GotoTag', theta_tag, ...
        'Position', [30 80 110 100]);
    add_line(sys, 'From_theta/1', 'MotionCtrl/2');

    add_line(sys, 'MotionCtrl/1', 'Unicycle/1'); 
    add_line(sys, 'MotionCtrl/2', 'Unicycle/2'); 

    mux = [sys '/Mux_odo'];
    add_block('simulink/Signal Routing/Mux', mux, ...
        'Inputs', '5', ...
        'Position', [540 25 550 115]);
    add_line(sys, 'Unicycle/1', 'Mux_odo/1'); 
    add_line(sys, 'Unicycle/2', 'Mux_odo/2'); 
    add_line(sys, 'Unicycle/3', 'Mux_odo/3'); 
    add_line(sys, 'Unicycle/4', 'Mux_odo/4'); 
    add_line(sys, 'Unicycle/5', 'Mux_odo/5'); 
    add_line(sys, 'Mux_odo/1', 'ToWS/1');

    goto_blk = [sys '/GoTo_theta'];
    add_block('simulink/Signal Routing/Goto', goto_blk, ...
        'GotoTag', theta_tag, ...
        'Position', [555 62 635 82]);
    add_line(sys, 'Unicycle/3', 'GoTo_theta/1');
end

function build_motion_ctrl(mc, k_heading, k_speed, max_v, max_omega)
    try; delete_block([mc '/In1']); catch; end
    try; delete_block([mc '/Out1']); catch; end

    in1 = [mc '/In1'];
    in2 = [mc '/In2'];
    add_block('built-in/Inport', in1, 'Position', [20 40 50 60]);
    add_block('built-in/Inport', in2, 'Position', [20 100 50 120], 'Port','2');

    dmx_u = [mc '/Demux_u'];
    add_block('simulink/Signal Routing/Demux', dmx_u, ...
        'Outputs', '2', 'Position', [70 30 75 80]);
    add_line(mc, 'In1/1', 'Demux_u/1');
    
    mux = [mc '/Mux_in'];
    add_block('simulink/Signal Routing/Mux', mux, ...
        'Inputs', '3', 'Position', [110 30 115 130]);
    add_line(mc, 'Demux_u/1', 'Mux_in/1'); 
    add_line(mc, 'Demux_u/2', 'Mux_in/2'); 
    add_line(mc, 'In2/1', 'Mux_in/3');     

    fcn_v = [mc '/Fcn_v'];
    add_block('simulink/User-Defined Functions/Fcn', fcn_v, ...
        'Expression', 'u(1)*cos(u(3)) + u(2)*sin(u(3))', ...
        'Position', [160 40 320 70]);
    add_line(mc, 'Mux_in/1', 'Fcn_v/1');

    fcn_w = [mc '/Fcn_w'];
    add_block('simulink/User-Defined Functions/Fcn', fcn_w, ...
        'Expression', '(1/0.15) * (-u(1)*sin(u(3)) + u(2)*cos(u(3)))', ...
        'Position', [160 90 320 120]);
    add_line(mc, 'Mux_in/1', 'Fcn_w/1');

    gain_v = [mc '/Gain_v'];
    add_block('simulink/Math Operations/Gain', gain_v, ...
        'Gain', num2str(k_speed), 'Position', [360 40 400 70]);
    add_line(mc, 'Fcn_v/1', 'Gain_v/1');

    sat_v = [mc '/Sat_v'];
    add_block('simulink/Discontinuities/Saturation', sat_v, ...
        'UpperLimit', num2str(max_v), 'LowerLimit', num2str(-max_v), ...
        'Position', [440 40 480 70]);
    add_line(mc, 'Gain_v/1', 'Sat_v/1');

    sat_w = [mc '/Sat_w'];
    add_block('simulink/Discontinuities/Saturation', sat_w, ...
        'UpperLimit', num2str(max_omega), 'LowerLimit', num2str(-max_omega), ...
        'Position', [440 90 480 120]);
    add_line(mc, 'Fcn_w/1', 'Sat_w/1');

    out1 = [mc '/Out1'];
    add_block('built-in/Outport', out1, 'Position', [520 45 550 65]);
    add_line(mc, 'Sat_v/1', 'Out1/1');

    out2 = [mc '/Out2'];
    add_block('built-in/Outport', out2, 'Position', [520 95 550 115], 'Port','2');
    add_line(mc, 'Sat_w/1', 'Out2/1');
end

function build_unicycle_dynamics(ud)
    try; delete_block([ud '/In1']); catch; end
    try; delete_block([ud '/Out1']); catch; end

    in1 = [ud '/In1'];
    in2 = [ud '/In2'];
    add_block('built-in/Inport', in1, 'Position', [10 30 30 50]);
    add_block('built-in/Inport', in2, 'Position', [10 90 30 110], 'Port','2');

    int_x = [ud '/Int_x'];
    add_block('simulink/Continuous/Integrator', int_x, ...
        'InitialCondition', '0', 'Position', [200 20 230 50]);

    int_y = [ud '/Int_y'];
    add_block('simulink/Continuous/Integrator', int_y, ...
        'InitialCondition', '0', 'Position', [200 70 230 100]);

    int_th = [ud '/Int_theta'];
    add_block('simulink/Continuous/Integrator', int_th, ...
        'InitialCondition', '0', 'Position', [200 120 230 150]);

    add_line(ud, 'In2/1', 'Int_theta/1');

    cos_blk = [ud '/Cos_theta'];
    add_block('simulink/Math Operations/Trigonometric Function', cos_blk, ...
        'Operator', 'cos', 'Position', [100 55 130 75]);
    mul_x = [ud '/Mul_x'];
    add_block('simulink/Math Operations/Product', mul_x, ...
        'Inputs', '**', 'Position', [150 25 180 55]);
    add_line(ud, 'Int_theta/1', 'Cos_theta/1');
    add_line(ud, 'Cos_theta/1', 'Mul_x/2');
    add_line(ud, 'In1/1', 'Mul_x/1');
    add_line(ud, 'Mul_x/1', 'Int_x/1');

    sin_blk = [ud '/Sin_theta'];
    add_block('simulink/Math Operations/Trigonometric Function', sin_blk, ...
        'Operator', 'sin', 'Position', [100 105 130 125]);
    mul_y = [ud '/Mul_y'];
    add_block('simulink/Math Operations/Product', mul_y, ...
        'Inputs', '**', 'Position', [150 75 180 105]);
    add_line(ud, 'Int_theta/1', 'Sin_theta/1');
    add_line(ud, 'Sin_theta/1', 'Mul_y/2');
    add_line(ud, 'In1/1', 'Mul_y/1');
    add_line(ud, 'Mul_y/1', 'Int_y/1');

    for port_num = 1:5
        if port_num == 1
            blk_src = 'Int_x/1';
        elseif port_num == 2
            blk_src = 'Int_y/1';
        elseif port_num == 3
            blk_src = 'Int_theta/1';
        elseif port_num == 4
            blk_src = 'In1/1';
        else
            blk_src = 'In2/1';
        end

        out_blk = [ud sprintf('/Out%d', port_num)];
        add_block('built-in/Outport', out_blk, ...
            'Position', [280 10+50*(port_num-1) 310 30+50*(port_num-1)], ...
            'Port', num2str(port_num));
        add_line(ud, blk_src, sprintf('Out%d/1', port_num));
    end
end