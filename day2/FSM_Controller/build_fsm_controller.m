function build_fsm_controller()
%BUILD_FSM_CONTROLLER Generate a fuzzy sliding-mode controller model.
%
% Sliding surface:
%   s = G*xhat
%
% Control law:
%   u_raw = sum_j theta_j * (Kj*xhat + u_sm)
%
% The T-S membership functions reuse the wind plant premise variable xhat(6).

rootDir = fileparts(mfilename('fullpath'));
if isempty(rootDir)
    rootDir = pwd;
end
parentDir = fileparts(rootDir);
lfcDir = fullfile(parentDir, 'LFC');
addpath(parentDir);
if exist(lfcDir, 'dir')
    addpath(lfcDir);
end

wind_p = wind_plant_parameters();

ctrl_h = 0.01;
ctrl_Tstop = 50;

% Placeholder gains. Replace with LMI-designed controller gains when ready.
ctrl_G = [1 0.2 0.2 0.1 0.05 0.05];
ctrl_K1 = [-0.8 -0.3 -0.4 -0.15 -0.05 -0.05];
ctrl_K2 = [-0.8 -0.3 -0.4 -0.15 -0.05 -0.05];
ctrl_eta = 0.1;
ctrl_boundary = 0.01;

assignin('base', 'wind_p', wind_p);
assignin('base', 'ctrl_h', ctrl_h);
assignin('base', 'ctrl_Tstop', ctrl_Tstop);
assignin('base', 'ctrl_G', ctrl_G);
assignin('base', 'ctrl_K1', ctrl_K1);
assignin('base', 'ctrl_K2', ctrl_K2);
assignin('base', 'ctrl_eta', ctrl_eta);
assignin('base', 'ctrl_boundary', ctrl_boundary);

mdl = 'fsm_controller';
sub = [mdl '/Fuzzy_Sliding_Mode_Controller'];
outFile = fullfile(rootDir, [mdl '.slx']);

if bdIsLoaded(mdl)
    close_system(mdl, 0);
end

new_system(mdl);
open_system(mdl);
set_param(mdl, ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'ode4', ...
    'FixedStep', 'ctrl_h', ...
    'StopTime', 'ctrl_Tstop', ...
    'SaveFormat', 'Dataset');

add_block('simulink/Ports & Subsystems/Subsystem', sub, ...
    'Position', [185 105 455 235]);
prepare_subsystem(sub);
build_controller_subsystem(sub);

add_block('simulink/Sources/In1', [mdl '/x_hat'], ...
    'Position', [45 155 75 169]);
add_block('simulink/Sinks/Out1', [mdl '/u_raw'], ...
    'Position', [565 155 595 169]);

add_line(mdl, 'x_hat/1', 'Fuzzy_Sliding_Mode_Controller/1', 'autorouting', 'on');
add_line(mdl, 'Fuzzy_Sliding_Mode_Controller/1', 'u_raw/1', 'autorouting', 'on');

Simulink.BlockDiagram.arrangeSystem(mdl);
save_system(mdl, outFile);
fprintf('Created %s\n', outFile);
fprintf('Workspace variables initialized: wind_p, ctrl_h, ctrl_Tstop, ctrl_G, ctrl_K1, ctrl_K2, ctrl_eta, ctrl_boundary.\n');
end

function prepare_subsystem(sub)
try
    delete_line(sub, 'In1/1', 'Out1/1');
catch
end
try
    delete_block([sub '/In1']);
catch
end
try
    delete_block([sub '/Out1']);
catch
end
end

function build_controller_subsystem(sub)
add_block('simulink/Sources/In1', [sub '/x_hat'], ...
    'Position', [35 155 65 169]);
add_block('simulink/Sinks/Out1', [sub '/u_raw'], ...
    'Position', [760 155 790 169]);

% Sliding surface s = G*xhat.
add_block('simulink/Math Operations/Gain', [sub '/sliding_surface_G_xhat'], ...
    'Gain', 'ctrl_G', ...
    'Multiplication', 'Matrix(K*u)', ...
    'Position', [120 60 235 100]);

% Basic sliding-mode term u_sm = -eta * sat(s / boundary).
add_block('simulink/Math Operations/Gain', [sub '/s_over_boundary'], ...
    'Gain', '1/ctrl_boundary', ...
    'Position', [280 60 385 100]);
add_block('simulink/Discontinuities/Saturation', [sub '/sat_s'], ...
    'LowerLimit', '-1', ...
    'UpperLimit', '1', ...
    'Position', [425 60 490 100]);
add_block('simulink/Math Operations/Gain', [sub '/u_sm'], ...
    'Gain', '-ctrl_eta', ...
    'Position', [530 60 615 100]);

add_line(sub, 'x_hat/1', 'sliding_surface_G_xhat/1', 'autorouting', 'on');
add_line(sub, 'sliding_surface_G_xhat/1', 's_over_boundary/1', 'autorouting', 'on');
add_line(sub, 's_over_boundary/1', 'sat_s/1', 'autorouting', 'on');
add_line(sub, 'sat_s/1', 'u_sm/1', 'autorouting', 'on');

build_membership(sub);

% Fuzzy feedback branches K1*xhat and K2*xhat.
add_block('simulink/Math Operations/Gain', [sub '/K1_xhat'], ...
    'Gain', 'ctrl_K1', ...
    'Multiplication', 'Matrix(K*u)', ...
    'Position', [120 200 235 240]);
add_block('simulink/Math Operations/Gain', [sub '/K2_xhat'], ...
    'Gain', 'ctrl_K2', ...
    'Multiplication', 'Matrix(K*u)', ...
    'Position', [120 300 235 340]);
add_block('simulink/Math Operations/Sum', [sub '/rule1_control'], ...
    'Inputs', '++', ...
    'Position', [310 200 340 230]);
add_block('simulink/Math Operations/Sum', [sub '/rule2_control'], ...
    'Inputs', '++', ...
    'Position', [310 300 340 330]);

add_line(sub, 'x_hat/1', 'K1_xhat/1', 'autorouting', 'on');
add_line(sub, 'x_hat/1', 'K2_xhat/1', 'autorouting', 'on');
add_line(sub, 'K1_xhat/1', 'rule1_control/1', 'autorouting', 'on');
add_line(sub, 'u_sm/1', 'rule1_control/2', 'autorouting', 'on');
add_line(sub, 'K2_xhat/1', 'rule2_control/1', 'autorouting', 'on');
add_line(sub, 'u_sm/1', 'rule2_control/2', 'autorouting', 'on');

% Weighted sum: u_raw = theta1*rule1 + theta2*rule2.
add_block('simulink/Math Operations/Product', [sub '/theta1_times_rule1'], ...
    'Position', [430 195 475 235]);
add_block('simulink/Math Operations/Product', [sub '/theta2_times_rule2'], ...
    'Position', [430 295 475 335]);
add_block('simulink/Math Operations/Sum', [sub '/u_raw_sum'], ...
    'Inputs', '++', ...
    'Position', [565 245 600 285]);

add_line(sub, 'rule1_control/1', 'theta1_times_rule1/1', 'autorouting', 'on');
add_line(sub, 'theta1/1', 'theta1_times_rule1/2', 'autorouting', 'on');
add_line(sub, 'rule2_control/1', 'theta2_times_rule2/1', 'autorouting', 'on');
add_line(sub, 'theta2/1', 'theta2_times_rule2/2', 'autorouting', 'on');
add_line(sub, 'theta1_times_rule1/1', 'u_raw_sum/1', 'autorouting', 'on');
add_line(sub, 'theta2_times_rule2/1', 'u_raw_sum/2', 'autorouting', 'on');
add_line(sub, 'u_raw_sum/1', 'u_raw/1', 'autorouting', 'on');
end

function build_membership(sub)
add_block('simulink/Signal Routing/Selector', [sub '/select_Rw_hat'], ...
    'NumberOfDimensions', '1', ...
    'IndexMode', 'One-based', ...
    'Indices', '6', ...
    'InputPortWidth', '6', ...
    'Position', [120 425 170 455]);
add_block('simulink/Math Operations/Gain', [sub '/Rw_hat_over_Rwmax'], ...
    'Gain', '1/wind_p.Rw_max', ...
    'Position', [220 425 325 455]);
add_block('simulink/Sources/Constant', [sub '/one_for_theta'], ...
    'Value', '1', ...
    'Position', [220 480 250 510]);
add_block('simulink/Math Operations/Sum', [sub '/one_plus_Rw_ratio'], ...
    'Inputs', '++', ...
    'Position', [365 440 395 470]);
add_block('simulink/Math Operations/Gain', [sub '/theta1_half'], ...
    'Gain', '0.5', ...
    'Position', [430 440 495 470]);
add_block('simulink/Discontinuities/Saturation', [sub '/theta1'], ...
    'LowerLimit', '0', ...
    'UpperLimit', '1', ...
    'Position', [530 440 595 470]);
add_block('simulink/Sources/Constant', [sub '/one_minus_base'], ...
    'Value', '1', ...
    'Position', [530 500 560 530]);
add_block('simulink/Math Operations/Sum', [sub '/theta2'], ...
    'Inputs', '+-', ...
    'Position', [630 480 660 510]);

add_line(sub, 'x_hat/1', 'select_Rw_hat/1', 'autorouting', 'on');
add_line(sub, 'select_Rw_hat/1', 'Rw_hat_over_Rwmax/1', 'autorouting', 'on');
add_line(sub, 'Rw_hat_over_Rwmax/1', 'one_plus_Rw_ratio/1', 'autorouting', 'on');
add_line(sub, 'one_for_theta/1', 'one_plus_Rw_ratio/2', 'autorouting', 'on');
add_line(sub, 'one_plus_Rw_ratio/1', 'theta1_half/1', 'autorouting', 'on');
add_line(sub, 'theta1_half/1', 'theta1/1', 'autorouting', 'on');
add_line(sub, 'one_minus_base/1', 'theta2/1', 'autorouting', 'on');
add_line(sub, 'theta1/1', 'theta2/2', 'autorouting', 'on');
end
