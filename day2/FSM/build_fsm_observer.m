function build_fsm_observer()
%BUILD_FSM_OBSERVER Generate a fuzzy sliding-mode observer model.
%
% Observer equation:
%   xhat_dot = sum_j theta_j * (Aj*xhat + Bj*u + Lj*(y_dos - yhat) ...
%              + S*(y_dos - yhat))
%   yhat     = C*xhat
%
% The model reuses wind_plant_parameters.m from the parent day2 folder.

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

fsm_h = 0.01;
fsm_Tstop = 50;
fsm_xhat0 = zeros(6, 1);

% Placeholder observer/sliding gains. Replace these with LMI-designed gains
% when available. Dimensions are fixed for compile-time consistency.
fsm_L1 = 0.25 * wind_p.C';
fsm_L2 = 0.25 * wind_p.C';
fsm_S = 0.05 * wind_p.C';

assignin('base', 'wind_p', wind_p);
assignin('base', 'fsm_h', fsm_h);
assignin('base', 'fsm_Tstop', fsm_Tstop);
assignin('base', 'fsm_xhat0', fsm_xhat0);
assignin('base', 'fsm_L1', fsm_L1);
assignin('base', 'fsm_L2', fsm_L2);
assignin('base', 'fsm_S', fsm_S);

mdl = 'fsm_observer';
sub = [mdl '/Fuzzy_Sliding_Mode_Observer'];
outFile = fullfile(rootDir, [mdl '.slx']);

if bdIsLoaded(mdl)
    close_system(mdl, 0);
end

new_system(mdl);
open_system(mdl);
set_param(mdl, ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'ode4', ...
    'FixedStep', 'fsm_h', ...
    'StopTime', 'fsm_Tstop', ...
    'SaveFormat', 'Dataset');

add_block('simulink/Ports & Subsystems/Subsystem', sub, ...
    'Position', [185 95 455 255]);
prepare_subsystem(sub);
build_observer_subsystem(sub);

add_block('simulink/Sources/In1', [mdl '/y_dos'], ...
    'Position', [45 130 75 144]);
add_block('simulink/Sources/In1', [mdl '/u'], ...
    'Position', [45 200 75 214]);
add_block('simulink/Sinks/Out1', [mdl '/x_hat'], ...
    'Position', [565 120 595 134]);
add_block('simulink/Sinks/Out1', [mdl '/e_y'], ...
    'Position', [565 205 595 219]);

add_line(mdl, 'y_dos/1', 'Fuzzy_Sliding_Mode_Observer/1', 'autorouting', 'on');
add_line(mdl, 'u/1', 'Fuzzy_Sliding_Mode_Observer/2', 'autorouting', 'on');
add_line(mdl, 'Fuzzy_Sliding_Mode_Observer/1', 'x_hat/1', 'autorouting', 'on');
add_line(mdl, 'Fuzzy_Sliding_Mode_Observer/2', 'e_y/1', 'autorouting', 'on');

Simulink.BlockDiagram.arrangeSystem(mdl);
save_system(mdl, outFile);
fprintf('Created %s\n', outFile);
fprintf('Workspace variables initialized: wind_p, fsm_h, fsm_Tstop, fsm_xhat0, fsm_L1, fsm_L2, fsm_S.\n');
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

function build_observer_subsystem(sub)
add_block('simulink/Sources/In1', [sub '/y_dos'], ...
    'Position', [35 135 65 149]);
add_block('simulink/Sources/In1', [sub '/u'], ...
    'Position', [35 345 65 359]);
add_block('simulink/Sinks/Out1', [sub '/x_hat'], ...
    'Position', [930 155 960 169]);
add_block('simulink/Sinks/Out1', [sub '/e_y'], ...
    'Position', [930 285 960 299]);

add_block('simulink/Continuous/Integrator', [sub '/xhat_integrator_group'], ...
    'InitialCondition', 'fsm_xhat0', ...
    'Position', [710 145 755 185]);

add_block('simulink/Math Operations/Gain', [sub '/C_xhat'], ...
    'Gain', 'wind_p.C', ...
    'Multiplication', 'Matrix(K*u)', ...
    'Position', [130 210 235 250]);
add_block('simulink/Math Operations/Sum', [sub '/e_y_calc'], ...
    'Inputs', '+-', ...
    'Position', [285 185 315 225]);

add_line(sub, 'xhat_integrator_group/1', 'C_xhat/1', 'autorouting', 'on');
add_line(sub, 'y_dos/1', 'e_y_calc/1', 'autorouting', 'on');
add_line(sub, 'C_xhat/1', 'e_y_calc/2', 'autorouting', 'on');
add_line(sub, 'e_y_calc/1', 'e_y/1', 'autorouting', 'on');
add_line(sub, 'xhat_integrator_group/1', 'x_hat/1', 'autorouting', 'on');

build_membership(sub);
build_rule(sub, 1, 'wind_p.A1', 'fsm_L1', 45);
build_rule(sub, 2, 'wind_p.A2', 'fsm_L2', 395);

add_block('simulink/Math Operations/Product', [sub '/theta1_times_rule1'], ...
    'Position', [555 105 600 145]);
add_block('simulink/Math Operations/Product', [sub '/theta2_times_rule2'], ...
    'Position', [555 455 600 495]);
add_block('simulink/Math Operations/Sum', [sub '/xhat_dot_sum'], ...
    'Inputs', '++', ...
    'Position', [640 270 675 310]);

add_line(sub, 'rule1_sum/1', 'theta1_times_rule1/1', 'autorouting', 'on');
add_line(sub, 'theta1/1', 'theta1_times_rule1/2', 'autorouting', 'on');
add_line(sub, 'rule2_sum/1', 'theta2_times_rule2/1', 'autorouting', 'on');
add_line(sub, 'theta2/1', 'theta2_times_rule2/2', 'autorouting', 'on');
add_line(sub, 'theta1_times_rule1/1', 'xhat_dot_sum/1', 'autorouting', 'on');
add_line(sub, 'theta2_times_rule2/1', 'xhat_dot_sum/2', 'autorouting', 'on');
add_line(sub, 'xhat_dot_sum/1', 'xhat_integrator_group/1', 'autorouting', 'on');
end

function build_membership(sub)
add_block('simulink/Signal Routing/Selector', [sub '/select_Rw_hat'], ...
    'NumberOfDimensions', '1', ...
    'IndexMode', 'One-based', ...
    'Indices', '6', ...
    'InputPortWidth', '6', ...
    'Position', [800 390 850 420]);
add_block('simulink/Math Operations/Gain', [sub '/Rw_hat_over_Rwmax'], ...
    'Gain', '1/wind_p.Rw_max', ...
    'Position', [115 500 220 530]);
add_block('simulink/Sources/Constant', [sub '/one_for_theta'], ...
    'Value', '1', ...
    'Position', [115 550 145 580]);
add_block('simulink/Math Operations/Sum', [sub '/one_plus_Rw_ratio'], ...
    'Inputs', '++', ...
    'Position', [255 515 285 545]);
add_block('simulink/Math Operations/Gain', [sub '/theta1_half'], ...
    'Gain', '0.5', ...
    'Position', [320 515 385 545]);
add_block('simulink/Discontinuities/Saturation', [sub '/theta1'], ...
    'LowerLimit', '0', ...
    'UpperLimit', '1', ...
    'Position', [420 515 485 545]);
add_block('simulink/Sources/Constant', [sub '/one_minus_base'], ...
    'Value', '1', ...
    'Position', [420 575 450 605]);
add_block('simulink/Math Operations/Sum', [sub '/theta2'], ...
    'Inputs', '+-', ...
    'Position', [520 555 550 585]);

add_line(sub, 'xhat_integrator_group/1', 'select_Rw_hat/1', 'autorouting', 'on');
add_line(sub, 'select_Rw_hat/1', 'Rw_hat_over_Rwmax/1', 'autorouting', 'on');
add_line(sub, 'Rw_hat_over_Rwmax/1', 'one_plus_Rw_ratio/1', 'autorouting', 'on');
add_line(sub, 'one_for_theta/1', 'one_plus_Rw_ratio/2', 'autorouting', 'on');
add_line(sub, 'one_plus_Rw_ratio/1', 'theta1_half/1', 'autorouting', 'on');
add_line(sub, 'theta1_half/1', 'theta1/1', 'autorouting', 'on');
add_line(sub, 'one_minus_base/1', 'theta2/1', 'autorouting', 'on');
add_line(sub, 'theta1/1', 'theta2/2', 'autorouting', 'on');
end

function build_rule(sub, ruleIndex, Aexpr, Lexpr, yBase)
ruleName = sprintf('rule%d', ruleIndex);

add_block('simulink/Math Operations/Gain', [sub '/' ruleName '_A_xhat'], ...
    'Gain', Aexpr, ...
    'Multiplication', 'Matrix(K*u)', ...
    'Position', [120 yBase 235 yBase+40]);
add_block('simulink/Math Operations/Gain', [sub '/' ruleName '_B_u'], ...
    'Gain', 'wind_p.B', ...
    'Multiplication', 'Matrix(K*u)', ...
    'Position', [120 yBase+60 235 yBase+100]);
add_block('simulink/Math Operations/Gain', [sub '/' ruleName '_L_ey'], ...
    'Gain', Lexpr, ...
    'Multiplication', 'Matrix(K*u)', ...
    'Position', [120 yBase+120 235 yBase+160]);
add_block('simulink/Math Operations/Gain', [sub '/' ruleName '_S_ey'], ...
    'Gain', 'fsm_S', ...
    'Multiplication', 'Matrix(K*u)', ...
    'Position', [120 yBase+180 235 yBase+220]);
add_block('simulink/Math Operations/Sum', [sub '/' ruleName '_sum'], ...
    'Inputs', '++++', ...
    'Position', [355 yBase+88 395 yBase+128]);

add_line(sub, 'xhat_integrator_group/1', [ruleName '_A_xhat/1'], 'autorouting', 'on');
add_line(sub, 'u/1', [ruleName '_B_u/1'], 'autorouting', 'on');
add_line(sub, 'e_y_calc/1', [ruleName '_L_ey/1'], 'autorouting', 'on');
add_line(sub, 'e_y_calc/1', [ruleName '_S_ey/1'], 'autorouting', 'on');
add_line(sub, [ruleName '_A_xhat/1'], [ruleName '_sum/1'], 'autorouting', 'on');
add_line(sub, [ruleName '_B_u/1'], [ruleName '_sum/2'], 'autorouting', 'on');
add_line(sub, [ruleName '_L_ey/1'], [ruleName '_sum/3'], 'autorouting', 'on');
add_line(sub, [ruleName '_S_ey/1'], [ruleName '_sum/4'], 'autorouting', 'on');
end
