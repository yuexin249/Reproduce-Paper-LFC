function build_scp_comm()
%BUILD_SCP_COMM Generate the stochastic communication protocol module.
%
% The SCP_Protocol subsystem implements:
%   r_k ~ U(0, 1)
%   transmit_k = r_k > scp_threshold
%   y_scp(k) = y(k),          if transmit_k = 1
%            = y_scp(k - 1),  if transmit_k = 0
%
% It is intended to be connected after wind_plant output y.

rootDir = fileparts(mfilename('fullpath'));
if isempty(rootDir)
    rootDir = pwd;
end

scp_h = 0.01;
scp_threshold = 0.6;
scp_seed = 2026;
assignin('base', 'scp_h', scp_h);
assignin('base', 'scp_threshold', scp_threshold);
assignin('base', 'scp_seed', scp_seed);

mdl = 'scp_comm';
sub = [mdl '/SCP_Protocol'];
outFile = fullfile(rootDir, [mdl '.slx']);

if bdIsLoaded(mdl)
    close_system(mdl, 0);
end

new_system(mdl);
open_system(mdl);
set_param(mdl, ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'ode4', ...
    'FixedStep', 'scp_h', ...
    'StopTime', '50', ...
    'SaveFormat', 'Dataset');

add_block('simulink/Ports & Subsystems/Subsystem', sub, ...
    'Position', [185 110 430 235]);
prepare_subsystem(sub);
build_scp_subsystem(sub);

add_block('simulink/Sources/In1', [mdl '/y'], ...
    'Position', [50 155 80 169]);
add_block('simulink/Sinks/Out1', [mdl '/y_scp'], ...
    'Position', [540 155 570 169]);

add_line(mdl, 'y/1', 'SCP_Protocol/1', 'autorouting', 'on');
add_line(mdl, 'SCP_Protocol/1', 'y_scp/1', 'autorouting', 'on');

Simulink.BlockDiagram.arrangeSystem(mdl);
save_system(mdl, outFile);
fprintf('Created %s\n', outFile);
fprintf('Workspace variables initialized: scp_h, scp_threshold, scp_seed.\n');
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

function build_scp_subsystem(sub)
add_block('simulink/Sources/In1', [sub '/y'], ...
    'Position', [40 125 70 139]);
add_block('simulink/Sinks/Out1', [sub '/y_scp'], ...
    'Position', [610 125 640 139]);

add_block('simulink/Sources/Uniform Random Number', [sub '/r_k'], ...
    'Minimum', '0', ...
    'Maximum', '1', ...
    'Seed', 'scp_seed', ...
    'SampleTime', 'scp_h', ...
    'Position', [40 30 120 65]);

add_block('simulink/Sources/Constant', [sub '/threshold'], ...
    'Value', 'scp_threshold', ...
    'Position', [40 80 120 110]);

add_block('simulink/Logic and Bit Operations/Relational Operator', ...
    [sub '/r_k_gt_threshold'], ...
    'Operator', '>', ...
    'Position', [165 45 205 85]);

add_block('simulink/Discrete/Unit Delay', [sub '/hold_y_scp_k_minus_1'], ...
    'InitialCondition', '0', ...
    'SampleTime', 'scp_h', ...
    'Position', [355 210 425 245]);

add_block('simulink/Signal Routing/Switch', [sub '/select_current_or_hold'], ...
    'Criteria', 'u2 ~= 0', ...
    'Threshold', '0.5', ...
    'Position', [430 105 490 165]);

add_block('simulink/Sinks/Terminator', [sub '/unused_rk_monitor'], ...
    'Position', [245 20 265 40]);

add_line(sub, 'r_k/1', 'r_k_gt_threshold/1', 'autorouting', 'on');
add_line(sub, 'threshold/1', 'r_k_gt_threshold/2', 'autorouting', 'on');

% Switch input order is: u1 current y, u2 control, u3 held output.
add_line(sub, 'y/1', 'select_current_or_hold/1', 'autorouting', 'on');
add_line(sub, 'r_k_gt_threshold/1', 'select_current_or_hold/2', 'autorouting', 'on');
add_line(sub, 'hold_y_scp_k_minus_1/1', 'select_current_or_hold/3', 'autorouting', 'on');

add_line(sub, 'select_current_or_hold/1', 'y_scp/1', 'autorouting', 'on');
add_line(sub, 'select_current_or_hold/1', 'hold_y_scp_k_minus_1/1', 'autorouting', 'on');
add_line(sub, 'r_k/1', 'unused_rk_monitor/1', 'autorouting', 'on');
end
