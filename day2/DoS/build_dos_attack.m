function build_dos_attack()
%BUILD_DOS_ATTACK Generate a DoS packet-loss module for the LFC chain.
%
% The DoS_Block subsystem implements:
%   r_k ~ U(0, 1)
%   dos_k = r_k < dos_threshold
%   y_dos(k) = y_dos(k - 1), if dos_k = 1
%            = y_scp(k),      if dos_k = 0
%
% This matches the communication-link DoS effect used in the wind-power
% LFC benchmark: attacked packets are lost and the receiver keeps the last
% available sample with a zero-order hold.

rootDir = fileparts(mfilename('fullpath'));
if isempty(rootDir)
    rootDir = pwd;
end
if ~exist(rootDir, 'dir')
    mkdir(rootDir);
end

dos_h = 0.01;
dos_threshold = 0.2;
dos_seed = 2026;
assignin('base', 'dos_h', dos_h);
assignin('base', 'dos_threshold', dos_threshold);
assignin('base', 'dos_seed', dos_seed);

mdl = 'dos_attack';
sub = [mdl '/DoS_Block'];
outFile = fullfile(rootDir, [mdl '.slx']);

if bdIsLoaded(mdl)
    close_system(mdl, 0);
end

new_system(mdl);
open_system(mdl);
set_param(mdl, ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'ode4', ...
    'FixedStep', 'dos_h', ...
    'StopTime', '50', ...
    'SaveFormat', 'Dataset');

add_block('simulink/Ports & Subsystems/Subsystem', sub, ...
    'Position', [185 110 430 235]);
prepare_subsystem(sub);
build_dos_subsystem(sub);

add_block('simulink/Sources/In1', [mdl '/y_scp'], ...
    'Position', [50 155 80 169]);
add_block('simulink/Sinks/Out1', [mdl '/y_dos'], ...
    'Position', [540 155 570 169]);

add_line(mdl, 'y_scp/1', 'DoS_Block/1', 'autorouting', 'on');
add_line(mdl, 'DoS_Block/1', 'y_dos/1', 'autorouting', 'on');

Simulink.BlockDiagram.arrangeSystem(mdl);
save_system(mdl, outFile);
fprintf('Created %s\n', outFile);
fprintf('Workspace variables initialized: dos_h, dos_threshold, dos_seed.\n');
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

function build_dos_subsystem(sub)
add_block('simulink/Sources/In1', [sub '/y_scp'], ...
    'Position', [40 125 70 139]);
add_block('simulink/Sinks/Out1', [sub '/y_dos'], ...
    'Position', [610 125 640 139]);

add_block('simulink/Sources/Uniform Random Number', [sub '/r_k'], ...
    'Minimum', '0', ...
    'Maximum', '1', ...
    'Seed', 'dos_seed', ...
    'SampleTime', 'dos_h', ...
    'Position', [40 30 120 65]);

add_block('simulink/Sources/Constant', [sub '/attack_threshold'], ...
    'Value', 'dos_threshold', ...
    'Position', [40 80 120 110]);

add_block('simulink/Logic and Bit Operations/Relational Operator', ...
    [sub '/r_k_lt_threshold'], ...
    'Operator', '<', ...
    'Position', [165 45 205 85]);

add_block('simulink/Discrete/Unit Delay', [sub '/hold_y_dos_k_minus_1'], ...
    'InitialCondition', '0', ...
    'SampleTime', 'dos_h', ...
    'Position', [355 210 425 245]);

add_block('simulink/Signal Routing/Switch', [sub '/select_hold_or_current'], ...
    'Criteria', 'u2 ~= 0', ...
    'Threshold', '0.5', ...
    'Position', [430 105 490 165]);

add_line(sub, 'r_k/1', 'r_k_lt_threshold/1', 'autorouting', 'on');
add_line(sub, 'attack_threshold/1', 'r_k_lt_threshold/2', 'autorouting', 'on');

% Switch input order: u1 held value when attacked, u2 DoS flag, u3 current y_scp.
add_line(sub, 'hold_y_dos_k_minus_1/1', 'select_hold_or_current/1', 'autorouting', 'on');
add_line(sub, 'r_k_lt_threshold/1', 'select_hold_or_current/2', 'autorouting', 'on');
add_line(sub, 'y_scp/1', 'select_hold_or_current/3', 'autorouting', 'on');

add_line(sub, 'select_hold_or_current/1', 'y_dos/1', 'autorouting', 'on');
add_line(sub, 'select_hold_or_current/1', 'hold_y_dos_k_minus_1/1', 'autorouting', 'on');
end
