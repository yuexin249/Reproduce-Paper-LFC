function build_wind_plant()
%BUILD_WIND_PLANT Generate an independent T-S fuzzy wind plant subsystem.
%
% Generated files:
%   wind_plant.slx          Simulink model containing subsystem "wind plant"
%   wind_plant_parameters.m Workspace parameter initializer

rootDir = fileparts(mfilename('fullpath'));
if isempty(rootDir)
    rootDir = pwd;
end
addpath(rootDir);

wind_p = wind_plant_parameters();
assignin('base', 'wind_p', wind_p);

mdl = 'wind_plant';
sub = [mdl '/wind plant'];
outFile = fullfile(rootDir, [mdl '.slx']);

if bdIsLoaded(mdl)
    close_system(mdl, 0);
end

new_system(mdl);
open_system(mdl);
set_param(mdl, ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'ode4', ...
    'FixedStep', 'wind_p.h', ...
    'StopTime', 'wind_p.T(2)', ...
    'SaveFormat', 'Dataset');

add_block('simulink/Ports & Subsystems/Subsystem', sub, ...
    'Position', [180 110 430 330]);
prepare_subsystem(sub);

% Top-level ports for quick standalone connection.
add_block('simulink/Sources/In1', [mdl '/u'], 'Position', [45 145 75 159]);
add_block('simulink/Sources/In1', [mdl '/Pl'], 'Position', [45 220 75 234]);
add_block('simulink/Sinks/Out1', [mdl '/x'], 'Position', [540 135 570 149]);
add_block('simulink/Sinks/Out1', [mdl '/y'], 'Position', [540 205 570 219]);
add_block('simulink/Sinks/Out1', [mdl '/theta'], 'Position', [540 275 570 289]);

build_wind_subsystem(sub);

add_line(mdl, 'u/1', 'wind plant/1', 'autorouting', 'on');
add_line(mdl, 'Pl/1', 'wind plant/2', 'autorouting', 'on');
add_line(mdl, 'wind plant/1', 'x/1', 'autorouting', 'on');
add_line(mdl, 'wind plant/2', 'y/1', 'autorouting', 'on');
add_line(mdl, 'wind plant/3', 'theta/1', 'autorouting', 'on');

Simulink.BlockDiagram.arrangeSystem(mdl);
save_system(mdl, outFile);
fprintf('Created %s\n', outFile);
fprintf('Workspace variable wind_p has been initialized.\n');
end

function prepare_subsystem(sub)
delete_line_safe(sub, 'In1/1', 'Out1/1');
delete_block_safe([sub '/In1']);
delete_block_safe([sub '/Out1']);
end

function build_wind_subsystem(sub)
stateNames = {'f', 'Pv', 'Pt', 'intACE', 'Irq', 'Rw'};

add_block('simulink/Sources/In1', [sub '/u'], 'Position', [30 105 60 119]);
add_block('simulink/Sources/In1', [sub '/Pl'], 'Position', [30 165 60 179]);
add_block('simulink/Sinks/Out1', [sub '/x'], 'Position', [940 120 970 134]);
add_block('simulink/Sinks/Out1', [sub '/y'], 'Position', [940 235 970 249]);
add_block('simulink/Sinks/Out1', [sub '/theta'], 'Position', [940 350 970 364]);

% Six state integrators, one state per equation.
y0 = 45;
dy = 72;
for k = 1:6
    add_block('simulink/Continuous/Integrator', [sub '/' stateNames{k}], ...
        'InitialCondition', sprintf('wind_p.x0(%d)', k), ...
        'Position', [620 y0+(k-1)*dy 665 y0+(k-1)*dy+35]);
end

add_block('simulink/Signal Routing/Mux', [sub '/x_mux'], ...
    'Inputs', '6', 'Position', [735 105 745 405]);
add_block('simulink/Signal Routing/Mux', [sub '/y_mux'], ...
    'Inputs', '3', 'Position', [825 215 835 295]);
add_block('simulink/Signal Routing/Mux', [sub '/theta_mux'], ...
    'Inputs', '2', 'Position', [825 350 835 395]);

for k = 1:6
    add_line(sub, [stateNames{k} '/1'], ['x_mux/' num2str(k)], 'autorouting', 'on');
end
add_line(sub, 'x_mux/1', 'x/1', 'autorouting', 'on');

% Fuzzy membership functions, paper form:
% theta1 = 0.5*(1 + Rw/Rw_max), theta2 = 1 - theta1, with saturation.
add_block('simulink/Math Operations/Gain', [sub '/Rw_over_Rwmax'], ...
    'Gain', '1/wind_p.Rw_max', 'Position', [160 430 250 460]);
add_block('simulink/Sources/Constant', [sub '/one_for_theta'], ...
    'Value', '1', 'Position', [160 485 190 515]);
add_block('simulink/Math Operations/Sum', [sub '/one_plus_RwRatio'], ...
    'Inputs', '++', 'Position', [290 445 320 475]);
add_block('simulink/Math Operations/Gain', [sub '/theta1_half'], ...
    'Gain', '0.5', 'Position', [350 445 415 475]);
add_block('simulink/Discontinuities/Saturation', [sub '/theta1'], ...
    'LowerLimit', '0', 'UpperLimit', '1', 'Position', [445 445 510 475]);
add_block('simulink/Sources/Constant', [sub '/one_minus_base'], ...
    'Value', '1', 'Position', [445 505 475 535]);
add_block('simulink/Math Operations/Sum', [sub '/theta2'], ...
    'Inputs', '+-', 'Position', [545 485 575 515]);

add_line(sub, 'Rw/1', 'Rw_over_Rwmax/1', 'autorouting', 'on');
add_line(sub, 'Rw_over_Rwmax/1', 'one_plus_RwRatio/1', 'autorouting', 'on');
add_line(sub, 'one_for_theta/1', 'one_plus_RwRatio/2', 'autorouting', 'on');
add_line(sub, 'one_plus_RwRatio/1', 'theta1_half/1', 'autorouting', 'on');
add_line(sub, 'theta1_half/1', 'theta1/1', 'autorouting', 'on');
add_line(sub, 'one_minus_base/1', 'theta2/1', 'autorouting', 'on');
add_line(sub, 'theta1/1', 'theta2/2', 'autorouting', 'on');
add_line(sub, 'theta1/1', 'theta_mux/1', 'autorouting', 'on');
add_line(sub, 'theta2/1', 'theta_mux/2', 'autorouting', 'on');
add_line(sub, 'theta_mux/1', 'theta/1', 'autorouting', 'on');

% Rule 1 and Rule 2 compute Ai*x + B*u + E*Pl. Weighted sums produce xdot.
for r = 1:2
    prefix = sprintf('rule%d', r);
    add_rule_blocks(sub, prefix, r, stateNames);
end

for eq = 1:6
    add_block('simulink/Math Operations/Product', ...
        sprintf('%s/rule1_xdot%d_weighted', sub, eq), ...
        'Position', [405 y0+(eq-1)*dy 445 y0+(eq-1)*dy+25]);
    add_block('simulink/Math Operations/Product', ...
        sprintf('%s/rule2_xdot%d_weighted', sub, eq), ...
        'Position', [405 y0+(eq-1)*dy+30 445 y0+(eq-1)*dy+55]);
    add_block('simulink/Math Operations/Sum', sprintf('%s/xdot%d', sub, eq), ...
        'Inputs', '++', 'Position', [515 y0+(eq-1)*dy+12 545 y0+(eq-1)*dy+42]);

    add_line(sub, sprintf('rule1_xdot%d/1', eq), sprintf('rule1_xdot%d_weighted/1', eq), 'autorouting', 'on');
    add_line(sub, 'theta1/1', sprintf('rule1_xdot%d_weighted/2', eq), 'autorouting', 'on');
    add_line(sub, sprintf('rule2_xdot%d/1', eq), sprintf('rule2_xdot%d_weighted/1', eq), 'autorouting', 'on');
    add_line(sub, 'theta2/1', sprintf('rule2_xdot%d_weighted/2', eq), 'autorouting', 'on');
    add_line(sub, sprintf('rule1_xdot%d_weighted/1', eq), sprintf('xdot%d/1', eq), 'autorouting', 'on');
    add_line(sub, sprintf('rule2_xdot%d_weighted/1', eq), sprintf('xdot%d/2', eq), 'autorouting', 'on');
    add_line(sub, sprintf('xdot%d/1', eq), [stateNames{eq} '/1'], 'autorouting', 'on');
end

% Plant output y = [ACE; Pv; intACE].
add_block('simulink/Math Operations/Gain', [sub '/ACE_beta_f'], ...
    'Gain', 'wind_p.beta', 'Position', [735 210 800 240]);
add_line(sub, 'f/1', 'ACE_beta_f/1', 'autorouting', 'on');
add_line(sub, 'ACE_beta_f/1', 'y_mux/1', 'autorouting', 'on');
add_line(sub, 'Pv/1', 'y_mux/2', 'autorouting', 'on');
add_line(sub, 'intACE/1', 'y_mux/3', 'autorouting', 'on');
add_line(sub, 'y_mux/1', 'y/1', 'autorouting', 'on');
end

function add_rule_blocks(sub, prefix, ruleIndex, stateNames)
% Adds Sum/Gain blocks for one T-S rule: xdot = Ai*x + B*u + E*Pl.
Aref = sprintf('wind_p.A%d', ruleIndex);
y0 = 45;
dy = 72;

terms = {
    {1, 1, sprintf('%s(1,1)', Aref), 'f'}, ...
    {1, 3, sprintf('%s(1,3)', Aref), 'Pt'}, ...
    {1, 5, sprintf('%s(1,5)', Aref), 'Irq'}, ...
    {1, 6, sprintf('%s(1,6)', Aref), 'Rw'}, ...
    {1, 7, 'wind_p.E(1)', 'Pl'}, ...
    {2, 1, sprintf('%s(2,1)', Aref), 'f'}, ...
    {2, 2, sprintf('%s(2,2)', Aref), 'Pv'}, ...
    {2, 7, 'wind_p.B(2)', 'u'}, ...
    {3, 2, sprintf('%s(3,2)', Aref), 'Pv'}, ...
    {3, 3, sprintf('%s(3,3)', Aref), 'Pt'}, ...
    {4, 1, sprintf('%s(4,1)', Aref), 'f'}, ...
    {5, 5, sprintf('%s(5,5)', Aref), 'Irq'}, ...
    {5, 7, 'wind_p.B(5)', 'u'}, ...
    {6, 5, sprintf('%s(6,5)', Aref), 'Irq'}, ...
    {6, 6, sprintf('%s(6,6)', Aref), 'Rw'} ...
    };

termCount = zeros(1, 6);
for i = 1:numel(terms)
    eq = terms{i}{1};
    termCount(eq) = termCount(eq) + 1;
end

for eq = 1:6
    add_block('simulink/Math Operations/Sum', sprintf('%s/%s_xdot%d', sub, prefix, eq), ...
        'Inputs', repmat('+', 1, termCount(eq)), ...
        'Position', [300 y0+(eq-1)*dy+12 330 y0+(eq-1)*dy+42]);
end

termIndex = zeros(1, 6);
for i = 1:numel(terms)
    eq = terms{i}{1};
    gainExpr = terms{i}{3};
    source = terms{i}{4};
    termIndex(eq) = termIndex(eq) + 1;
    blockName = sprintf('%s_%s_to_xdot%d_%d', prefix, source, eq, termIndex(eq));
    add_block('simulink/Math Operations/Gain', [sub '/' blockName], ...
        'Gain', gainExpr, ...
        'Position', [150 y0+(eq-1)*dy+(termIndex(eq)-1)*18 245 y0+(eq-1)*dy+20+(termIndex(eq)-1)*18]);
    add_line(sub, [source '/1'], [blockName '/1'], 'autorouting', 'on');
    add_line(sub, [blockName '/1'], sprintf('%s_xdot%d/%d', prefix, eq, termIndex(eq)), 'autorouting', 'on');
end
end

function delete_block_safe(pathName)
try
    delete_block(pathName);
catch
end
end

function delete_line_safe(sys, src, dst)
try
    delete_line(sys, src, dst);
catch
end
end
