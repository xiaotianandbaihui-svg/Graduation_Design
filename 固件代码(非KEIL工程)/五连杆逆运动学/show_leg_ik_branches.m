function show_leg_ik_branches(visibility)
% SHOW_LEG_IK_BRANCHES Draw why inverse kinematics has two folding branches.
% Run show_leg_ik_branches in MATLAB. No toolbox is needed.
% The example uses +x to the right, +y downward, as in leg_ik.h.
% All lengths below are illustrative, not measured robot dimensions.
if nargin < 1
    visibility = 'on';
end
folder = fileparts(mfilename('fullpath'));
font = 'Microsoft YaHei';
orange = [0.88 0.30 0.12];
blue = [0.10 0.40 0.79];
green = [0.07 0.51 0.35];
gray = [0.52 0.57 0.63];

%% One side of the leg: same A and P, two possible elbow positions B.
fig = figure('Name', '同一个目标点，两种连杆姿态', 'NumberTitle', 'off', ...
    'Color', 'white', 'Position', [80 100 1200 660], 'Visible', visibility);
layout = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'loose');
title(layout, 'A 和 P 都不动：中间的肘关节 B 可以在两个位置', ...
    'FontName', font, 'FontSize', 20, 'FontWeight', 'bold');
A = [0 0]; P = [100 100];
elbows = [0 100; 100 0];
names = {'branch = +1：先向下，再向右', 'branch = -1：先向右，再向下'};
for k = 1:2
    ax = nexttile(layout); hold(ax, 'on');
    set(ax, 'FontName', font, 'FontSize', 12, 'YDir', 'reverse', ...
        'XColor', gray, 'YColor', gray, 'GridColor', [0.82 0.85 0.89], ...
        'GridAlpha', 0.6, 'Layer', 'bottom');
    axis(ax, 'equal'); xlim(ax, [-35 145]); ylim(ax, [-28 140]);
    xticks(ax, [0 50 100]); yticks(ax, [0 50 100]); grid(ax, 'on');
    B = elbows(k, :);
    assert(abs(norm(B-A)-100) < 1e-8 && abs(norm(P-B)-100) < 1e-8);
    plot(ax, [A(1) P(1)], [A(2) P(2)], '--', 'Color', [0.78 0.8 0.83], 'LineWidth', 1.5);
    plot(ax, [A(1) B(1)], [A(2) B(2)], '-', 'Color', orange, 'LineWidth', 9);
    plot(ax, [B(1) P(1)], [B(2) P(2)], '-', 'Color', blue, 'LineWidth', 9);
    plot(ax, A(1), A(2), 'ko', 'MarkerFaceColor', [0.15 0.19 0.23], 'MarkerSize', 12);
    plot(ax, B(1), B(2), 'ko', 'MarkerFaceColor', [1 0.85 0.38], 'MarkerSize', 12);
    plot(ax, P(1), P(2), 'o', 'Color', green, 'MarkerFaceColor', green, 'MarkerSize', 13);
    text(ax, 0, -15, 'A：固定舵机轴', 'HorizontalAlignment', 'center', ...
        'FontName', font, 'FontSize', 14, 'FontWeight', 'bold');
    text(ax, 100, 116, 'P：固定目标点', 'HorizontalAlignment', 'center', ...
        'FontName', font, 'FontSize', 14, 'Color', green, 'FontWeight', 'bold');
    if k == 1
        text(ax, -8, 108, 'B', 'FontName', font, 'FontSize', 16, 'FontWeight', 'bold');
        text(ax, 6, 50, 'L1 = 100 mm', 'FontName', font, 'FontSize', 13, 'Color', orange);
        text(ax, 50, 92, 'L2 = 100 mm', 'HorizontalAlignment', 'center', ...
            'FontName', font, 'FontSize', 13, 'Color', blue);
        label = '主动杆角度 q = 90°';
    else
        text(ax, 108, -4, 'B', 'FontName', font, 'FontSize', 16, 'FontWeight', 'bold');
        text(ax, 50, -8, 'L1 = 100 mm', 'HorizontalAlignment', 'center', ...
            'FontName', font, 'FontSize', 13, 'Color', orange);
        text(ax, 94, 50, 'L2 = 100 mm', 'HorizontalAlignment', 'right', ...
            'FontName', font, 'FontSize', 13, 'Color', blue);
        label = '主动杆角度 q = 0°';
    end
    title(ax, {names{k}, label}, 'FontName', font, 'FontSize', 15, 'FontWeight', 'normal');
    xlabel(ax, 'x / mm（向右）', 'FontName', font);
    ylabel(ax, 'y / mm（向下）', 'FontName', font);
end
drawnow;
exportgraphics(fig, fullfile(folder, '两根连杆_两种姿态.png'), 'Resolution', 140);
savefig(fig, fullfile(folder, '两根连杆_两种姿态.fig'));

%% Full five-bar linkage: hold branch4 fixed and change only branch1.
fig2 = figure('Name', '五连杆：branch1选择哪种折叠姿态', 'NumberTitle', 'off', ...
    'Color', 'white', 'Position', [130 140 1200 680], 'Visible', visibility);
layout2 = tiledlayout(fig2, 1, 2, 'TileSpacing', 'compact', 'Padding', 'loose');
title(layout2, {'放回五连杆：轮轴 P 位置相同，右侧姿态也相同', ...
    '只改变左侧 branch1，肘关节 B 的位置就不同'}, ...
    'FontName', font, 'FontSize', 18, 'FontWeight', 'bold');
L1 = 100; L2 = 140; L3 = 140; L4 = 100; L5 = 80;
A = [-L5/2 0]; E = [L5/2 0]; P = [0 160];
q4 = side_angle(P-E, L4, L3, -1);
D = E + L4*[cos(q4) sin(q4)];
for k = 1:2
    branch = 3-2*k;
    q1 = side_angle(P-A, L1, L2, branch);
    B = A + L1*[cos(q1) sin(q1)];
    assert(abs(norm(B-P)-L2) < 1e-8 && abs(norm(D-P)-L3) < 1e-8);
    ax = nexttile(layout2); hold(ax, 'on');
    set(ax, 'FontName', font, 'FontSize', 12, 'YDir', 'reverse', ...
        'GridAlpha', 0.45, 'XColor', gray, 'YColor', gray);
    axis(ax, 'equal'); xlim(ax, [-150 150]); ylim(ax, [-25 205]); grid(ax, 'on');
    plot(ax, [A(1) E(1)], [0 0], '-', 'Color', [0.25 0.28 0.32], 'LineWidth', 5);
    hFixed = plot(ax, [E(1) D(1) P(1)], [E(2) D(2) P(2)], '-', 'Color', gray, 'LineWidth', 6);
    h1 = plot(ax, [A(1) B(1)], [A(2) B(2)], '-', 'Color', orange, 'LineWidth', 8);
    h2 = plot(ax, [B(1) P(1)], [B(2) P(2)], '-', 'Color', blue, 'LineWidth', 8);
    plot(ax, [A(1) E(1)], [A(2) E(2)], 'ko', 'MarkerFaceColor', [0.15 0.19 0.23], 'MarkerSize', 10);
    plot(ax, D(1), D(2), 'ko', 'MarkerFaceColor', gray, 'MarkerSize', 10);
    plot(ax, B(1), B(2), 'ko', 'MarkerFaceColor', [1 0.85 0.38], 'MarkerSize', 12);
    plot(ax, P(1), P(2), 'o', 'Color', green, 'MarkerFaceColor', green, 'MarkerSize', 13);
    text(ax, A(1)-5, -10, 'A', 'FontSize', 14, 'FontWeight', 'bold');
    text(ax, E(1)+3, -10, 'E', 'FontSize', 14, 'FontWeight', 'bold');
    text(ax, B(1)-12, B(2)-7, 'B', 'FontSize', 16, 'FontWeight', 'bold');
    text(ax, D(1)+8, D(2), 'D', 'FontSize', 14, 'Color', gray);
    text(ax, 0, 178, '同一个 P = (0, 160) mm', 'HorizontalAlignment', 'center', ...
        'FontName', font, 'FontSize', 14, 'Color', green, 'FontWeight', 'bold');
    title(ax, {sprintf('branch1 = %+d，branch4 = -1', branch), ...
        sprintf('q1 = %.1f°，q4 = %.1f°', rad2deg(q1), rad2deg(q4))}, ...
        'FontName', font, 'FontSize', 14, 'FontWeight', 'normal');
    xlabel(ax, 'x / mm', 'FontName', font); ylabel(ax, 'y / mm（向下）', 'FontName', font);
    legend(ax, [h1 h2 hFixed], {'L1 主动杆', 'L2 从动杆', '右侧保持相同'}, ...
        'Location', 'southoutside', 'Orientation', 'horizontal', 'Box', 'off', ...
        'FontName', font, 'FontSize', 11);
end
drawnow;
exportgraphics(fig2, fullfile(folder, '五连杆_branch1对比.png'), 'Resolution', 140);
savefig(fig2, fullfile(folder, '五连杆_branch1对比.fig'));
fprintf('LEG_IK_BRANCH_PLOTS_READY: %s\n', folder);
end

function q = side_angle(target, upper, lower, branch)
d = hypot(target(1), target(2));
c = (upper^2 + d^2 - lower^2)/(2*upper*d);
q = atan2(target(2), target(1)) + branch*acos(max(-1, min(1, c)));
end
