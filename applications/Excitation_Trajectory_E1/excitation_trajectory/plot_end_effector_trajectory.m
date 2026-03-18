function plot_end_effector_trajectory(robot_limb, refPos, t_period, title_prefix, pillar)
% plot_end_effector_trajectory  绘制末端连杆的单周期 xyz 笛卡尔轨迹
%
% 输入:
%   robot_limb   - rigidBodyTree（get_e1_limb_robot 返回）
%   refPos       - 单周期关节位置 (N×dim)
%   t_period     - 单周期时间列 (N×1)，单位 s
%   title_prefix - 图标题前缀字符串，如 'TLBO' 或 'Standalone'
%   pillar       - 可选，柱体参数 [cx, cy, half_dx, half_dy]（默认 [-0.5, 0.0, 0.1, 0.1]）
%
% 绘制内容：
%   左子图 - 末端 x/y/z 随时间变化曲线（三条线叠加），标注柱体 x 范围
%   右子图 - 末端 xyz 三维空间轨迹（plot3），绘制柱体障碍物（半透明长方体）

if nargin < 4, title_prefix = ''; end
if nargin < 5 || isempty(pillar), pillar = [-0.5, 0.0, 0.1, 0.1]; end

if isempty(robot_limb), return; end

try
    baseName  = robot_limb.BaseName;
    end_body  = robot_limb.Bodies{robot_limb.NumBodies}.Name;

    N = size(refPos, 1);
    xyz = zeros(N, 3);
    for i = 1:N
        cfg = refPos(i, :);
        if iscolumn(cfg), cfg = cfg'; end
        T = getTransform(robot_limb, cfg, end_body, baseName);
        xyz(i, :) = T(1:3, 4)';
    end

    cx = pillar(1); cy = pillar(2); hdx = pillar(3); hdy = pillar(4);
    x_lo = cx - hdx; x_hi = cx + hdx;
    y_lo = cy - hdy; y_hi = cy + hdy;

    fig_name = strtrim([title_prefix ' 末端笛卡尔轨迹 xyz']);
    figure('Name', fig_name, 'Position', [150, 150, 1200, 500]);

    % ---- 左：xyz 时间曲线 ----
    subplot(1, 2, 1);
    hold on;
    plot(t_period, xyz(:,1), 'r-',  'LineWidth', 1.4, 'DisplayName', 'x');
    plot(t_period, xyz(:,2), 'g-',  'LineWidth', 1.4, 'DisplayName', 'y');
    plot(t_period, xyz(:,3), 'b-',  'LineWidth', 1.4, 'DisplayName', 'z');
    % 柱体 x 范围参考线
    yline(x_lo, 'r--', 'LineWidth', 0.9, 'DisplayName', sprintf('柱x_{lo}=%.2f', x_lo));
    yline(x_hi, 'r:',  'LineWidth', 0.9, 'DisplayName', sprintf('柱x_{hi}=%.2f', x_hi));
    yline(y_lo, 'g--', 'LineWidth', 0.9, 'DisplayName', sprintf('柱y_{lo}=%.2f', y_lo));
    yline(y_hi, 'g:',  'LineWidth', 0.9, 'DisplayName', sprintf('柱y_{hi}=%.2f', y_hi));
    hold off;
    grid on; legend('Location', 'best', 'FontSize', 7);
    xlabel('t (s)'); ylabel('位置 (m)');
    title(sprintf('%s 末端 xyz（%s）', title_prefix, end_body), 'Interpreter', 'none');

    % ---- 右：三维空间轨迹 + 柱体 ----
    subplot(1, 2, 2);
    hold on;
    % 轨迹
    plot3(xyz(:,1), xyz(:,2), xyz(:,3), 'b-', 'LineWidth', 1.8, 'DisplayName', '末端轨迹');
    plot3(xyz(1,1),   xyz(1,2),   xyz(1,3),   'go', 'MarkerSize', 8, ...
        'MarkerFaceColor', 'g', 'DisplayName', '起点');
    plot3(xyz(end,1), xyz(end,2), xyz(end,3), 'rs', 'MarkerSize', 8, ...
        'MarkerFaceColor', 'r', 'DisplayName', '终点');
    % 柱体（长方形截面，z 范围取轨迹 z 范围扩展一点）
    try
        zmin_p = min(xyz(:,3)) - 0.05; zmax_p = max(xyz(:,3)) + 0.05;
        draw_box_pillar(x_lo, x_hi, y_lo, y_hi, zmin_p, zmax_p);
    catch; end
    hold off;
    grid on; legend('Location', 'best', 'FontSize', 7);
    xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
    title(sprintf('%s 末端三维轨迹（%s）', title_prefix, end_body), 'Interpreter', 'none');
    view(45, 25);

    sgtitle(sprintf('%s 激励轨迹 - 末端 %s 笛卡尔坐标', title_prefix, end_body), 'Interpreter', 'none');
catch ME
    warning('plot_end_effector_trajectory: %s', ME.message);
end
end

% ---- 绘制半透明长方体柱障碍物 ----
function draw_box_pillar(x_lo, x_hi, y_lo, y_hi, z_lo, z_hi)
% 6 个面，每面 2 个三角形
verts = [x_lo y_lo z_lo; x_hi y_lo z_lo; x_hi y_hi z_lo; x_lo y_hi z_lo;
         x_lo y_lo z_hi; x_hi y_lo z_hi; x_hi y_hi z_hi; x_lo y_hi z_hi];
faces = [1 2 3 4;   % 底
         5 6 7 8;   % 顶
         1 2 6 5;   % 前
         3 4 8 7;   % 后
         1 4 8 5;   % 左
         2 3 7 6];  % 右
patch('Vertices', verts, 'Faces', faces, ...
      'FaceColor', [1 0.4 0.1], 'FaceAlpha', 0.25, ...
      'EdgeColor', [0.7 0.2 0], 'EdgeAlpha', 0.6, ...
      'DisplayName', sprintf('柱体 x∈[%.1f,%.1f] y∈[%.1f,%.1f]', x_lo,x_hi,y_lo,y_hi));
end
