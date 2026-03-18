% plot_multi_speed_velocity  绘制多速度档位轨迹的速度图
%
% 功能：读取生成的多速度档位轨迹CSV文件，绘制各关节的速度时序图
%
% 用法:
%   plot_multi_speed_velocity('my_trajectory.csv')
%   plot_multi_speed_velocity('my_trajectory.csv', 'joints', [1:6])  % 只显示前6个关节

function plot_multi_speed_velocity(csv_file, varargin)

p = inputParser;
addRequired(p, 'csv_file', @ischar);
addParameter(p, 'joints', 1:12, @isnumeric);  % 要显示的关节索引
addParameter(p, 'save_fig', false, @islogical);  % 是否保存图片
addParameter(p, 'expected_speeds', [], @isnumeric);  % 期望速度档位数组（rad/s），如 [3.738, 7.476, 11.214]
parse(p, csv_file, varargin{:});
opts = p.Results;

% 确保 utility 在路径（read_trajectory_csv）
if isempty(which('read_trajectory_csv'))
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'utility_function'));
end

fprintf('===== 绘制多速度档位轨迹速度 =====\n');
fprintf('读取文件: %s\n', opts.csv_file);
[time, q_all] = read_trajectory_csv(opts.csv_file);
fprintf('数据点数: %d, 时长: %.3f s\n', length(time), time(end));
fprintf('关节数: %d\n', size(q_all, 2));

%% 计算采样周期
if length(time) > 1
    Ts = median(diff(time));
else
    Ts = 0.002;  % 默认
    warning('无法确定采样周期，使用默认值 %.4f s', Ts);
end
fprintf('采样周期: %.4f s (%.1f Hz)\n', Ts, 1/Ts);

%% 计算速度（数值微分）
qd_all = zeros(size(q_all));
for j = 1:size(q_all, 2)
    qd_all(:, j) = [0; diff(q_all(:, j)) / Ts];
end

[~, ~, joint_names] = get_e1_leg_joint_names();  % leg_l1_joint ... leg_r6_joint

%% 确定期望速度档位
v_all_abs = abs(qd_all);
v_global_max = max(v_all_abs(:));

if isempty(opts.expected_speeds)
    % 自动估算：假设速度档位为 [0.3, 0.6, 0.9] 倍最大速度
    speed_ratios = [0.3, 0.6, 0.9];
    v_max_urdf_est = v_global_max / max(speed_ratios);
    expected_speeds = v_max_urdf_est * speed_ratios;
    fprintf('\n分析速度档位（自动估算）...\n');
    fprintf('全局最大速度: %.3f rad/s\n', v_global_max);
    fprintf('估算的期望速度档位: ');
    fprintf('%.3f, ', expected_speeds);
    fprintf('rad/s\n');
else
    expected_speeds = opts.expected_speeds;
    fprintf('\n使用指定的期望速度档位: ');
    fprintf('%.3f, ', expected_speeds);
    fprintf('rad/s\n');
    fprintf('全局最大速度: %.3f rad/s\n', v_global_max);
end

%% 绘制速度图
num_joints = length(opts.joints);
if num_joints <= 4
    rows = 2;
    cols = 2;
elseif num_joints <= 6
    rows = 2;
    cols = 3;
elseif num_joints <= 9
    rows = 3;
    cols = 3;
else
    rows = 4;
    cols = 3;
end

figure('Name', '多速度档位轨迹 - 速度', 'Position', [100, 100, 1400, 900]);

for i = 1:num_joints
    joint_idx = opts.joints(i);
    if joint_idx > size(q_all, 2)
        continue;
    end
    
    subplot(rows, cols, i);
    plot(time, qd_all(:, joint_idx), 'g-', 'LineWidth', 1.5);
    grid on;
    xlabel('时间 (s)');
    ylabel('速度 (rad/s)');
    title(sprintf('%s', joint_names{joint_idx}), 'FontSize', 11, 'FontWeight', 'bold');
    
    % 添加期望速度参考线
    hold on;
    yline(0, 'k--', 'LineWidth', 0.5);
    colors = {'r', 'm', 'c'};  % 不同速度档位用不同颜色
    for v_idx = 1:length(expected_speeds)
        v_exp = expected_speeds(v_idx);
        color = colors{min(v_idx, length(colors))};
        yline(v_exp, [color '--'], 'LineWidth', 1.5, 'DisplayName', sprintf('期望: %.2f rad/s', v_exp));
        yline(-v_exp, [color '--'], 'LineWidth', 1.5);
    end
    hold off;
    
    % 添加统计信息
    v_max = max(abs(qd_all(:, joint_idx)));
    v_mean = mean(abs(qd_all(abs(qd_all(:, joint_idx)) > 1e-3, joint_idx)));  % 非零速度的平均值
    
    % 找出各速度档位的实际峰值
    v_abs = abs(qd_all(:, joint_idx));
    v_peaks = [];
    v_peak_info = {};
    for v_idx = 1:length(expected_speeds)
        v_exp = expected_speeds(v_idx);
        % 找出速度接近期望值的时间段（±20%容差）
        idx_near = find(v_abs > v_exp * 0.8 & v_abs < v_exp * 1.2);
        if ~isempty(idx_near)
            v_peak_actual = max(v_abs(idx_near));
            v_peaks = [v_peaks, v_peak_actual];
            error_pct = (v_peak_actual - v_exp) / v_exp * 100;
            v_peak_info{end+1} = sprintf('档%d: %.2f (误差%.1f%%)', v_idx, v_peak_actual, error_pct);
        end
    end
    
    % 显示统计信息
    info_str = sprintf('最大: %.3f rad/s', v_max);
    if ~isempty(v_peak_info)
        info_str = [info_str, sprintf('\n')];
        for k = 1:length(v_peak_info)
            info_str = [info_str, v_peak_info{k}, sprintf('\n')];
        end
    end
    if ~isnan(v_mean) && v_mean > 0.1
        info_str = [info_str, sprintf('平均: %.3f rad/s', v_mean)];
    end
    
    text(0.02, 0.95, info_str, ...
        'Units', 'normalized', 'VerticalAlignment', 'top', ...
        'BackgroundColor', 'white', 'EdgeColor', [0.5 0.5 0.5], 'FontSize', 9);
end

sgtitle('多速度档位轨迹 - 各关节速度', 'FontSize', 14, 'FontWeight', 'bold');

%% 保存图片（可选）
if opts.save_fig
    [~, name, ~] = fileparts(opts.csv_file);
    fig_file = fullfile(fileparts(opts.csv_file), [name, '_velocity.png']);
    saveas(gcf, fig_file);
    fprintf('图片已保存: %s\n', fig_file);
end

fprintf('\n===== 完成 =====\n');

end
