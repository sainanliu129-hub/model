function visualize_trajectory(csv_file, varargin)
% visualize_trajectory  可视化轨迹CSV文件
%
% 用法：
%   visualize_trajectory('trajectory.csv')
%   visualize_trajectory('trajectory.csv', 'joints', [1:6])  % 只显示前6个关节
%   visualize_trajectory('trajectory.csv', 'save_fig', true)  % 保存图片

p = inputParser;
addRequired(p, 'csv_file', @ischar);
addParameter(p, 'joints', 1:12, @isnumeric);  % 要显示的关节索引
addParameter(p, 'save_fig', false, @islogical);  % 是否保存图片
parse(p, csv_file, varargin{:});
opts = p.Results;

% 确保 utility 在路径（read_trajectory_csv）
if isempty(which('read_trajectory_csv'))
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'utility_function'));
end

fprintf('读取轨迹文件: %s\n', opts.csv_file);
[time, q_all] = read_trajectory_csv(opts.csv_file);
fprintf('数据点数: %d, 时长: %.3f s\n', length(time), time(end));
fprintf('关节数: %d\n', size(q_all, 2));

[~, ~, joint_names] = get_e1_leg_joint_names();  % leg_l1_joint ... leg_r6_joint

% 绘制轨迹
num_joints = length(opts.joints);
if num_joints <= 6
    rows = 2;
    cols = 3;
elseif num_joints <= 9
    rows = 3;
    cols = 3;
else
    rows = 4;
    cols = 3;
end

figure('Name', '轨迹可视化', 'Position', [100, 100, 1400, 900]);
for i = 1:num_joints
    joint_idx = opts.joints(i);
    if joint_idx > size(q_all, 2)
        continue;
    end
    
    subplot(rows, cols, i);
    plot(time, q_all(:, joint_idx), 'b-', 'LineWidth', 1.5);
    grid on;
    xlabel('时间 (s)');
    ylabel('位置 (rad)');
    title(joint_names{joint_idx}, 'Interpreter', 'none');
end

sgtitle(sprintf('轨迹可视化: %s', opts.csv_file), 'Interpreter', 'none');

% 保存图片
if opts.save_fig
    [~, name, ~] = fileparts(opts.csv_file);
    fig_file = [name '_plot.png'];
    saveas(gcf, fig_file);
    fprintf('图片已保存: %s\n', fig_file);
end

% 显示统计信息
fprintf('\n===== 轨迹统计信息 =====\n');
fprintf('时间范围: [%.3f, %.3f] s\n', time(1), time(end));
fprintf('采样周期: %.5f s (%.1f Hz)\n', median(diff(time)), 1/median(diff(time)));
fprintf('\n各关节位置范围:\n');
for i = 1:min(12, size(q_all, 2))
    fprintf('  %s: [%.4f, %.4f] rad\n', joint_names{i}, ...
        min(q_all(:, i)), max(q_all(:, i)));
end

end
