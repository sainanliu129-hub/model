function plot_preprocess_before_after_from_data(data_before, data_after, prep_opts, limb, plot_joints)
% plot_preprocess_before_after_from_data  根据预处理前后数据绘制对比图
%
% 输入：
%   data_before: struct with .t, .q, .qd, .qdd, .tau（预处理前）
%   data_after:  struct with .t, .q, .qd, .qdd, .tau_id, .tau_s（预处理后）
%   prep_opts:   预处理选项（用于图标题）
%   limb:       肢体名，如 'left_leg'
%   plot_joints: 可选，默认 1:6

if nargin < 5 || isempty(plot_joints)
    plot_joints = 1:6;
end

t_before = data_before.t;
q_before = data_before.q;
qd_before = data_before.qd;
qdd_before = data_before.qdd;
tau_before = data_before.tau;

t_after = data_after.t;
q_after = data_after.q;
qd_after = data_after.qd;
qdd_after = data_after.qdd;
tau_id_after = data_after.tau_id;
tau_s_after = data_after.tau_s;

plot_joints = plot_joints(plot_joints >= 1 & plot_joints <= 6);
if isempty(plot_joints), plot_joints = 1; end

nj = numel(plot_joints);
if isfield(prep_opts, 'derivative_method')
    method_name = lower(char(string(prep_opts.derivative_method)));
else
    method_name = 'lowpass_diff';
end
if strcmp(method_name, 'sg')
    q_after_label = 'after (SG)';
    qd_after_label = 'after (SG)';
    qdd_after_label = 'after (SG 2nd)';
    title_filter_desc = sprintf('SG[%d,%d]', prep_opts.sg_order, prep_opts.sg_frame);
else
    fc = prep_opts.q_lowpass_fc_Hz;
    ord = prep_opts.q_lowpass_order;
    q_after_label = sprintf('after (q lowpass %gHz)', fc);
    qd_after_label = 'after (q lowpass + diff)';
    qdd_after_label = 'after (q lowpass + 2nd diff)';
    title_filter_desc = sprintf('q lowpass=%gHz (ord=%d)+diff', fc, ord);
end

figure('Name', 'Preprocess_Before_After_trajectories', 'Position', [60, 60, 1500, 900]);
for ii = 1:nj
    j = plot_joints(ii);

    subplot(4, nj, ii);
    plot(t_before, q_before(:, j), 'Color', [0.7 0.7 0.7], 'LineWidth', 0.8); hold on;
    plot(t_after,  q_after(:, j),  'b', 'LineWidth', 1.1);
    xlabel('t (s)'); ylabel('q (rad)'); title(sprintf('q joint%d', j));
    legend({'before (raw)', q_after_label}, 'Location', 'best'); grid on; hold off;

    subplot(4, nj, nj + ii);
    plot(t_before, qd_before(:, j), 'Color', [0.7 0.7 0.7], 'LineWidth', 0.8); hold on;
    plot(t_after,  qd_after(:, j),  'b', 'LineWidth', 1.1);
    xlabel('t (s)'); ylabel('qd (rad/s)'); title(sprintf('qd joint%d', j));
    legend({'before (CSV)', qd_after_label}, 'Location', 'best'); grid on; hold off;

    subplot(4, nj, 2*nj + ii);
    plot(t_before, qdd_before(:, j), 'Color', [0.7 0.7 0.7], 'LineWidth', 0.8); hold on;
    plot(t_after,  qdd_after(:, j),  'b', 'LineWidth', 1.1);
    xlabel('t (s)'); ylabel('qdd (rad/s^2)'); title(sprintf('qdd joint%d', j));
    legend({'before (central diff)', qdd_after_label}, 'Location', 'best'); grid on; hold off;

    subplot(4, nj, 3*nj + ii);
    plot(t_before, tau_before(:, j), 'Color', [0.7 0.7 0.7], 'LineWidth', 0.8); hold on;
    plot(t_after,  tau_s_after(:, j), 'b', 'LineWidth', 1.1);
    plot(t_after,  tau_id_after(:, j), 'k--', 'LineWidth', 1.0);
    xlabel('t (s)'); ylabel('\tau (N\cdotm)'); title(sprintf('\\tau joint%d', j));
    legend({'before (CSV)', 'after (tau\_s lowpass)', 'after (tau\_id target)'}, 'Location', 'best');
    grid on; hold off;
end

t_start = prep_opts.t_start_s;
t_end = prep_opts.t_end_s;
tau_fc = 25;
if isfield(prep_opts, 'tau_lowpass_fc_Hz') && ~isempty(prep_opts.tau_lowpass_fc_Hz)
    tau_fc = prep_opts.tau_lowpass_fc_Hz;
end
sgtitle(sprintf('预处理前后轨迹对比（%s, t=[%.2f, %.2f], %s, tau lowpass=%dHz）',...
    limb, t_start, t_end, title_filter_desc, tau_fc));
end
