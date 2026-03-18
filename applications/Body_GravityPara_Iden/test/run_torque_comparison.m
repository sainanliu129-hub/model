%% run_torque_comparison  多版本力矩对比：牛顿-欧拉刚体 / 刚体+摩擦+转子 / 辨识力矩 vs 实际力矩
%
% 两种模式（同数据下结果会不同）：
%   align_test_ReMatrix = false（默认）：先预处理 SG(q)→q/qd/qdd、τ 低通，再窗；τ_meas=滤波力矩，轨迹=SG 结果
%   align_test_ReMatrix = true：        与 test_ReMatrix_E1_limb_URDF 完全一致：不预处理，窗 [2,4]、qd 50Hz、qdd half=15；
%                                        τ_meas=窗内原始力矩，τ_ref=inverseDynamics(同一轨迹)，应与 test 图一致
%
% 使用步骤：
%   1. 修改下方「一、配置」中的 csv_file、align_test_ReMatrix 等
%   2. 若要与 test_ReMatrix 同一 CSV 得到相同三条线，设 align_test_ReMatrix = true
%   3. 在 MATLAB 中 cd 到 Body_GravityPara_Iden，运行：run_torque_comparison

function run_torque_comparison(csv_file, opts_in)

clc;
close all;

app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
ensure_body_gravity_para_iden_path();

%% ========== 一、配置 ==========
if nargin < 1 || isempty(csv_file)
    % 默认与 test_ReMatrix_E1_limb_URDF 的 csv_file_validation 一致，便于 align_test_ReMatrix 对比
    csv_file = fullfile(app_root, 'data', 'excitation', 'PD-M1-v0_multi_joint_20260305-195346_exctra_sim.csv');
end
if nargin < 2
    opts_in = struct();
end

t_start_s = 2.1;
t_end_s   = 4.1;
if isfield(opts_in, 't_start_s') && ~isempty(opts_in.t_start_s), t_start_s = opts_in.t_start_s; end
if isfield(opts_in, 't_end_s')   && ~isempty(opts_in.t_end_s),   t_end_s   = opts_in.t_end_s; end

load_friction_from_summary = true;
if isfield(opts_in, 'load_friction_from_summary'), load_friction_from_summary = opts_in.load_friction_from_summary; end
n_joints = 6;
if isfield(opts_in, 'n_joints'), n_joints = opts_in.n_joints; end
row_for_joint = [];
if isfield(opts_in, 'row_for_joint'), row_for_joint = opts_in.row_for_joint; end

% true = 从 min_param_id_result.mat 加载 X_hat/index_base/metrics，跳过辨识
use_saved_id = false;
if isfield(opts_in, 'use_saved_id'), use_saved_id = opts_in.use_saved_id; end

% true = 与 test_ReMatrix_E1_limb_URDF 完全一致：不预处理，窗 [2,4]、qd 50Hz、qdd_smooth_half=15，τ_meas=窗内原始
align_test_ReMatrix = false;
if isfield(opts_in, 'align_test_ReMatrix'), align_test_ReMatrix = opts_in.align_test_ReMatrix; end

% 预处理选项（与 run_iden_full_workflow 一致）
prep_opts = struct();
prep_opts.t_start_s         = t_start_s;
prep_opts.t_end_s           = t_end_s;
prep_opts.sg_order          = 4;
prep_opts.sg_frame          = 23;
prep_opts.tau_lowpass_fc_Hz = 15;
prep_opts.tau_lowpass_order = 2;
prep_opts.do_plot           = false;   % 本脚本只画对比图，可改为 true 看预处理图
prep_opts.plot_joints       = [1 2 3];

%% ========== 二、摩擦/惯量（可选） ==========
friction_params = [];
I_a = [];
if load_friction_from_summary
    friction_iden_dir = fullfile(app_root, '..', 'Friction_Iden');
    if isfolder(friction_iden_dir) && isempty(which('load_friction_stribeck_from_summary'))
        addpath(friction_iden_dir);
    end
    load_opts = struct();
    if ~isempty(row_for_joint), load_opts.row_for_joint = row_for_joint; end
    [friction_params, I_a] = load_friction_stribeck_from_summary(n_joints, load_opts);
    fprintf('已从「全部电机参数汇总」表加载 stribeck_viscous 与 I_a（n_joints=%d）\n', n_joints);
end
prep_opts.J_eq = I_a;
prep_opts.friction_params = friction_params;

%% ========== 三、读 CSV + 预处理（可选）+ 连续窗 ==========
if ischar(csv_file) && exist(csv_file, 'file') ~= 2
    csv_file = fullfile(app_root, csv_file);
end
if ~exist(csv_file, 'file')
    error('文件不存在: %s', csv_file);
end

fprintf('===== 力矩多版本对比 =====\n');
fprintf('数据文件: %s\n', csv_file);

data = read_leg_joint_csv(csv_file);
t    = data.time(:);
q    = data.pos_leg_l;
qd   = data.vel_leg_l;
tau  = data.torque_leg_l;
if size(q,1) ~= length(t) || size(tau,1) ~= length(t)
    error('时间与 q/tau 行数不一致。');
end

qdd_for_win = [];
aux_prep = [];
t_raw = []; q_raw = []; qd_raw = []; tau_raw = [];  % 预处理前备份，供滤波前后图

if align_test_ReMatrix
    % 与 test_ReMatrix_E1_limb_URDF 完全一致：不预处理，窗 [2,4]、qd 50Hz、qdd_smooth_half=15
    fprintf('模式: 与 test_ReMatrix_E1_limb_URDF 一致（不预处理，窗 [2,4] s，qd 50Hz，qdd half=15）\n');
    win_opts = struct('t_start_s', 2, 't_end_s', 4, 'qd_lowpass_fc_Hz', 50, 'qdd_smooth_half', 15);
    avg_data = continuous_window_id_data(t, q, qd, tau, win_opts);   % 不传 qdd_given，窗内由 qd 差分+qdd 平滑
else
    t_raw = t; q_raw = q; qd_raw = qd; tau_raw = tau;
    [t, q, qd, qdd_for_win, tau, aux_prep] = preprocess_id_data(t, q, qd, tau, prep_opts);
    fprintf('已做辨识前预处理，得到 tau_s（滤波力矩）与 tau_id\n');
    win_opts = struct('t_start_s', [], 't_end_s', [], 'qd_lowpass_fc_Hz', 0, 'qdd_smooth_half', 0);
    avg_data = continuous_window_id_data(t, q, qd, tau, win_opts, qdd_for_win);
end

t_equiv  = avg_data.t_equiv;
M        = size(avg_data.q_bar, 1);
fprintf('连续窗样本数 M = %d，时间范围 %.2f～%.2f s\n', M, t_equiv(1), t_equiv(end));

%% ========== 四、辨识结果：加载或重算 ==========
% align_test_ReMatrix 时必须用当前窗内数据重算，否则 tau_id_pred 与 test 不一致
out_mat = fullfile(app_root, 'min_param_id_result.mat');
if align_test_ReMatrix || ~use_saved_id || ~exist(out_mat, 'file')
    opts_id = struct('use_wls', false);
    if align_test_ReMatrix
        % 与 test 一致：不扣摩擦，b = tau_bar（窗内原始）
        opts_id.friction_params = [];
        opts_id.I_a = [];
    end
    [X_hat, index_base, metrics] = identify_min(avg_data, 'left_leg', opts_id);
    if align_test_ReMatrix
        fprintf('已运行最小参数辨识（与 test 一致：窗内原始 tau）\n');
    else
        fprintf('已运行最小参数辨识（预处理下不扣摩擦/转子）\n');
    end
else
    ld = load(out_mat);
    X_hat      = ld.X_hat;
    index_base = ld.index_base;
    metrics    = ld.metrics;
    fprintf('已从 %s 加载 X_hat, index_base, metrics\n', out_mat);
end

%% ========== 四b. Step1 条件数 + Step2 Regressor energy ==========
try
    q_bar  = avg_data.q_bar;
    qd_bar = avg_data.qd_bar;
    qdd_bar = avg_data.qdd_bar;
    Mm = size(q_bar, 1);
    Y_full = [];
    for k = 1:Mm
        Y_one = ReMatrix_E1_limb_URDF('left_leg', q_bar(k,:), qd_bar(k,:), qdd_bar(k,:), 1, 1);
        Y_full = [Y_full; Y_one];
    end
    Y_min = Y_full(:, index_base);
    cond_Y_min = cond(Y_min);
    fprintf('\n--- Step1: cond(Y_min) 【最重要】 = %.4e（最小参数 %d 列） ---\n', cond_Y_min, numel(index_base));
    % Step2: regressor energy ||Y(:,i)||
    ncol = size(Y_full, 2);
    regressor_energy = sqrt(sum(Y_full.^2, 1));
    fprintf('--- Step2: Regressor energy ||Y(:,i)||  min=%.4e, max=%.4e ---\n', min(regressor_energy), max(regressor_energy));
    idx_near_zero = find(regressor_energy < 1e-6 * max(regressor_energy));
    if ~isempty(idx_near_zero)
        fprintf('  列范数近似为 0（未被激励）: %s\n', mat2str(idx_near_zero));
    end
    fig_reg = figure('Name', 'Regressor_energy', 'Position', [160 160 1000 400]);
    bar(1:ncol, regressor_energy, 'FaceColor', [0.3 0.5 0.8]);
    xlabel('参数列 i'); ylabel('||Y(:,i)||_2');
    title('Regressor energy：列范数 \approx 0 表示该参数未被激励');
    grid on;
    if ~isempty(idx_near_zero)
        hold on;
        plot(idx_near_zero, regressor_energy(idx_near_zero), 'ro', 'MarkerSize', 8, 'LineWidth', 1.5);
        legend('||Y(:,i)||', '近似为 0（未激励）', 'Location', 'best');
        hold off;
    end
catch ME
    fprintf('Step1/Step2 跳过: %s\n', ME.message);
end

%% ========== 五、基准力矩与窗内对齐 ==========
if align_test_ReMatrix
    tau_meas_filt = avg_data.tau_bar;   % 与 test 一致：窗内原始力矩
else
    tau_meas_filt = aux_prep.tau_s;    % 预处理得到的滤波力矩
end
if size(tau_meas_filt, 1) ~= M
    L = min(size(tau_meas_filt, 1), M);
    tau_meas_filt = tau_meas_filt(1:L, :);
    t_equiv_plot  = t_equiv(1:L);
    q_bar_plot    = avg_data.q_bar(1:L, :);
    qd_bar_plot   = avg_data.qd_bar(1:L, :);
    qdd_bar_plot  = avg_data.qdd_bar(1:L, :);
    tau_pred_id   = metrics.tau_pred(1:L, :);
else
    t_equiv_plot  = t_equiv;
    q_bar_plot    = avg_data.q_bar;
    qd_bar_plot   = avg_data.qd_bar;
    qdd_bar_plot  = avg_data.qdd_bar;
    tau_pred_id   = metrics.tau_pred;
end
M_plot = size(q_bar_plot, 1);

%% ========== 五b. 滤波前后全关节 q, qd, qdd, tau（仅预处理模式） ==========
if ~align_test_ReMatrix && ~isempty(t_raw)
    [~, loc] = ismember(t, t_raw);
    if any(loc == 0)
        [~, loc] = min(abs(t_raw - t.'), [], 1);
    end
    q_before   = q_raw(loc, :);
    qd_before  = qd_raw(loc, :);
    tau_before = tau_raw(loc, :);
    dt_seg = median(diff(t));
    if dt_seg <= 0 || isnan(dt_seg), dt_seg = 0.002; end
    qdd_before = zeros(M_plot, 6);
    for j = 1:6
        for k = 2:M_plot-1
            qdd_before(k, j) = (qd_before(k+1, j) - qd_before(k-1, j)) / (2*dt_seg);
        end
        qdd_before(1, j) = qdd_before(2, j);
        qdd_before(M_plot, j) = qdd_before(M_plot-1, j);
    end
    fig_filt = figure('Name', '滤波前后全关节_q_qd_qdd_tau', 'Position', [50 50 1400 900]);
    for j = 1:6
        subplot(4, 6, j);
        plot(t_equiv_plot, q_before(:, j), 'Color', [0.65 0.65 0.65], 'LineWidth', 0.8); hold on;
        plot(t_equiv_plot, q_bar_plot(:, j), 'b', 'LineWidth', 1);
        xlabel('t (s)'); ylabel('rad'); title(sprintf('q 关节%d', j)); legend('滤波前', 'SG后'); grid on; hold off;
        subplot(4, 6, 6 + j);
        plot(t_equiv_plot, qd_before(:, j), 'Color', [0.65 0.65 0.65], 'LineWidth', 0.8); hold on;
        plot(t_equiv_plot, qd_bar_plot(:, j), 'b', 'LineWidth', 1);
        xlabel('t (s)'); ylabel('rad/s'); title(sprintf('qd 关节%d', j)); legend('滤波前', 'SG后'); grid on; hold off;
        subplot(4, 6, 12 + j);
        plot(t_equiv_plot, qdd_before(:, j), 'Color', [0.65 0.65 0.65], 'LineWidth', 0.8); hold on;
        plot(t_equiv_plot, qdd_bar_plot(:, j), 'b', 'LineWidth', 1);
        xlabel('t (s)'); ylabel('rad/s^2'); title(sprintf('qdd 关节%d', j)); legend('滤波前', 'SG后'); grid on; hold off;
        subplot(4, 6, 18 + j);
        plot(t_equiv_plot, tau_before(:, j), 'Color', [0.65 0.65 0.65], 'LineWidth', 0.8); hold on;
        plot(t_equiv_plot, tau_meas_filt(:, j), 'b', 'LineWidth', 1);
        xlabel('t (s)'); ylabel('N\cdotm'); title(sprintf('\\tau 关节%d', j)); legend('滤波前', '低通后'); grid on; hold off;
    end
    sgtitle('滤波前后对比：q/qd/qdd（SG 求导）与 \\tau（Butterworth 低通）');
end

%% ========== 六、牛顿-欧拉刚体 + 刚体+摩擦+转子 ==========
% 与 test_ReMatrix_E1_limb_URDF 一致：用 get_e1_limb_robot + inverseDynamics 计算刚体逆动力学
limb = 'left_leg';
para_order = 1;
[robot_limb, n_limb] = get_e1_limb_robot(limb);
tau_ne = zeros(M_plot, 6);
for k = 1:M_plot
    q_k   = q_bar_plot(k, :);
    qd_k  = qd_bar_plot(k, :);
    qdd_k = qdd_bar_plot(k, :);
    tau_ref_k = inverseDynamics(robot_limb, q_k, qd_k, qdd_k);
    tau_ne(k, :) = tau_ref_k(:)';
end

tau_comp_ne = zeros(M_plot, 6);
if ~isempty(friction_params) || ~isempty(I_a)
    tau_comp_ne = subtract_friction_rotor_torque(qd_bar_plot, qdd_bar_plot, friction_params, I_a);
end
tau_ne_plus = tau_ne + tau_comp_ne;

%% ========== 七、误差（基准 = 滤波后实际力矩） ==========
err_ne      = tau_ne      - tau_meas_filt;
err_ne_plus = tau_ne_plus - tau_meas_filt;
err_id      = tau_pred_id - tau_meas_filt;

t_span = t_equiv_plot(end) - t_equiv_plot(1);
joint_names = {'leg\_l1', 'leg\_l2', 'leg\_l3', 'leg\_l4', 'leg\_l5', 'leg\_l6'};

%% ========== 七b. 数据预处理参数与各模型误差统计（供测试报告） ==========
fprintf('\n');
fprintf('========== 数据与预处理参数（测试报告） ==========\n');
fprintf('  数据文件: %s\n', csv_file);
fprintf('  时间窗: %.2f～%.2f s，样本数 M = %d，时间跨度 = %.2f s\n', t_equiv_plot(1), t_equiv_plot(end), M_plot, t_span);
if align_test_ReMatrix
    fprintf('  模式: 与 test_ReMatrix 一致（不预处理）\n');
    fprintf('  连续窗: t_start_s = 2, t_end_s = 4\n');
    fprintf('  qd 低通: Butterworth 2 阶, fc = 50 Hz\n');
    fprintf('  qdd: 由 qd 中心差分 + SG 平滑, qdd_smooth_half = 15\n');
    fprintf('  基准力矩: 窗内原始 tau_bar\n');
else
    fprintf('  模式: 预处理（SG 求导 + τ 低通）\n');
    fprintf('  时间裁剪: t_start_s = %.2f, t_end_s = %.2f\n', prep_opts.t_start_s, prep_opts.t_end_s);
    fprintf('  SG: order = %d, frame = %d（先全段求导再裁剪）\n', prep_opts.sg_order, prep_opts.sg_frame);
    fprintf('  τ 低通: Butterworth %d 阶, fc = %d Hz\n', prep_opts.tau_lowpass_order, prep_opts.tau_lowpass_fc_Hz);
    fprintf('  基准力矩: 滤波后 tau_s（零相位）\n');
end
fprintf('  摩擦/转子: load_friction_from_summary = %d\n', load_friction_from_summary);
fprintf('================================================\n');

fprintf('\n========== 各模型相对基准力矩的误差统计（测试报告） ==========\n');
fprintf('  基准: tau_meas_filt（窗内实测/滤波力矩）\n');
fprintf('  模型: tau_NE_rigid（牛顿-欧拉刚体）, tau_NE+fric+Ia（刚体+摩擦+转子）, tau_id_pred（最小参数辨识）\n\n');

norm_meas = norm(tau_meas_filt, 'fro');
if norm_meas < 1e-12, norm_meas = 1; end

% 各关节 RMSE、最大绝对误差（每模型）
rmse_ne      = sqrt(mean(err_ne.^2, 1));      % 1×6
rmse_ne_plus = sqrt(mean(err_ne_plus.^2, 1));
rmse_id      = sqrt(mean(err_id.^2, 1));
max_ne      = max(abs(err_ne), [], 1);
max_ne_plus = max(abs(err_ne_plus), [], 1);
max_id      = max(abs(err_id), [], 1);

fprintf('  ---- 各关节 RMSE (N·m) ----\n');
fprintf('  关节       ');
fprintf('  %6d  ', 1:6);
fprintf('\n');
fprintf('  NE_rigid   ');
fprintf(' %7.4f ', rmse_ne);
fprintf('\n');
fprintf('  NE+fric+Ia ');
fprintf(' %7.4f ', rmse_ne_plus);
fprintf('\n');
fprintf('  id_pred    ');
fprintf(' %7.4f ', rmse_id);
fprintf('\n\n');

fprintf('  ---- 各关节最大绝对误差 (N·m) ----\n');
fprintf('  关节       ');
fprintf('  %6d  ', 1:6);
fprintf('\n');
fprintf('  NE_rigid   ');
fprintf(' %7.4f ', max_ne);
fprintf('\n');
fprintf('  NE+fric+Ia ');
fprintf(' %7.4f ', max_ne_plus);
fprintf('\n');
fprintf('  id_pred    ');
fprintf(' %7.4f ', max_id);
fprintf('\n\n');

fprintf('  ---- 整体误差（相对 ||tau_meas||_F） ----\n');
fprintf('  ||err_NE_rigid||_F    = %.4f, 相对 = %.4f\n', norm(err_ne, 'fro'), norm(err_ne, 'fro')/norm_meas);
fprintf('  ||err_NE+fric+Ia||_F  = %.4f, 相对 = %.4f\n', norm(err_ne_plus, 'fro'), norm(err_ne_plus, 'fro')/norm_meas);
fprintf('  ||err_id_pred||_F     = %.4f, 相对 = %.4f\n', norm(err_id, 'fro'), norm(err_id, 'fro')/norm_meas);
fprintf('  ||tau_meas_filt||_F   = %.4f\n', norm_meas);
fprintf('================================================\n\n');

%% ========== 八、力矩对比图 ==========
fig_cmp = figure('Name', '力矩多版本对比', 'Position', [120, 120, 1200, 800]);
for j = 1:6
    subplot(2, 3, j);
    plot(t_equiv_plot, tau_meas_filt(:, j), 'k-', 'LineWidth', 1.2); hold on;
    plot(t_equiv_plot, tau_ne(:, j),       'b--', 'LineWidth', 1.0);
    plot(t_equiv_plot, tau_ne_plus(:, j), 'g-.', 'LineWidth', 1.0);
    plot(t_equiv_plot, tau_pred_id(:, j),  'r:', 'LineWidth', 1.2);
    xlabel('t (s)'); ylabel('N\cdotm');
    title(sprintf('%s 多版本力矩', joint_names{j}));
    legend({'tau\_meas\_filt', 'tau\_NE\_rigid', 'tau\_NE+fric+Ia', 'tau\_id\_pred'}, 'Location', 'best');
    grid on; hold off;
end
sgtitle(fig_cmp, sprintf('滤波后力矩多版本对比（时间跨度 %.2f s）', t_span));

%% ========== 九、误差图 ==========
fig_err = figure('Name', '力矩误差对比', 'Position', [140, 140, 1200, 800]);
for j = 1:6
    subplot(2, 3, j);
    plot(t_equiv_plot, err_ne(:, j),      'b--', 'LineWidth', 1.0); hold on;
    plot(t_equiv_plot, err_ne_plus(:, j), 'g-.', 'LineWidth', 1.0);
    plot(t_equiv_plot, err_id(:, j),      'r-',  'LineWidth', 1.2);
    xlabel('t (s)'); ylabel('N\cdotm');
    title(sprintf('%s 误差 (模型 - tau\\_meas\\_filt)', joint_names{j}));
    legend({'NE\_rigid - meas', 'NE+fric+Ia - meas', 'id\_pred - meas'}, 'Location', 'best');
    grid on; hold off;
end
sgtitle(fig_err, sprintf('滤波后力矩误差对比（基准 = tau\\_meas\\_filt，时间跨度 %.2f s）', t_span));

fprintf('已绘制：力矩多版本对比、力矩误差对比\n');

end
