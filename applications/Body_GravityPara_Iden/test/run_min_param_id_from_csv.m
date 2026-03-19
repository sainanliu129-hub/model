%% run_min_param_id_from_csv  从采集 CSV 跑通「低通预处理 + 连续窗 → 最小参数辨识 → FD 验证」
%
% 数据文件放在 applications/Body_GravityPara_Iden/data/ 下，例如：
%   data/excitation/  激励辨识用（推荐）
%   data/跑步/        与现有对比脚本一致
%
% 用法：
%   1. 把采集好的 CSV 放到 data/excitation/（或 data/跑步/），CSV 格式需与 read_leg_joint_csv 兼容。
%   2. 在 MATLAB 中 cd 到 applications/Body_GravityPara_Iden，直接点运行即可（csv_file 留空则自动选第一个 CSV）。
%   3. 若要用指定文件，在下方「配置」处设 csv_file = 'data/excitation/你的文件.csv'。
%
% 当前仅保留连续窗模式（已移除周期平均）：
%   先低通预处理（q 低通+差分得到 qd/qdd，tau 低通）再连续窗辨识。
%
% 输出：辨识用数据图、最小参数 X_hat、index_base、辨识误差图；可选保存 min_param_id_result.mat 供 FD 复用。
%
% 摩擦与转子惯量补偿已并入“数据预处理可选项”：
%   - 统一入口：run_id_preprocess_pipeline（滤波 + 可选补偿）
%   - 在 opts.prep_opts 中设置：
%       do_compensation=true/false（默认 false）
%       J_eq / friction_params / tau_bias（手动）
%       或 load_friction_from_summary=true（自动从汇总表读取）
%   - 本脚本的辨识主流程（identify_min）不再单独处理摩擦/转子补偿。

function run_min_param_id_from_csv(csv_file, opts)

clc;
close all;

if nargin < 2, opts = struct(); end
if nargin < 1, csv_file = []; end
if ~isfield(opts, 'continuous_opts'), opts.continuous_opts = struct(); end
% 辨识前预处理：统一低通链路（q 低通+差分，tau 低通），默认开启
if ~isfield(opts, 'use_preprocess_id'), opts.use_preprocess_id = true; end
if ~isfield(opts, 'prep_opts'), opts.prep_opts = struct(); end
prep_opts_used = [];   % 若做了预处理则写入实际选项，供后续 FD/ID 验证脚本复用
co = opts.continuous_opts;
if ~isfield(co, 't_start_s'),         co.t_start_s = 2.1; end
if ~isfield(co, 't_end_s'),           co.t_end_s   = 4.1; end
if ~isfield(co, 'qd_lowpass_fc_Hz'),  co.qd_lowpass_fc_Hz = 0; end
if ~isfield(co, 'qdd_smooth_half'),   co.qdd_smooth_half  = 0; end
opts.continuous_opts = co;

%% 路径与默认数据文件（本脚本在 test/ 下，app_root = Body_GravityPara_Iden）
app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
ensure_body_gravity_para_iden_path();

% 若未传入或留空 csv_file，依次尝试：data/excitation/*.csv、data/跑步/*.csv（默认选最新的一个）
if nargin < 1 || isempty(csv_file)
    dir1 = fullfile(app_root, 'data', 'excitation');
    dir2 = fullfile(app_root, 'data', '跑步');
    if isfolder(dir1)
        f = dir(fullfile(dir1, '*.csv'));
        if ~isempty(f)
            [~, idx] = sort([f.datenum], 'descend');
            f = f(idx);
            csv_file = fullfile(dir1, f(1).name);
        end
    end
    if isempty(csv_file) && isfolder(dir2)
        f = dir(fullfile(dir2, '*.csv'));
        if ~isempty(f)
            [~, idx] = sort([f.datenum], 'descend');
            f = f(idx);
            csv_file = fullfile(dir2, f(1).name);
        end
    end
    if isempty(csv_file)
        error(['未找到 CSV 文件。请：\n' ...
               '  1) 将采集的 5 周期激励 CSV 放到 %s 或 %s\n' ...
               '  2) 或调用 run_min_param_id_from_csv(''path/to/your.csv'')'], dir1, dir2);
    end
end

% 若传入的是 string/相对路径，则统一转为 char 并相对于 app_root 解析
if isstring(csv_file)
    csv_file = char(csv_file);
end
if ischar(csv_file) && ~isempty(csv_file) && exist(csv_file, 'file') ~= 2
    csv_file = fullfile(app_root, csv_file);
end
if ~exist(csv_file, 'file')
    error('文件不存在: %s', csv_file);
end

fprintf('===== 最小参数辨识流程（低通预处理 + 连续窗 → 辨识 → FD） =====\n');
fprintf('数据文件: %s\n', csv_file);

%% 读取数据（单腿：左腿 6 关节）
data = read_leg_joint_csv(csv_file);
t   = data.time(:);
q   = data.pos_leg_l;    % N×6
qd  = data.vel_leg_l;   % N×6
tau = data.torque_leg_l; % N×6

if size(q,1) ~= length(t) || size(tau,1) ~= length(t)
    error('时间与 q/tau 行数不一致。');
end

% 可选：辨识前预处理（统一低通链路）
qdd_for_win = [];   % 若预处理则传入 qdd_s，避免窗内重算
if opts.use_preprocess_id
    prep = opts.prep_opts;
    if ~isfield(prep, 't_start_s'), prep.t_start_s = 2.1; end
    if ~isfield(prep, 't_end_s'),   prep.t_end_s   = 4.1; end
    if ~isfield(prep, 'q_lowpass_fc_Hz'), prep.q_lowpass_fc_Hz = 25; end
    if ~isfield(prep, 'q_lowpass_order'), prep.q_lowpass_order = 2; end
    if ~isfield(prep, 'tau_lowpass_fc_Hz'), prep.tau_lowpass_fc_Hz = 25; end
    if ~isfield(prep, 'tau_lowpass_order'), prep.tau_lowpass_order = 2; end
    if ~isfield(prep, 'do_plot'),   prep.do_plot = false; end

    [t, q, qd, qdd_for_win, tau, aux_prep, prep_opts_used] = run_id_preprocess_pipeline(t, q, qd, tau, prep);
    if isfield(prep_opts_used, 'do_compensation') && prep_opts_used.do_compensation
        fprintf('  已做预处理（低通+可选补偿）：q低通+差分，τ低通，且已启用补偿项\n');
    else
        fprintf('  已做预处理（仅低通）：q低通+差分，τ低通（未启用补偿）\n');
    end
end

% 若 CSV 中力矩全为零（未采集力矩），可用 URDF 逆动力学生成仿真 τ 做流程验证
if max(abs(tau(:))) < 1e-6
    warning('CSV 中关节力矩近似为 0，将用 URDF 逆动力学生成仿真 τ 做流程验证。');
    N = size(q, 1);
    qdd_raw = zeros(N, 6);
    dt = median(diff(t));
    if dt <= 0 || isnan(dt), dt = 0.002; end
    for j = 1:6
        for k = 1:N
            qdd_raw(k,j) = central_diff_point(qd(:,j), t, k, dt);
        end
    end
    tau = zeros(N, 6);
    for k = 1:N
        tau(k,:) = compute_e1_limb_dynamics('left_leg', q(k,:)', qd(k,:)', qdd_raw(k,:)')';
    end
end

%% 原始输入 qdd（用于绘图：预处理时用 qdd_s，否则由 qd 中心差分）
N = length(t);
dt = median(diff(t));
if dt <= 0 || isnan(dt), dt = 0.002; end
if ~isempty(qdd_for_win)
    qdd_raw = qdd_for_win;
else
    qdd_raw = zeros(N, 6);
    for j = 1:6
        for k = 1:N
            qdd_raw(k,j) = central_diff_point(qd(:,j), t, k, dt);
        end
    end
end

%% 图1：输入轨迹（时间 t — 位置、速度、加速度、力矩）
fig1 = figure('Name', '输入轨迹_q_qd_qdd_tau', 'Position', [50, 50, 1200, 800]);
joint_names = {'leg\_l1', 'leg\_l2', 'leg\_l3', 'leg\_l4', 'leg\_l5', 'leg\_l6'};
for j = 1:6
    subplot(4, 6, j);
    plot(t, q(:,j), 'b-'); xlabel('t (s)'); ylabel('rad'); title(joint_names{j}); grid on;
    subplot(4, 6, 6 + j);
    plot(t, qd(:,j), 'b-'); xlabel('t (s)'); ylabel('rad/s'); grid on;
    subplot(4, 6, 12 + j);
    plot(t, qdd_raw(:,j), 'b-'); xlabel('t (s)'); ylabel('rad/s^2'); grid on;
    subplot(4, 6, 18 + j);
    plot(t, tau(:,j), 'b-'); xlabel('t (s)'); ylabel('N\cdotm'); grid on;
end
sgtitle(fig1, '输入轨迹（原始采集）：q, qd, qdd, \tau');

%% 1. 辨识用数据：连续窗
fprintf('\n--- Step 1: 连续窗（高 |qdd| 降权）---\n');
win_opts = opts.continuous_opts;
if opts.use_preprocess_id
    % 预处理已做时间窗与 q/qd/qdd 同源，仅做降权、不再重算 qdd
    win_opts = struct('t_start_s', [], 't_end_s', [], 'qd_lowpass_fc_Hz', 0, 'qdd_smooth_half', 0);
    fprintf('  注意：use_preprocess_id=true，连续窗时间窗/滤波由 prep_opts 接管，这里的 continuous_opts 仅用于权重相关配置。\n');
end
avg_data = continuous_window_id_data(t, q, qd, tau, win_opts, qdd_for_win);
t_equiv = avg_data.t_equiv;
M = size(avg_data.q_bar, 1);
fprintf('连续窗样本数 M = %d，时间范围 %.2f～%.2f s\n', M, t_equiv(1), t_equiv(end));
if M < 1
    error('辨识用数据点数为 0，请检查时间窗与数据有效段。');
end

%% Step1：裁减后轨迹辨识矩阵条件数（与 identify_min 同源：Y 列归一化 + QR 选列）
fprintf('\n--- Step1: 条件数（cond(Y_min) 最重要） ---\n');
try
    q_bar  = avg_data.q_bar;
    qd_bar = avg_data.qd_bar;
    qdd_bar = avg_data.qdd_bar;
    [Mm, nn] = size(q_bar);
    Y_full = [];
    for k = 1:Mm
        Y_one = ReMatrix_E1_limb_URDF('left_leg', q_bar(k,:), qd_bar(k,:), qdd_bar(k,:), 1, 1);
        Y_full = [Y_full; Y_one];
    end
    col_norm = sqrt(sum(Y_full.^2, 1));
    col_norm(col_norm < 1e-12) = 1;
    W_norm = Y_full ./ (ones(size(Y_full,1), 1) * col_norm);
    r_cond = rank(W_norm);
    [~, ~, piv_cond] = qr(W_norm, 'vector');
    index_base_cond = sort(piv_cond(1:r_cond));
    Y_min_cond = Y_full(:, index_base_cond);
    cond_Y_full = cond(Y_full);   % 注意：Y_full 为冗余全参回归矩阵，可能秩亏，仅作参考
    cond_Y_min  = cond(Y_min_cond);
    cond_Wmin   = cond(W_norm(:, index_base_cond));
    fprintf('  样本数 M = %d，时间 %.2f～%.2f s\n', Mm, t_equiv(1), t_equiv(end));
    fprintf('  cond(Y_full)     = %.4e  （全参 60 列，可能秩亏，仅作参考）\n', cond_Y_full);
    fprintf('  cond(Y_min)      = %.4e  （最小参数 %d 列）【最重要】\n', cond_Y_min, numel(index_base_cond));
    fprintf('  cond(W_min)      = %.4e  （列归一化后最小参数）\n', cond_Wmin);
catch ME
    fprintf('  条件数计算失败: %s\n', ME.message);
end
fprintf('-----------------------------------\n');

%% Step2: Regressor energy ||Y(:,i)||（列范数 ≈0 表示该参数未被激励）
fprintf('\n--- Step2: Regressor energy（各参数列范数） ---\n');
try
    if ~exist('Y_full', 'var')
        error('Y_full 未定义（Step1 条件数未成功时跳过）');
    end
    ncol = size(Y_full, 2);
    regressor_energy = sqrt(sum(Y_full.^2, 1));   % 1×ncol, ||Y(:,i)||_2
    fprintf('  全参列数 = %d，||Y(:,i)||_2 最小值 = %.4e，最大值 = %.4e\n', ncol, min(regressor_energy), max(regressor_energy));
    idx_near_zero = find(regressor_energy < 1e-6 * max(regressor_energy));
    if ~isempty(idx_near_zero)
        fprintf('  列范数近似为 0 的列（未被激励）: %s\n', mat2str(idx_near_zero));
    end
    fig_reg = figure('Name', 'Regressor_energy', 'Position', [160 160 1000 400]);
    bar(1:ncol, regressor_energy, 'FaceColor', [0.3 0.5 0.8]);
    xlabel('参数列 i'); ylabel('||Y(:,i)||_2');
    title('Regressor energy：列范数 \approx 0 表示该参数未被激励');
    grid on;
    hold on;
    if ~isempty(idx_near_zero)
        plot(idx_near_zero, regressor_energy(idx_near_zero), 'ro', 'MarkerSize', 8, 'LineWidth', 1.5);
        legend('||Y(:,i)||', '近似为 0（未激励）', 'Location', 'best');
    end
    hold off;
catch ME
    fprintf('  Regressor energy 计算/绘图失败: %s\n', ME.message);
end
fprintf('-----------------------------------\n');

%% 图2：辨识用数据（等效时间 = 连续窗时间）
fig2 = figure('Name', '辨识用数据', 'Position', [100, 100, 1200, 800]);
for j = 1:6
    subplot(4, 6, j);
    plot(t_equiv, avg_data.q_bar(:,j), 'b-'); xlabel('t (s)'); ylabel('rad'); title(joint_names{j}); grid on;
    subplot(4, 6, 6 + j);
    plot(t_equiv, avg_data.qd_bar(:,j), 'b-'); xlabel('t (s)'); ylabel('rad/s'); grid on;
    subplot(4, 6, 12 + j);
    plot(t_equiv, avg_data.qdd_bar(:,j), 'b-'); xlabel('t (s)'); ylabel('rad/s^2'); grid on;
    subplot(4, 6, 18 + j);
    plot(t_equiv, avg_data.tau_bar(:,j), 'b-'); xlabel('t (s)'); ylabel('N\cdotm'); grid on;
end
sgtitle(fig2, '辨识用数据（continuous）：q, qd, qdd, \tau');

%% 2. 最小参数辨识（与 test_ReMatrix_E1_limb_URDF 一致：普通最小二乘，不用 WLS）
fprintf('\n--- Step 2: 最小参数辨识 ---\n');
opts_id = struct('use_wls', false);   % 与验收脚本一致，保证同轨迹下 τ_pred 与 test_ReMatrix 的 Y_min*X_hat_meas 一致
% 摩擦/转子补偿已在 preprocess 阶段通过 prep_opts 控制，这里不再重复扣除。
[X_hat, index_base, metrics] = identify_min(avg_data, 'left_leg', opts_id);
fprintf('最小参数维度 p_min = %d\n', numel(X_hat));
fprintf('各关节力矩 RMSE (N·m): '); fprintf(' %.4f', metrics.rmse_per_joint); fprintf('\n');
fprintf('各关节最大误差 (N·m): '); fprintf(' %.4f', metrics.maxerr_per_joint); fprintf('\n');

% 自检：用同一批数据直接 Y_min\b，应与 identify_min 的 X_hat 一致
[M_id, n_id] = size(avg_data.q_bar);
YY_check = [];
for k = 1:M_id
    Y_one = ReMatrix_E1_limb_URDF('left_leg', avg_data.q_bar(k,:), avg_data.qd_bar(k,:), avg_data.qdd_bar(k,:), 1, 1);
    YY_check = [YY_check; Y_one];
end
Y_min_check = YY_check(:, index_base);
tau_bar_check = avg_data.tau_bar;
b_check = reshape(tau_bar_check', [], 1);
X_hat_direct = Y_min_check \ b_check;
fprintf('自检: |X_hat - (Y_min\\b)| = %.2e (应≈0)\n', norm(X_hat - X_hat_direct));
residual = Y_min_check * X_hat - b_check;
fprintf('残差: ||Y_min*X_hat - b|| = %.4f, ||b|| = %.4f, 相对残差 = %.4f\n', ...
    norm(residual), norm(b_check), norm(residual)/max(norm(b_check),1e-12));
if norm(residual)/max(norm(b_check),1e-12) > 0.5
    fprintf('提示: 相对残差很大，说明刚体模型 Y_min*X_hat 难以拟合目标力矩。请检查：CSV 力矩列序/单位/符号、或 q/qd/qdd 与 τ 是否同一时刻。\n');
end

%% 3. 辨识结果图：τ 预测 vs 测量（横轴 = 等效时间）
t_span = t_equiv(end) - t_equiv(1);
fig = figure('Name', '最小参数辨识_力矩预测vs测量');
for j = 1:6
    subplot(2, 3, j);
    plot(t_equiv, metrics.tau_meas(:,j), 'b-', 'DisplayName', '\tau\_meas'); hold on;
    plot(t_equiv, metrics.tau_pred(:,j), 'r--', 'DisplayName', '\tau\_pred');
    xlabel('t (s)'); ylabel('N\cdotm');
    title(sprintf('%s RMSE=%.3f', joint_names{j}, metrics.rmse_per_joint(j)));
    legend('Location', 'best'); grid on;
end
sgtitle(sprintf('最小参数辨识：测量 vs 预测（时间跨度 %.2f s）', t_span));

%% 3b. 多版本力矩对比：牛顿-欧拉刚体 / 刚体+摩擦+转子 / 辨识力矩 vs 实际力矩
if opts.use_preprocess_id && exist('aux_prep', 'var')
    % 以预处理后的零相位力矩 tau_s 作为“实际力矩”（已滤波、未减摩擦/转子）
    tau_meas_win = aux_prep.tau_s;
    M_meas = size(tau_meas_win, 1);
    if M_meas ~= M
        % 时间长度不一致时，简单截断到共同长度
        L = min(M_meas, M);
        tau_meas_win = tau_meas_win(1:L, :);
        t_equiv_plot = t_equiv(1:L);
        q_bar_plot   = avg_data.q_bar(1:L, :);
        qd_bar_plot  = avg_data.qd_bar(1:L, :);
        qdd_bar_plot = avg_data.qdd_bar(1:L, :);
        tau_pred_id  = metrics.tau_pred(1:L, :);
    else
        t_equiv_plot = t_equiv;
        q_bar_plot   = avg_data.q_bar;
        qd_bar_plot  = avg_data.qd_bar;
        qdd_bar_plot = avg_data.qdd_bar;
        tau_pred_id  = metrics.tau_pred;
    end
    M_plot = size(q_bar_plot, 1);

    % 1) 牛顿-欧拉刚体力矩（与 test_ReMatrix_E1_limb_URDF 一致：inverseDynamics(robot_limb, q, qd, qdd)）
    [robot_limb, ~] = get_e1_limb_robot('left_leg');
    tau_ne = zeros(M_plot, 6);
    for k = 1:M_plot
        q_k   = q_bar_plot(k, :);
        qd_k  = qd_bar_plot(k, :);
        qdd_k = qdd_bar_plot(k, :);
        tau_ref_k = inverseDynamics(robot_limb, q_k, qd_k, qdd_k);
        tau_ne(k, :) = tau_ref_k(:)';
    end

    % 2) 牛顿-欧拉 + 预处理补偿项（来自 prep_opts/aux_prep）
    tau_comp_ne = zeros(M_plot, 6);
    if isfield(aux_prep, 'tau_comp') && ~isempty(aux_prep.tau_comp)
        if size(aux_prep.tau_comp, 1) >= M_plot
            tau_comp_ne = aux_prep.tau_comp(1:M_plot, :);
        else
            tau_comp_ne(1:size(aux_prep.tau_comp, 1), :) = aux_prep.tau_comp;
        end
    end
    tau_ne_plus = tau_ne + tau_comp_ne;

    % 3) 误差（全部以滤波后的“实际力矩” tau_meas_win 为基准）
    err_ne      = tau_ne      - tau_meas_win;
    err_ne_plus = tau_ne_plus - tau_meas_win;
    err_id      = tau_pred_id - tau_meas_win;

    % 4) 力矩对比图
    fig_cmp = figure('Name', '力矩多版本对比', 'Position', [120, 120, 1200, 800]);
    for j = 1:6
        subplot(2, 3, j);
        plot(t_equiv_plot, tau_meas_win(:, j), 'k-', 'LineWidth', 1.2); hold on;
        plot(t_equiv_plot, tau_ne(:, j),      'b--', 'LineWidth', 1.0);
        plot(t_equiv_plot, tau_ne_plus(:, j), 'g-.', 'LineWidth', 1.0);
        plot(t_equiv_plot, tau_pred_id(:, j), 'r:', 'LineWidth', 1.2);
        xlabel('t (s)'); ylabel('N\cdotm');
        title(sprintf('%s 多版本力矩', joint_names{j}));
        legend({'tau\_meas\_filt', 'tau\_NE\_rigid', 'tau\_NE+prep\_comp', 'tau\_id\_pred'}, ...
               'Location', 'best');
        grid on; hold off;
    end
    sgtitle(fig_cmp, sprintf('滤波后力矩多版本对比（时间跨度 %.2f s）', t_span));

    % 5) 误差图
    fig_err = figure('Name', '力矩误差对比', 'Position', [140, 140, 1200, 800]);
    for j = 1:6
        subplot(2, 3, j);
        plot(t_equiv_plot, err_ne(:, j),      'b--', 'LineWidth', 1.0); hold on;
        plot(t_equiv_plot, err_ne_plus(:, j), 'g-.', 'LineWidth', 1.0);
        plot(t_equiv_plot, err_id(:, j),      'r-',  'LineWidth', 1.2);
        xlabel('t (s)'); ylabel('N\cdotm');
        title(sprintf('%s 误差 (模型 - tau\\_meas\\_filt)', joint_names{j}));
        legend({'NE\_rigid - meas', 'NE+prep\_comp - meas', 'id\_pred - meas'}, ...
               'Location', 'best');
        grid on; hold off;
    end
    sgtitle(fig_err, sprintf('滤波后力矩误差对比（基准 = tau\\_meas\\_filt，时间跨度 %.2f s）', t_span));
end

%% 4. 任选一点做 FD 验证（可选）
k_test = min(round(size(avg_data.q_bar,1)/2), size(avg_data.q_bar,1));
q_test  = avg_data.q_bar(k_test, :)';
qd_test = avg_data.qd_bar(k_test, :)';
tau_test = avg_data.tau_bar(k_test, :)';

% FD 验证的输入力矩必须与辨识目标一致：
%   - 预处理阶段若启用补偿，avg_data.tau_bar 已是 τ_id，可直接使用。

qdd_fd = forward_dynamics_min(q_test, qd_test, tau_test, X_hat, index_base, 'left_leg', 1);
fprintf('\n--- Step 3: FD 验证（相位点 %d）---\n', k_test);
fprintf('forward_dynamics_min 输出 qdd (rad/s^2): '); fprintf(' %.4f', qdd_fd); fprintf('\n');

%% 5. 可选：保存结果供后续 FD/ID 验证使用（含 prep_opts 便于复现同一预处理）
out_mat = fullfile(app_root, 'min_param_id_result.mat');
if ~isempty(prep_opts_used)
    prep_opts = prep_opts_used;   % 保存为 prep_opts 供 run_full_dynamics_validation / 调试脚本复用
    save(out_mat, 'X_hat', 'index_base', 'avg_data', 'metrics', 'prep_opts');
    fprintf('\n已保存 X_hat, index_base, avg_data, metrics, prep_opts 到: %s\n', out_mat);
else
    save(out_mat, 'X_hat', 'index_base', 'avg_data', 'metrics');
    fprintf('\n已保存 X_hat, index_base 到: %s\n', out_mat);
end
fprintf('后续可加载后调用: qdd = forward_dynamics_min(q, qd, tau, X_hat, index_base, ''left_leg'', 1);\n');

end

% 单点中心差分已统一到 utility_function/central_diff_point.m
