%% run_min_param_id_from_csv  从采集 CSV 跑通「周期平均 或 连续窗 → 最小参数辨识 → FD 验证」
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
% 辨识模式（opts.mode）：
%   'continuous'（默认）- 只取激励段 + qd 低通 + qdd SG 平滑 → 辨识，快、不对齐。
%     continuous_opts 默认与 test_ReMatrix_E1_limb_URDF 一致：t_start_s=2, t_end_s=4, qd_lowpass_fc_Hz=50, qdd_smooth_half=15。
%     未指定字段时用上述默认，保证同轨迹下 run_min_param_id 与 test_ReMatrix 验收一致。
%   'cycle_average' - 周期对齐 + 相位平均，需 K、T。
%
% 输出：辨识用数据图、最小参数 X_hat、index_base、辨识误差图；可选保存 min_param_id_result.mat 供 FD 复用。
%
% 摩擦与转子惯量补偿（推荐先做 Friction_Iden 再辨识刚体参数）：
%   (1) 从汇总表自动读：opts.load_friction_from_summary = true；从 Friction_Iden 的 data/build 下
%       「全部电机参数汇总.xlsx」读取 stribeck_viscous 参数与 I_a，缺省或 NaN 填 0。
%   (2) 手动传入：opts.friction_params（含 .model='stribeck_viscous' 或 tau_c_pos/neg、b）、opts.I_a。
%   示例：直接点运行用默认；或 run_min_param_id_from_csv('data/excitation/你的文件.csv', opts)

function run_min_param_id_from_csv(csv_file, opts)

clc;
close all;

if nargin < 2, opts = struct(); end
if nargin < 1, csv_file = []; end
if ~isfield(opts, 'K'), opts.K = 5; end
if ~isfield(opts, 'T'), opts.T = 8; end
if ~isfield(opts, 'trim_ramp')
    opts.trim_ramp = (opts.K == 1) * 0 + (opts.K > 1) * 1.0;  % 单周期不裁，多周期默认裁 1s 过渡段
end
if ~isfield(opts, 'anchor_joint'), opts.anchor_joint = 0; end  % 0=自动选幅值最大关节
if ~isfield(opts, 'anchor_type'), opts.anchor_type = 'fixed_period'; end  % 'fixed_period'=按生成激励的 T 切分，与单周期一致
% 可选：用生成激励的单周期作为相位网格（ref_t, ref_q, ref_qd）；未提供则尝试同目录 *_ref.mat
if ~isfield(opts, 'ref_t'), opts.ref_t = []; end
if ~isfield(opts, 'ref_q'), opts.ref_q = []; end
if ~isfield(opts, 'ref_qd'), opts.ref_qd = []; end
% 辨识模式：'continuous'=整段滤波+辨识（默认，快、不对齐不裁剪）；'cycle_average'=周期对齐平均
if ~isfield(opts, 'mode'), opts.mode = 'continuous'; end
if ~isfield(opts, 'continuous_opts'), opts.continuous_opts = struct(); end
% 摩擦与转子惯量补偿：传入则在辨识前用 τ_rb = τ_meas - τ_friction - τ_rotor
if ~isfield(opts, 'friction_params'), opts.friction_params = []; end
if ~isfield(opts, 'I_a'), opts.I_a = []; end
% 若从「全部电机参数汇总」表读取 stribeck_viscous + I_a（data/build 下 xlsx）
if ~isfield(opts, 'load_friction_from_summary'), opts.load_friction_from_summary = false; end
if ~isfield(opts, 'n_joints'), opts.n_joints = 6; end
if ~isfield(opts, 'row_for_joint'), opts.row_for_joint = []; end  % 关节→表行映射，0=该关节用0
% 辨识前预处理：与 run_torque_comparison 一致（SG 求导 + τ 低通 + 摩擦/转子补偿），再连续窗辨识；默认关闭
if ~isfield(opts, 'use_preprocess_id'), opts.use_preprocess_id = false; end
if ~isfield(opts, 'prep_opts'), opts.prep_opts = struct(); end
prep_opts_used = [];   % 若做了预处理则写入实际选项，供后续 FD/ID 验证脚本复用
% 与 test_ReMatrix_E1_limb_URDF 的 csv_continuous_opts 一致，保证同轨迹下辨识与验收一致（尤其 qdd_smooth_half 避免 τ_pred 噪声）
if strcmpi(opts.mode, 'continuous')
    co = opts.continuous_opts;
    % 默认与说明保持一致：t_start_s=2.0, t_end_s=4.0, qd_lowpass_fc_Hz=50, qdd_smooth_half=15
    if ~isfield(co, 't_start_s'),         co.t_start_s = 2.0; end
    if ~isfield(co, 't_end_s'),           co.t_end_s   = 4.0; end
    if ~isfield(co, 'qd_lowpass_fc_Hz'),  co.qd_lowpass_fc_Hz = 50; end
    if ~isfield(co, 'qdd_smooth_half'),   co.qdd_smooth_half  = 15; end
    opts.continuous_opts = co;
end

%% 路径与默认数据文件（本脚本在 test/ 下，app_root = Body_GravityPara_Iden）
app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
ensure_body_gravity_para_iden_path();
% 若从汇总表加载摩擦/惯量，需 Friction_Iden 的 load_friction_stribeck_from_summary
if opts.load_friction_from_summary
    friction_iden_dir = fullfile(app_root, '..', 'Friction_Iden');
    if isfolder(friction_iden_dir) && isempty(which('load_friction_stribeck_from_summary'))
        addpath(friction_iden_dir);
    end
    load_opts = struct();
    if ~isempty(opts.row_for_joint), load_opts.row_for_joint = opts.row_for_joint; end
    [opts.friction_params, opts.I_a] = load_friction_stribeck_from_summary(opts.n_joints, load_opts);
    fprintf('  已从「全部电机参数汇总」表加载 stribeck_viscous 摩擦参数与 I_a（n_joints=%d）\n', opts.n_joints);
end

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

fprintf('===== 最小参数辨识流程（周期平均 → 辨识 → FD） =====\n');
fprintf('数据文件: %s\n', csv_file);

% 若未在 opts 中提供 ref_t/ref_q，尝试从同目录 *_ref.mat 或 excitation_ref.mat 加载（与生成激励一致）
ref_t = opts.ref_t;
ref_q = opts.ref_q;
ref_qd = opts.ref_qd;
if isempty(ref_t) || isempty(ref_q)
    csv_dir = fileparts(csv_file);
    [~, base] = fileparts(csv_file);
    ref_mat = fullfile(csv_dir, [base '_ref.mat']);
    if exist(ref_mat, 'file')
        ld = load(ref_mat);
        if isempty(ref_t) && isfield(ld, 'ref_t'), ref_t = ld.ref_t; end
        if isempty(ref_q) && isfield(ld, 'ref_q'), ref_q = ld.ref_q; end
        if isempty(ref_qd) && isfield(ld, 'ref_qd'), ref_qd = ld.ref_qd; end
        if ~isempty(ref_t) || ~isempty(ref_q), fprintf('已加载参考网格: %s\n', ref_mat); end
    end
    if (isempty(ref_t) || isempty(ref_q)) && exist(fullfile(csv_dir, 'excitation_ref.mat'), 'file')
        ld = load(fullfile(csv_dir, 'excitation_ref.mat'));
        if isempty(ref_t) && isfield(ld, 'ref_t'), ref_t = ld.ref_t; end
        if isempty(ref_q) && isfield(ld, 'ref_q'), ref_q = ld.ref_q; end
        if isempty(ref_qd) && isfield(ld, 'ref_qd'), ref_qd = ld.ref_qd; end
        if ~isempty(ref_t) || ~isempty(ref_q), fprintf('已加载参考网格: excitation_ref.mat\n'); end
    end
end

%% 读取数据（单腿：左腿 6 关节）
data = read_leg_joint_csv(csv_file);
t   = data.time(:);
q   = data.pos_leg_l;    % N×6
qd  = data.vel_leg_l;   % N×6
tau = data.torque_leg_l; % N×6

if size(q,1) ~= length(t) || size(tau,1) ~= length(t)
    error('时间与 q/tau 行数不一致。');
end

% 可选：辨识前预处理（与 run_torque_comparison 一致：SG 求导、τ 低通、摩擦/转子补偿）
qdd_for_win = [];   % 若预处理则传入 qdd_s，避免窗内重算
if opts.use_preprocess_id
    prep = opts.prep_opts;
    if ~isfield(prep, 't_start_s'), prep.t_start_s = 2.1; end   % 与 run_torque_comparison 一致
    if ~isfield(prep, 't_end_s'),   prep.t_end_s   = 4.1; end
    if ~isfield(prep, 'sg_order'),  prep.sg_order  = 4; end
    if ~isfield(prep, 'sg_frame'),  prep.sg_frame  = 23; end   % 与 run_torque_comparison 一致
    if ~isfield(prep, 'tau_lowpass_fc_Hz'), prep.tau_lowpass_fc_Hz = 15; end
    if ~isfield(prep, 'tau_lowpass_order'), prep.tau_lowpass_order = 2; end
    if ~isfield(prep, 'J_eq'),      prep.J_eq = opts.I_a; end
    if ~isfield(prep, 'friction_params'), prep.friction_params = opts.friction_params; end
    if ~isfield(prep, 'do_plot'),   prep.do_plot = false; end
    [t, q, qd, qdd_for_win, tau, aux_prep] = preprocess_id_data(t, q, qd, tau, prep);
    prep_opts_used = prep;
    fprintf('  已做辨识前预处理（与 run_torque_comparison 一致）：SG(q)→q/qd/qdd，τ 低通，τ_id = τ_s − J_eq·qdd − τ_f − τ_bias\n');
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

%% 周期参数（与激励轨迹生成时一致）
K = opts.K;      % 周期数
T = opts.T;      % 单周期时长 (s)，若未知可改为 [] 由数据估计

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

%% 1. 辨识用数据：周期平均 或 连续窗（单周期+衔接段）
use_continuous = strcmpi(opts.mode, 'continuous');
if use_continuous
    fprintf('\n--- Step 1: 连续窗（掐头去尾 + qdd 平滑 + 高 |qdd| 降权）---\n');
    win_opts = opts.continuous_opts;
    if opts.use_preprocess_id
        % 预处理已做时间窗与 q/qd/qdd 同源，仅做降权、不再重算 qdd
        win_opts = struct('t_start_s', [], 't_end_s', [], 'qd_lowpass_fc_Hz', 0, 'qdd_smooth_half', 0);
        fprintf('  注意：use_preprocess_id=true，连续窗的时间窗/滤波设置由 prep_opts 接管，这里的 continuous_opts 仅用于权重相关配置。\n');
    end
    avg_data = continuous_window_id_data(t, q, qd, tau, win_opts, qdd_for_win);
    t_equiv = avg_data.t_equiv;
    M = size(avg_data.q_bar, 1);
    fprintf('连续窗样本数 M = %d，时间范围 %.2f～%.2f s\n', M, t_equiv(1), t_equiv(end));
else
    fprintf('\n--- Step 1: 周期平均 ---\n');
    ca_args = {'qd', qd, 'trim_ramp', opts.trim_ramp, 'anchor_joint', opts.anchor_joint, 'anchor_type', opts.anchor_type};
    if ~isempty(ref_t) && ~isempty(ref_q)
        ca_args = [ca_args, {'ref_t', ref_t, 'ref_q', ref_q}];
        if ~isempty(ref_qd), ca_args = [ca_args, {'ref_qd', ref_qd}]; end
        fprintf('使用生成激励参考网格，M_ref = %d\n', numel(ref_t));
    end
    avg_data = cycle_average(t, q, tau, K, T, ca_args{:});
    phi = avg_data.phi;
    t_equiv = phi * T;
    M = size(avg_data.q_bar, 1);
    fprintf('相位网格点数 M = %d，trim_ramp = %.1f s，anchor_type = %s\n', M, opts.trim_ramp, opts.anchor_type);
end
if M < 1
    error('辨识用数据点数为 0。周期平均模式：请检查数据时间范围是否覆盖 t(1)+trim_ramp ～ t(1)+trim_ramp+K*T（fixed_period 下 t_start=t(1)+trim_ramp），或改用 mode=''continuous''。');
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

%% 图2：辨识用数据（等效时间 = 连续窗时间 或 φ*T）
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
sgtitle(fig2, sprintf('辨识用数据（%s）：q, qd, qdd, \\tau', opts.mode));

% 图2b：仅周期平均模式绘制多周期叠图
if ~use_continuous && K > 1
    fig2b = figure('Name', '5周期对齐与平均', 'Position', [150, 150, 1200, 800]);
    qd_cyc_plot = cell(K, 1);
    qdd_cyc = cell(K, 1);
    if M > 1
        dphi = 1 / (M - 1);
    else
        dphi = 1;
    end
    for c = 1:K
        qd_c = avg_data.per_cycle.qd{c};
        if isempty(qd_c)
            q_c = avg_data.per_cycle.q{c};
            qd_c = zeros(M, 6);
            for j = 1:6
                for i = 2:M-1
                    qd_c(i,j) = (q_c(i+1,j) - q_c(i-1,j)) / (2*dphi) / T;
                end
                qd_c(1,j) = qd_c(2,j);
                qd_c(M,j) = qd_c(M-1,j);
            end
        end
        qd_cyc_plot{c} = qd_c;
        qdd_cyc{c} = zeros(M, 6);
        for j = 1:6
            for i = 2:M-1
                qdd_cyc{c}(i,j) = (qd_c(i+1,j) - qd_c(i-1,j)) / (2*dphi) / T;
            end
            qdd_cyc{c}(1,j) = qdd_cyc{c}(2,j);
            qdd_cyc{c}(M,j) = qdd_cyc{c}(M-1,j);
        end
    end
    for j = 1:6
        subplot(4, 6, j);
        for c = 1:K
            plot(t_equiv, avg_data.per_cycle.q{c}(:,j), 'Color', [0.7 0.7 0.7], 'LineWidth', 0.6); hold on;
        end
        plot(t_equiv, avg_data.q_bar(:,j), 'b-', 'LineWidth', 1.5);
        xlabel('t (s)'); ylabel('rad'); title(joint_names{j}); grid on; hold off;
        subplot(4, 6, 6 + j);
        for c = 1:K
            plot(t_equiv, qd_cyc_plot{c}(:,j), 'Color', [0.7 0.7 0.7], 'LineWidth', 0.6); hold on;
        end
        plot(t_equiv, avg_data.qd_bar(:,j), 'b-', 'LineWidth', 1.5);
        xlabel('t (s)'); ylabel('rad/s'); grid on; hold off;
        subplot(4, 6, 12 + j);
        for c = 1:K
            plot(t_equiv, qdd_cyc{c}(:,j), 'Color', [0.7 0.7 0.7], 'LineWidth', 0.6); hold on;
        end
        plot(t_equiv, avg_data.qdd_bar(:,j), 'b-', 'LineWidth', 1.5);
        xlabel('t (s)'); ylabel('rad/s^2'); grid on; hold off;
        subplot(4, 6, 18 + j);
        for c = 1:K
            plot(t_equiv, avg_data.per_cycle.tau{c}(:,j), 'Color', [0.7 0.7 0.7], 'LineWidth', 0.6); hold on;
        end
        plot(t_equiv, avg_data.tau_bar(:,j), 'b-', 'LineWidth', 1.5);
        xlabel('t (s)'); ylabel('N\cdotm'); grid on; hold off;
    end
    sgtitle(fig2b, sprintf('多周期对齐 + 平均（灰=各周期，蓝=平均，等效时间 0～%.2f s）', T));
end

%% 2. 最小参数辨识（与 test_ReMatrix_E1_limb_URDF 一致：普通最小二乘，不用 WLS）
fprintf('\n--- Step 2: 最小参数辨识 ---\n');
opts_id = struct('use_wls', false);   % 与验收脚本一致，保证同轨迹下 τ_pred 与 test_ReMatrix 的 Y_min*X_hat_meas 一致
% 预处理时 avg_data.tau_bar 已是 τ_id = τ_s − J_eq·qdd − τ_f − τ_bias，不得再在 identify_min 中扣摩擦/转子（否则双重扣除）
if ~opts.use_preprocess_id
    if ~isempty(opts.friction_params), opts_id.friction_params = opts.friction_params; end
    if ~isempty(opts.I_a), opts_id.I_a = opts.I_a; end
end
has_fp_id = isfield(opts_id, 'friction_params') && ~isempty(opts_id.friction_params);
has_Ia_id = isfield(opts_id, 'I_a') && ~isempty(opts_id.I_a);
if has_fp_id || has_Ia_id
    fprintf('  辨识前将减去摩擦与转子惯量力矩（τ_rb = τ_meas - τ_friction - τ_rotor）\n');
end
[X_hat, index_base, metrics] = identify_min(avg_data, 'left_leg', opts_id);
fprintf('最小参数维度 p_min = %d\n', numel(X_hat));
fprintf('各关节力矩 RMSE (N·m): '); fprintf(' %.4f', metrics.rmse_per_joint); fprintf('\n');
fprintf('各关节最大误差 (N·m): '); fprintf(' %.4f', metrics.maxerr_per_joint); fprintf('\n');

% 自检：用同一批数据直接 Y_min\b，应与 identify_min 的 X_hat 一致；b 须与 identify_min 内一致（预处理时=tau_bar，未预处理且传摩擦/转子时=tau_bar-tau_comp）
[M_id, n_id] = size(avg_data.q_bar);
YY_check = [];
for k = 1:M_id
    Y_one = ReMatrix_E1_limb_URDF('left_leg', avg_data.q_bar(k,:), avg_data.qd_bar(k,:), avg_data.qdd_bar(k,:), 1, 1);
    YY_check = [YY_check; Y_one];
end
Y_min_check = YY_check(:, index_base);
tau_bar_check = avg_data.tau_bar;
if has_fp_id || has_Ia_id
    fp_for_check = []; if has_fp_id, fp_for_check = opts_id.friction_params; end
    Ia_for_check = []; if has_Ia_id, Ia_for_check = opts_id.I_a; end
    tau_comp_check = subtract_friction_rotor_torque(avg_data.qd_bar, avg_data.qdd_bar, fp_for_check, Ia_for_check);
    tau_bar_check = tau_bar_check - tau_comp_check;
end
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
if opts.use_preprocess_id && use_continuous && exist('aux_prep', 'var')
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

    % 2) 牛顿-欧拉 + 摩擦 + 转子惯量（使用 Friction_Iden 中的参数）
    tau_comp_ne = zeros(M_plot, 6);
    if ~isempty(opts.friction_params) || ~isempty(opts.I_a)
        % subtract_friction_rotor_torque 需要 qd、qdd、friction_params、I_a
        fp_for_ne = [];
        if ~isempty(opts.friction_params), fp_for_ne = opts.friction_params; end
        Ia_for_ne = [];
        if ~isempty(opts.I_a), Ia_for_ne = opts.I_a; end
        tau_comp_ne = subtract_friction_rotor_torque(qd_bar_plot, qdd_bar_plot, fp_for_ne, Ia_for_ne);
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
        legend({'tau\_meas\_filt', 'tau\_NE\_rigid', 'tau\_NE+fric+Ia', 'tau\_id\_pred'}, ...
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
        legend({'NE\_rigid - meas', 'NE+fric+Ia - meas', 'id\_pred - meas'}, ...
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
%   - use_preprocess_id=true 时，avg_data.tau_bar 已为 τ_id = τ_s − J_eq·qdd − τ_f − τ_bias，直接使用；
%   - use_preprocess_id=false 且辨识时扣除了摩擦/转子，则此处也需扣除一次，以免“模型拟合刚体力矩，FD 却喂原始力矩”。
if ~opts.use_preprocess_id && (has_fp_id || has_Ia_id)
    fp_for_fd = [];
    if has_fp_id, fp_for_fd = opts_id.friction_params; end
    Ia_for_fd = [];
    if has_Ia_id, Ia_for_fd = opts_id.I_a; end
    tau_comp_fd = subtract_friction_rotor_torque(avg_data.qd_bar(k_test, :), avg_data.qdd_bar(k_test, :), fp_for_fd, Ia_for_fd);
    tau_test = tau_test - tau_comp_fd(:);
end

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
