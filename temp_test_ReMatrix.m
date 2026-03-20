addpath(genpath(fullfile(pwd, 'applications', 'Body_GravityPara_Iden')));
addpath(genpath(fullfile(pwd, 'FrankaEmikaPandaDynModel', 'matlab')));
addpath(genpath(fullfile(pwd, 'utility_function')));
addpath(genpath(fullfile(pwd, 'Excitation_Trajectory_E1', 'excitation_trajectory')));

%% test_ReMatrix_E1_limb_URDF  辨识矩阵正确性验收
% 
% 在 URDF 一致环境下：用 inverseDynamics 得到 τ_ref，用 Y*θ（θ 从 URDF 惯性拼出）得到 τ_Y，
% 检查 |τ_ref − τ_Y| 是否在数值误差量级。若通过，说明 ReMatrix_E1_limb_URDF 的 base/相对变换/轴/链序 等实现正确。
% 
% 轨迹模式：trajectory_source = 'csv' 用辨识 CSV 窗内轨迹；'excitation' 用激励 ref.mat；'random' 用 20 随机点。
% 本脚本参考为 τ_ref = inverseDynamics（模型）；「测量 vs 预测」图来自 run_min_param_id_from_csv。若该图 τ_pred 噪声大，
% 请用 example_run_min_param_id（qdd_smooth_half=15）跑辨识。CSV 模式下会额外画 τ_meas vs Y_min*X_hat。
% 
% 运行：在 MATLAB 中 cd 到 Body_GravityPara_Iden 目录，或把该目录及 robot_model 加入路径后运行本脚本。

clc;clear;close all;
app_root = fullfile(fileparts(mfilename('fullpath')), '..');
exc_dir = fullfile(app_root, '..', 'Excitation_Trajectory_E1', 'excitation_trajectory');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
if isempty(which('get_e1_limb_robot'))
    addpath(fullfile(app_root, '..', '..', 'robot_model'));
end
if isempty(which('plot_compare_6dof'))
    addpath(fullfile(app_root, '..', '..', 'utility_function'));
end
ensure_body_gravity_par-iden_path();

limb = 'left_leg';
para_order = 1;   % 参数顺序 [m mx my mz Ixx Ixy Ixz Iyy Iyz Izz]
tol_norm = 5e-1;  % N·m，允许的最大 |τ_ref - τ_Y|

% debug_level 选择额外调试模式（仍保持脚本形式，由你手动改这里）：
%   0 - 只运行当前验收流程（原始行为）
%   1 - 额外运行 verify_error_source（零位重力 / 重力符号 / 链序+轴检查）
%   2 - 在 1 的基础上再运行 test_ReMatrix_vs_inverseDynamics_local（局部点对比）
debug_level = 1;

% 整机 URDF（用于与 limb subtree 的逆动力学对比）
[robot_full, n_full] = get_e1_full_robot();
[idx_ll, ~, ~, ~] = get_e1_full_robot_limb_indices();
idx_limb = idx_ll;   % 当前仅 left_leg

% 轨迹来源：'csv' = 从辨识用 CSV 取窗内轨迹；'excitation' = ref.mat 或现场生成；'random' = 20 随机点
trajectory_source = 'csv';

% CSV 轨迹开关：是否做与辨识一致的预处理（窗 + qd 低通 + qdd 平滑）
% true  = 与 example_run_min_param_id / run_min_param_id_from_csv 相同处理
% false = 仅按时间窗截取，直接使用 CSV 中的 q, qd，并用 gradient 在 t 上数值求导得到 qdd_raw
use_preprocess_csv = false;

% CSV 轨迹：文件名与时间窗设置
csv_file_validation = fullfile(app_root, 'data', 'excitation', 'excitation_trajectory_TLBO_E1_PD-M1-v0_multi_joint_20260304-081559.csv');
csv_continuous_opts = struct('t_start_s', 2, 't_end_s', 4, 'qd_lowpass_fc_Hz', 50, 'qdd_smooth_half', 15);

[robot_limb, n] = get_e1_limb_robot(limb);

% 默认保持原始行为：直接使用 URDF/CAD 全参 theta = get_limb_theta_from_URDF
theta  = get_limb_theta_from_URDF(robot_limb, para_order);

% 如需基于 β 恢复全参，可将此开关改为 true
use_beta_full = true;
if use_beta_full
    pi_cad = theta;
    result_mat = fullfile(app_root, 'min_param_id_result.mat');
    if isfile(result_mat)
        ld = load(result_mat);
        if isfield(ld, 'X_hat') && isfield(ld, 'index_base') && isfield(ld, 'avg_data')
            try
                q_bar_id  = ld.avg_data.q_bar;
                qd_bar_id = ld.avg_data.qd_bar;
                qdd_bar_id = ld.avg_data.qdd_bar;
                index_base = ld.index_base;

                Y_full_id = [];
                for k = 1:size(q_bar_id, 1)
                    Y_one = ReMatrix_E1_limb_URDF(limb, q_bar_id(k,:), qd_bar_id(k,:), qdd_bar_id(k,:), 1, para_order);
                    Y_full_id = [Y_full_id; Y_one]; %#ok<AGROW>
                end
                Y_min_id = Y_full_id(:, index_base);
                K = pinv(Y_min_id) * Y_full_id;

                beta_hat = ld.X_hat(:);
                [theta_rec, res_norm] = recover_full_params_from_beta(K, beta_hat, pi_cad, 1e-2);
                theta = theta_rec;
                fprintf('已从 min_param_id_result.mat 恢复全参数 θ（recover_full_params_from_beta，残差范数=%.3e）。\n', res_norm);
            catch ME
                warning('基于 β 恢复全参数失败（%s），回退到 CAD θ。', ME.message);
            end
        else
            warning('min_param_id_result.mat 中缺少 X_hat/index_base/avg_data 字段，回退到 CAD θ。');
        end
    else
        fprintf('未找到 %s，使用 CAD θ 进行 ReMatrix 验证。\n', result_mat);
    end
end

%% 动力学参数顺序核对（θ 与 ReMatrix 列序一致，便于排查全参误差）
% ReMatrix 与 get_limb_theta 约定：每连杆 [m mx my mz Ixx Ixy Ixz Iyy Iyz Izz]；
% MATLAB body.Inertia 为 [Ixx Iyy Izz Iyz Ixz Ixy]，已正确映射到上述顺序。
% 若全参 |τ_ref - Y*θ| 仍大，更可能是：重力方向/符号、Bodies 链序(base→tip)、或逆动力学实现细节，而非 θ 顺序。
if n >= 1
    b1 = robot_limb.Bodies{1};
    theta1 = theta(1:10);
    fprintf('===== 动力学参数顺序核对（首连杆） =====\n');
    fprintf('  body.Inertia (MATLAB 顺序 [Ixx Iyy Izz Iyz Ixz Ixy]): %s\n', mat2str(b1.Inertia(:)', 6));
    fprintf('  theta(1:10) ([m mx my mz Ixx Ixy Ixz Iyy Iyz Izz]):     %s\n', mat2str(theta1', 6));
    fprintf('  body.Mass=%.6g, CenterOfMass=%s\n', b1.Mass, mat2str(b1.CenterOfMass(:)', 6));
    fprintf('========================================\n');
end

%% 生成或加载 (q_all, qd_all, qdd_all)
q_all = []; qd_all = []; qdd_all = []; nTest = [];

if strcmp(trajectory_source, 'csv')
    % CSV 模式：
    % - use_preprocess_csv = true: 与 example_run_min_param_id 同轨迹（continuous_window_id_data）
    % - use_preprocess_csv = false: 仅时间窗 + gradient(qd) 求 qdd_raw，不做滤波和平滑
    if ~isfile(csv_file_validation)
        error('test_ReMatrix:CSV', 'CSV 不存在: %s', csv_file_validation);
    end
    data = read_leg_joint_csv(csv_file_validation);
    t = data.time(:);
    q = data.pos_leg_l;
    qd = data.vel_leg_l;
    tau = data.torque_leg_l;
    if use_preprocess_csv
        % 与辨识一致：continuous_window_id_data 做窗 + qd 低通 + qdd 平滑
        opts_win = struct('t_start_s', csv_continuous_opts.t_start_s, 't_end_s', csv_continuous_opts.t_end_s, ...
            'qd_lowpass_fc_Hz', csv_continuous_opts.qd_lowpass_fc_Hz, 'qdd_smooth_half', csv_continuous_opts.qdd_smooth_half);
        avg_data = continuous_window_id_data(t, q, qd, tau, opts_win);
        q_all = avg_data.q_bar;
        qd_all = avg_data.qd_bar;
        qdd_all = avg_data.qdd_bar;
        tau_meas_all = avg_data.tau_bar;   % 同窗内实测力矩，用于 CSV 模式下「测量 vs 预测」对照
        nTest = size(q_all, 1);
        fprintf('已从 CSV 加载窗内轨迹(预处理 on): %s，点数 %d，窗 [%.2f, %.2f] s\n', ...
            csv_file_validation, nTest, csv_continuous_opts.t_start_s, csv_continuous_opts.t_end_s);
    else
        % 仅时间窗 + gradient(qd) 求 qdd_raw，不做滤波/平滑
        t = t(:);
        if any(diff(t) <= 0) || any(isnan(t))
            error('test_ReMatrix:CSV', '时间戳异常，无法直接用 CSV 原始数据。');
        end
        % 按时间窗截取
        mask = (t >= csv_continuous_opts.t_start_s) & (t <= csv_continuous_opts.t_end_s);
        t_win = t(mask);
        q_win = q(mask, :);
        qd_win = qd(mask, :);
        tau_win = tau(mask, :);
        % 用 gradient 在 t 上求 qdd_raw
        qdd_win = zeros(size(qd_win));
        for j = 1:size(qd_win, 2)
            qdd_win(:, j) = gradient(qd_win(:, j), t_win);
        end
        q_all        = q_win;
        qd_all       = qd_win;
        qdd_all      = qdd_win;
        tau_meas_all = tau_win;   % 预处理 off 时也记录测量力矩，供图3/4/5 使用
        nTest = size(q_all, 1);
        fprintf('已从 CSV 加载窗内轨迹(预处理 off): %s，点数 %d，窗 [%.2f, %.2f] s\n', ...
            csv_file_validation, nTest, csv_continuous_opts.t_start_s, csv_continuous_opts.t_end_s);
    end
end

if isempty(q_all) && strcmp(trajectory_source, 'excitation')
    % 优先从 excitation_trajectory 下已保存的 *_ref.mat 加载
    ref_q = []; ref_qd = []; ref_qdd = []; ref_t = [];
    ref_candidates = {
        fullfile(exc_dir, 'excitation_ref.mat')
        fullfile(exc_dir, 'excitation_trajectory_standalone_ref.mat')
    };
    for ii = 1:numel(ref_candidates)
        ref_mat = ref_candidates{ii};
        if exist(ref_mat, 'file')
            ld = load(ref_mat);
            if isfield(ld, 'ref_q') && isfield(ld, 'ref_qd')
                ref_q = ld.ref_q;
                ref_qd = ld.ref_qd;
                ref_t = ld.ref_t;
                if isfield(ld, 'ref_acc')
                    ref_qdd = ld.ref_acc;
                else
                    ref_t = ref_t(:);
                    dt = median(diff(ref_t));
                    if dt <= 0 || isnan(dt), dt = 0.01; end
                    ref_qdd = zeros(size(ref_qd));
                    for j = 1:size(ref_qd, 2)
                        ref_qdd(:, j) = gradient(ref_qd(:, j), ref_t);
                    end
                end
                fprintf('已加载激励参考: %s，点数 %d\n', ref_mat, size(ref_q, 1));
                break;
            end
        end
    end
    if isempty(ref_q)
        addpath(exc_dir);
        config = struct();
        config.move_axis = [0, 1, 2, 3, 4, 5];
        config.init_joint = [0; 0; 0; 0; 0; 0];
        config.upper_joint_bound = [0.61; 0.436; 0.785; 2.44; 0.523; 0.262];
        config.lower_joint_bound = [-0.61; -0.261; -2.09; 0; -0.872; -0.262];
        config.max_velocity = [16.75; 20.1; 20.1; 13.18; 12.46; 12.46];
        config.max_acceleration = [200; 200; 200; 200; 200; 200];
        config.period = 8;
        config.sample_frequency = 100;
        config.order = 4;
        config.sample_number = 15;
        config.enable_collision_check = false;
        try
            [~, ~, refPos, refVel, refAcc, t_period] = generate_excitation_trajectory(config, ...
                'optimize', false, 'validate', false);
            ref_q = refPos;
            ref_qd = refVel;
            ref_qdd = refAcc;
            ref_t = t_period;
            fprintf('已生成单周期激励轨迹，点数 %d\n', size(ref_q, 1));
        catch me
            warning('激励轨迹生成失败 (%s)，改用随机点。', me.message);
        end
    end
    if ~isempty(ref_q)
        ref_q = ref_q(:, 1:n);
        ref_qd = ref_qd(:, 1:n);
        ref_qdd = ref_qdd(:, 1:n);
        q_all = ref_q;
        qd_all = ref_qd;
        qdd_all = ref_qdd;
        nTest = size(q_all, 1);
    end
end

if isempty(q_all)
    % 随机点（默认或 excitation 失败时）
    rng(42);
    nTest = 200;
    q_rad = 0.6;
    qd_rad = 0.5;
    qdd_rad = 0.8;
    q_all = (rand(nTest, n) - 0.5) * 2 * q_rad;
    qd_all = (rand(nTest, n) - 0.5) * 2 * qd_rad;
    qdd_all = (rand(nTest, n) - 0.5) * 2 * qdd_rad;
    q_all(1, :) = 0;
    qd_all(1, :) = 0;
    qdd_all(1, :) = 0;
end

err_norms = zeros(nTest, 1);
tau_ref_all = zeros(nTest, n);
tau_ref_full_all = zeros(nTest, n);
tau_Y_all = zeros(nTest, n);
tau_ref_stack = zeros(nTest * n, 1);
YY = [];

for k = 1:nTest
    q = q_all(k, :);
    qd = qd_all(k, :);
    qdd = qdd_all(k, :);

    % 整机参考力矩：full robot 逆动力学，只取腿的 6 个关节
    q_full   = zeros(1, n_full);
    qd_full  = zeros(1, n_full);
    qdd_full = zeros(1, n_full);
    q_full(idx_limb)   = q;
    qd_full(idx_limb)  = qd;
    qdd_full(idx_limb) = qdd;
    tau_ref_full = inverseDynamics(robot_full, q_full, qd_full, qdd_full);
    tau_ref_full = tau_ref_full(idx_limb);

    % 参考力矩：limb subtree 逆动力学
    tau_ref = inverseDynamics(robot_limb, q, qd, qdd);
    tau_ref = tau_ref(:);   % 6×1

    % 观测矩阵单点 + Y*θ
    Y_one = ReMatrix_E1_limb_URDF(limb, q, qd, qdd, 1, para_order);
    tau_Y = Y_one * theta;
    YY = [YY; Y_one];

    tau_ref_all(k, :)      = tau_ref';
    tau_ref_full_all(k, :) = tau_ref_full';
    tau_Y_all(k, :)        = tau_Y';
    err_norms(k) = max(abs(tau_ref - tau_Y));
    tau_ref_stack((k-1)*n + (1:n)) = tau_ref;
end

%% ===== 全参校验 =====
max_err = max(err_norms);
err_zero = err_norms(1);
fprintf('===== ReMatrix_E1_limb_URDF 验收（全参） =====\n');
traj_label = '随机';
if strcmp(trajectory_source, 'csv'), traj_label = 'CSV'; elseif strcmp(trajectory_source, 'excitation'), traj_label = '激励'; end
fprintf('肢体: %s, 轨迹: %s, 点数: %d, para_order: %d\n', limb, traj_label, nTest, para_order);
fprintf('max |τ_ref - Y*θ|_∞ = %.6e N·m\n', max_err);
fprintf('  (q=qd=qdd=0 时误差 = %.6e)\n', err_zero);
if max_err <= tol_norm
    fprintf('通过 (tol = %.2e N·m)\n', tol_norm);
else
    fprintf('未通过 (tol = %.2e N·m)\n', tol_norm);
    [~, idx] = max(err_norms);
    fprintf('最差点: k=%d, err_∞=%.6e\n', idx, max_err);
    fprintf('  q     = %s\n', mat2str(q_all(idx,:), 4));
    fprintf('  τ_ref = %s\n', mat2str(tau_ref_all(idx,:), 4));
    fprintf('  τ_Y   = %s\n', mat2str(tau_Y_all(idx,:), 4));
    fprintf('可能原因: 链顺序(Bodies≠base→tip)、θ 与 Y 列序/惯量约定不一致、vd0 符号。\n');
end

% 若是 CSV 轨迹，额外对最差点做一次“局部逐关节”对比，便于排查 ReMatrix vs inverseDynamics
if strcmp(trajectory_source, 'csv')
    % 若上面没有触发失败分支，则以误差最大的点作为 idx（包含通过时）
    if exist('idx', 'var')
        k_bad = idx;
    else
        [~, k_bad] = max(err_norms);
    end
    q_bad   = q_all(k_bad, :);
    qd_bad  = qd_all(k_bad, :);
    qdd_bad = qdd_all(k_bad, :);
    tau_ref_bad = tau_ref_all(k_bad, :).';
    tau_Y_bad   = tau_Y_all(k_bad, :).';

    fprintf('\n--- CSV 最差点逐关节对比 (k=%d) ---\n', k_bad);
    fprintf('  q_bad   = %s\n', mat2str(q_bad, 6));
    fprintf('  qd_bad  = %s\n', mat2str(qd_bad, 6));
    fprintf('  qdd_bad = %s\n', mat2str(qdd_bad, 6));
    fprintf('  关节 |   tau_ref   |    tau_Y   |  差值(tau_ref - tau_Y)\n');
    for j = 1:n
        fprintf('   %d   | %9.4f | %9.4f | %9.4f\n', j, tau_ref_bad(j), tau_Y_bad(j), tau_ref_bad(j) - tau_Y_bad(j));
    end
    fprintf('  |τ_ref - τ_Y|_∞(k_bad) = %.6e N·m\n', max(abs(tau_ref_bad - tau_Y_bad)));
end

%% ===== 最小参数集校验（与 aubo 参考代码一致：H = pinv(Y_min)*Y，β = H*θ） =====
% 列归一化 + SVD 定秩 + QR 列主元选列（不用 rref，避免病态误判）。
% H = pinv(Y_min)*Y 为全参→最小参数映射，β = H*θ，满足 Y_min*β ≈ Y*θ。
col_norm = sqrt(sum(YY.^2, 1));
col_norm(col_norm < 1e-12) = 1;
W = YY ./ (ones(size(YY,1), 1) * col_norm);

r = rank(W);                      % SVD 定秩
[~, ~, piv] = qr(W, 'vector');    % QR 列主元，W(:,piv)=Q*R
index_base = sort(piv(1:r));

YY_base_min = YY(:, index_base);
p_min = numel(index_base);

% 辨识矩阵条件数（最小参数 Y_min、全参 Y）
cond_Y_min = cond(YY_base_min);
cond_Y_full = cond(YY);

% 中间矩阵 H（= S）：全参 θ 映射到最小参数 β，Y_min*β ≈ Y*θ
H = pinv(YY_base_min) * YY;        % p×60，即参考代码里的 H
p_full2base = H * theta;          % β = H*θ，p×1

% Cover 验证：Y*θ 与 Y_min*(H*θ) 应一致（数值误差量级）
tau_full = YY * theta;
tau_base = YY_base_min * p_full2base;
cover_inf = norm(tau_full - tau_base, inf);
cover_fro_rel = norm(tau_full - tau_base, 'fro') / max(norm(tau_full, 'fro'), 1e-12);

% 方式1 等价于 β = H*θ（X_urdf 应与 p_full2base 一致）
X_urdf = YY_base_min \ (YY * theta);
% 堆叠顺序为 [点1的6维; 点2的6维; ...]，故 reshape(·, n, nTest)' 得 nTest×n
tau_min_from_theta = reshape(YY_base_min * X_urdf, n, nTest)';
err_theta_norms = max(abs(tau_min_from_theta - tau_Y_all), [], 2);
max_err_theta = max(err_theta_norms);

% 方式2：用 τ_ref 最小二乘辨识（实测辨识等价）
X_hat = YY_base_min \ tau_ref_stack;
tau_min_pred = reshape(YY_base_min * X_hat, n, nTest)';
err_min_norms = max(abs(tau_min_pred - tau_ref_all), [], 2);
max_err_min = max(err_min_norms);

% CSV 轨迹时：用实际力矩 τ_meas 辨识最小参数 X_hat_meas，得到 τ_pred_meas = Y_min*X_hat_meas
if strcmp(trajectory_source, 'csv') && exist('tau_meas_all', 'var') && isequal(size(tau_meas_all), [nTest, n])
    tau_meas_stack = reshape(tau_meas_all', n, nTest);
    tau_meas_stack = tau_meas_stack(:);
    X_hat_meas = YY_base_min \ tau_meas_stack;
    tau_pred_meas = reshape(YY_base_min * X_hat_meas, n, nTest)';
else
    tau_pred_meas = [];
end

fprintf('\n===== ReMatrix_E1_limb_URDF 验收（最小参数集，rank+QR 选列） =====\n');
fprintf('最小参数维度 p_min = %d / %d（index_base 前 12 个: %s ...）\n', p_min, 10*n, mat2str(index_base(1:min(12,end))));
fprintf('辨识矩阵条件数: cond(Y_min) = %.3e, cond(Y_full) = %.3e\n', cond_Y_min, cond_Y_full);
fprintf('--- H = pinv(Y_min)*Y，β = H*θ，cover 验证 ---\n');
fprintf('  ||Y*θ - Y_min*β||_∞       = %.3e N·m\n', cover_inf);
fprintf('  ||Y*θ - Y_min*β||_F/||Y*θ||_F = %.3e\n', cover_fro_rel);
fprintf('--- 方式1: Y*θ 在 col(Y_min) 上投影 ---\n');
fprintf('  max |Y*θ - Y_min*X_urdf|_∞ = %.6e N·m\n', max_err_theta);
fprintf('--- 方式2: Y_min \\ τ_ref（实测辨识等价） ---\n');
fprintf('  max |τ_ref - Y_min*X_hat|_∞ = %.6e N·m\n', max_err_min);
if max_err_theta <= tol_norm && max_err_min <= tol_norm
    fprintf('通过 (tol = %.2e N·m)\n', tol_norm);
else
    if max_err_theta > tol_norm
        fprintf('方式1 未通过：列空间或秩判定需检查。\n');
    end
    if max_err_min > tol_norm
        fprintf('方式2 未通过 (与全参误差同量级时可接受)。\n');
        [~, idx] = max(err_min_norms);
        fprintf('最差点: k=%d，τ_ref=%s，pred=%s\n', idx, ...
            mat2str(tau_ref_all(idx,:),4), mat2str(tau_min_pred(idx,:),4));
    end
end
fprintf('========================================\n');

%% ===== 最小参数显式公式：p_m = S*θ（S = H = pinv(Y_min)*Y） =====
% p_m(i) = sum_j S(i,j)*θ(j)；系数由本机构 regressor 唯一确定。
opts_export = struct('coeff_threshold', 1e-10, 'do_latex', false, 'n_links', n);
[S_base, lines_text, lines_latex] = export_base_parameter_formulas(YY, index_base, para_order, opts_export);

formula_file = fullfile(app_root, 'docs', 'base_parameter_formulas.txt');
fid = fopen(formula_file, 'w');
fprintf(fid, '%% 最小参数显式线性组合（E1 limb, para_order=%d, p_min=%d）\n', para_order, p_min);
fprintf(fid, '%% p_m = S*θ，S = H = pinv(Y_min)*Y；θ 顺序: 每连杆 [m mx my mz Ixx Ixy Ixz Iyy Iyz Izz]\n');
fprintf(fid, '%% cond(Y_min)=%.3e, cond(Y_full)=%.3e\n', cond_Y_min, cond_Y_full);
fprintf(fid, '%% 生成自 test_ReMatrix_E1_limb_URDF / export_base_parameter_formulas\n\n');
for i = 1:numel(lines_text)
    fprintf(fid, '%s\n', lines_text{i});
end
fclose(fid);

fprintf('\n===== 最小参数显式公式 p_m = S*θ（S = H） =====\n');
fprintf('（系数由 Y 与 index_base 自动生成，与当前 ReMatrix/URDF 一致）\n');
n_show = min(5, p_min);
for i = 1:n_show
    fprintf('  %s\n', lines_text{i});
end
if p_min > n_show
    fprintf('  ... 共 %d 条，完整公式已写入: %s\n', p_min, formula_file);
else
    fprintf('完整公式已写入: %s\n', formula_file);
end
fprintf('========================================\n');

%% 正动力学验证：与辨识轨迹一致的三种 FD 对比
% 1) URDF 正动力学：qdd_urdf = forwardDynamics(robot_limb, q, qd, τ_ref^{limb})
% 2) 全参 FD：qdd_fd_full = forward_dynamics_full(q, qd, τ_ref^{limb}, θ 或 π_rec)
% 3) 最小参数 FD：qdd_fd_min = forward_dynamics_min(q, qd, τ_ref^{limb}, X_hat, index_base)

fprintf('\n===== 正动力学对比：与验收同一批轨迹 =====\n');

% 轨迹采用上面构造 YY 时的 (q_all, qd_all, qdd_all, tau_ref_all)，共 nTest 点
qdd_fd_urdf = zeros(nTest, n);
qdd_fd_full = zeros(nTest, n);
qdd_fd_min  = zeros(nTest, n);

% 若已从 β 恢复 θ，则 theta 已是 π_rec；否则 theta = π_cad
pi_used = theta(:);

for k = 1:nTest
    qk       = q_all(k, :);          % 1×n
    qdk      = qd_all(k, :);         % 1×n
    tauk_row = tau_ref_all(k, :);    % 1×n
    tauk_col = tauk_row.';           % n×1

    % 1) URDF 正动力学（以 limb subtree 为模型，要求 tau 为 1×n 行向量）
    qdd_fd_urdf(k, :) = forwardDynamics(robot_limb, qk, qdk, tauk_row).';

    % 2) 全参 FD（ReMatrix + π，用 forward_dynamics_full，同样用行向量 tau）
    qdd_fd_full(k, :) = forward_dynamics_full(qk, qdk, tauk_row, pi_used, limb, para_order).';

    % 3) 最小参数 FD（forward_dynamics_min 期望列向量 tau）
    qdd_fd_min(k, :)  = forward_dynamics_min(qk.', qdk.', tauk_col, ...
                                             X_hat, index_base, limb, para_order).';
end

err_qdd_urdf = qdd_fd_urdf - qdd_all;
err_qdd_full = qdd_fd_full - qdd_all;
err_qdd_min  = qdd_fd_min  - qdd_all;

rmse_qdd_urdf   = sqrt(mean(err_qdd_urdf.^2, 1));
rmse_qdd_full   = sqrt(mean(err_qdd_full.^2, 1));
rmse_qdd_min    = sqrt(mean(err_qdd_min.^2, 1));
maxabs_qdd_urdf = max(abs(err_qdd_urdf), [], 1);
maxabs_qdd_full = max(abs(err_qdd_full), [], 1);
maxabs_qdd_min  = max(abs(err_qdd_min),  [], 1);

fprintf('  URDF  FD   各关节 RMSE(qdd)   (rad/s^2):'); fprintf(' %.4f', rmse_qdd_urdf); fprintf('\n');
fprintf('  full  FD   各关节 RMSE(qdd)   (rad/s^2):'); fprintf(' %.4f', rmse_qdd_full); fprintf('\n');
fprintf('  min   FD   各关节 RMSE(qdd)   (rad/s^2):'); fprintf(' %.4f', rmse_qdd_min);  fprintf('\n');
fprintf('  URDF  FD   各关节 max|err_qdd| (rad/s^2):'); fprintf(' %.4f', maxabs_qdd_urdf); fprintf('\n');
fprintf('  full  FD   各关节 max|err_qdd| (rad/s^2):'); fprintf(' %.4f', maxabs_qdd_full); fprintf('\n');
fprintf('  min   FD   各关节 max|err_qdd| (rad/s^2):'); fprintf(' %.4f', maxabs_qdd_min);  fprintf('\n');
fprintf('========================================\n');

%% 绘图：共三张图（力矩 + 力矩误差 + 加速度）
x_pts = (1 : nTest)';

% 图1：力矩对比（limb τ_ref / full τ_ref / Y*θ 全参 / Y_min*X_hat 最小参数集，四曲线）
figure('Name', '力矩对比');
plot_compare_6dof(x_pts, [tau_ref_all, tau_ref_full_all, tau_Y_all, tau_min_pred], 'torque', ...
    {'\tau_{ref}^{limb}', '\tau_{ref}^{full} (整机URDF)', '\tau_{Y\theta} (全参)', '\tau_{Y\_min} (最小参数集)'});
sgtitle('验收：\tau_{ref}^{limb} / \tau_{ref}^{full} vs Y\theta（全参）vs Y\_min*X\_hat（最小参数集）');

% 图2：误差对比（limb vs Yθ、full vs Yθ、limb vs Y_min*X_hat）
figure('Name', '力矩误差对比');
err_full_vs_Y = tau_ref_full_all - tau_Y_all;
plot_compare_6dof(x_pts, [tau_ref_all - tau_Y_all, err_full_vs_Y, tau_ref_all - tau_min_pred], 'torque', ...
    {'\tau_{ref}^{limb} - Y\theta', '\tau_{ref}^{full} - Y\theta', '\tau_{ref}^{limb} - Y\_min*X\_hat'});
sgtitle('验收：力矩误差对比（limb/full vs 全参 vs 最小参数集）');

% 图2：加速度对比（qdd_all vs 三种 FD）
figure('Name', '正动力学_加速度对比');
plot_compare_6dof(x_pts, [qdd_all, qdd_fd_urdf, qdd_fd_full, qdd_fd_min], 'qdd', ...
    {'qdd\_{meas/YY轨迹}', 'qdd\_{FD,URDF}', 'qdd\_{FD,full}', 'qdd\_{FD,min}'});
sgtitle('验收：qdd 对比（URDF FD / full FD(\pi) / min FD(\beta)）');

% 图3（仅 CSV 轨迹）：实际力矩 vs 模型拟合预测（X_hat 由 τ_ref 拟合）
if strcmp(trajectory_source, 'csv') && exist('tau_meas_all', 'var') && isequal(size(tau_meas_all), [nTest, n])
    figure('Name', 'CSV_测量vs预测_模型拟合');
    plot_compare_6dof(x_pts, [tau_meas_all, tau_min_pred], 'torque', ...
        {'\tau_{meas} (CSV)', 'Y\_min*X\_hat (X\_hat 由 \tau_{ref} 拟合)'});
    sgtitle('验收(CSV)：测量 vs 预测（X\_hat 由 \tau_{ref} 拟合）');
    err_meas_pred = max(abs(tau_meas_all - tau_min_pred), [], 2);
    fprintf('\n===== CSV 轨迹：测量 vs 预测（X_hat 由 τ_ref 拟合） =====\n');
    fprintf('max |τ_meas - Y_min*X_hat|_∞ = %.4f N·m\n', max(err_meas_pred));
    fprintf('若该误差大而 τ_ref - Y_min*X_hat 小，说明 τ_ref（逆动力学）与 τ_meas（实测）在该轨迹上不一致。\n');
    fprintf('可能原因：辨识时 qdd 未平滑（请用 example_run_min_param_id）、或模型与实机差异（摩擦/刚度等）。\n');
    fprintf('========================================\n');
end

% 图4（仅 CSV 轨迹）：实际力矩 + 实际力矩辨识的最小参数集计算的关节力矩（与 τ_ref 对照）
if strcmp(trajectory_source, 'csv') && exist('tau_meas_all', 'var') && isequal(size(tau_meas_all), [nTest, n]) && ~isempty(tau_pred_meas)
    figure('Name', 'CSV_实际力矩与辨识预测');
    plot_compare_6dof(x_pts, [tau_ref_all, tau_meas_all, tau_pred_meas], 'torque', ...
        {'\tau_{ref} (逆动力学)', '\tau_{meas} (实际)', 'Y\_min*X\_hat_{meas} (实际辨识)'});
    sgtitle('验收(CSV)：实际力矩 vs 实际辨识最小参数集计算的关节力矩');
    err_meas_id = max(abs(tau_meas_all - tau_pred_meas), [], 2);
    fprintf('\n===== CSV 轨迹：实际力矩 vs 实际辨识最小参数预测 =====\n');
    fprintf('max |τ_meas - Y_min*X_hat_meas|_∞ = %.4f N·m\n', max(err_meas_id));
    fprintf('（X_hat_meas = Y_min \\ τ_meas，与本脚本同轨迹下 run_min_param_id 等价）\n');
    fprintf('========================================\n');
end

% 图5（仅 CSV 轨迹）：测量力矩 vs 各模型的误差对比
if strcmp(trajectory_source, 'csv') && exist('tau_meas_all', 'var') && isequal(size(tau_meas_all), [nTest, n])
    figure('Name', 'CSV_误差对比_模型vs测量');
    err_meas_vs_ref  = tau_meas_all - tau_ref_all;      % 测量 - limb 逆动力学
    err_meas_vs_Y    = tau_meas_all - tau_Y_all;        % 测量 - 全参 Yθ
    err_meas_vs_Ymin = tau_meas_all - tau_min_pred;     % 测量 - 最小参数预测(X_hat 由 τ_ref)
    plot_compare_6dof(x_pts, [err_meas_vs_ref, err_meas_vs_Y, err_meas_vs_Ymin], 'torque', ...
        {'\tau_{meas} - \tau_{ref}^{limb}', '\tau_{meas} - Y\theta', '\tau_{meas} - Y\_min*X\_hat'});
    sgtitle('CSV：测量 vs 模型误差对比');
end

%% 可选：根据 debug_level 运行额外的 ReMatrix 调试脚本
if debug_level >= 1
    fprintf('\n[debug_level = %d] 运行 verify_error_source（零位重力 / 重力符号 / 链序+轴检查）。\n', debug_level);
    try
        verify_error_source;
    catch ME
        warning('调用 verify_error_source 失败: %s', ME.message);
    end
end

if debug_level >= 2
    fprintf('[debug_level = %d] 运行 test_ReMatrix_vs_inverseDynamics_local（局部 ReMatrix vs inverseDynamics 对比）。\n', debug_level);
    try
        test_ReMatrix_vs_inverseDynamics_local;
    catch ME
        warning('调用 test_ReMatrix_vs_inverseDynamics_local 失败: %s', ME.message);
    end
end
