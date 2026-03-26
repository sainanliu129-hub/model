%% run_full_dynamics_validation  解耦版全流程验证
% 流程：读取 -> 预处理 -> 连续窗 -> 最小参数辨识 -> full参数恢复/辨识 -> ID/FD 多模型对比
% 每步结果均缓存，任意步骤结束后重启无需重复计算。

clc; clear; close all;

app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
if exist('ensure_body_gravity_para_iden_path', 'file')
    ensure_body_gravity_para_iden_path();
end

%% 1) 配置
csv_file = fullfile(app_root, 'data', 'excitation', 'PD-M1-v0_multi_joint_20260325-183300.csv');
if ~isfile(csv_file)
    error('CSV 不存在: %s', csv_file);
end

cfg = struct();
cfg.use_preprocess = true;
cfg.do_append_result_summary = true;   % 每次跑完生成/追加 scheme_result_summary
% 预处理：tau 低通 25Hz；q 对 J4/J6 单独降低 cutoff
% 全动力学验证：不读 Friction_Iden 汇总表、不做 J_eq/摩擦/偏置补偿（tau_id≈低通 tau_s）
% scheme_result_summary 中 use_friction_56 见下方：按「实际是否带摩擦参数」与 row 配置共同判定
cfg.prep_opts = struct( ...
    't_start_s', 2.0, ...
    't_end_s', 4.0, ...
    'q_lowpass_fc_Hz', [25 25 25 18 25 18], ...  % J4/J6：25 -> 18Hz（可改 15~20）
    'q_lowpass_order', 2, ...
    'tau_lowpass_fc_Hz', 25, ...
    'tau_lowpass_order', 2, ...
    'do_compensation', false, ...
    'load_friction_from_summary', false, ...
    'do_Jeq_compensation', false, ...  % 只减摩擦，不减电机等效惯量项
    'row_for_joint', [0 0 0 6 0 0], ...
    'do_plot', true);
cfg.window_opts = struct('t_start_s', [], 't_end_s', [], 'qd_lowpass_fc_Hz', 0, 'qdd_smooth_half', 0);
% Step3 降采样：在“滤波+截取时间窗”完成后，再对 dataset 降采样以减少求解规模
cfg.downsample_stride = 1;  % 约 500Hz -> 100Hz（取决于原始 dt）

% Step4 最小参数辨识调参（方案二推荐默认）
cfg.min_id_opts = struct( ...
    'use_wls', true, ...
    'lambda_ridge', 1e-6);

% Step5 物理恢复调参（面向 FD）
cfg.phys_opts = struct( ...
    'lambda_phys', 1, ...
    'lambda_qd', 1e-4, ...  % H 步串联速度一致 J_qd（与 idx_M 同稀疏采样）
    'qd_H_steps', 1, ...    % J_qd 串联步数 H
    'lambda_M', 1e-3, ...
    'algorithm', 'sqp', ...
    'max_iter', 1000, ...
    'MaxFunctionEvaluations', 12000, ...
    'display', 'iter', ...
    'm_min_frac', 0.8, ...
    'm_max_frac', 1.2, ...
    'delta_c', 0.03, ...
    'eps_pd', 1e-4, ...
    'eps_M', 1e-5, ...
    'h_box_abs', 0.12, ...
    'I_diag_lim_frac', [0.5, 1.6], ...
    'w_m', 1, ...
    'w_h', 5, ...
    'w_I', 8, ...
    'offdiag_max_frac', 0.3, ...
    'diag_dev_frac', 0.6, ...
    'offdiag_dev_abs', 0.05);

% 每一步是否强制重新计算（忽略缓存）
cfg.force_recompute_step2 = true;
cfg.force_recompute_step3 = true;
cfg.force_recompute_step4 = true;
cfg.force_recompute_step5 = true;
cfg.force_recompute_step6 = true;
cfg.force_recompute_step7 = true;

limb = 'left_leg';
para_order = 1;
run_pi_phys = true;
run_pi_fd = false;
M_fd_max = inf;
save_excel = true;
use_fast_fd_id_tuning = true; % true=快速调参档, false=标准档
plot_id_compare = true;
plot_fd_compare = true;
export_id_plot_png_to_excel = true;
% FD 速度 qd 的对比/RMSE/图：Inf=全程积分；正整数 H=H 步串联；验证统一按 1 步
cfg.fd_qd_compare_steps = 1;

% Step6/7：只计算/打印/对比列出的模型（省算力）。可选：'cad','min','rec','phys','fd'
% 例如只跑 MIN + PHYS：cfg.compare_models = {'min','phys'};
if ~isfield(cfg, 'compare_models') || isempty(cfg.compare_models)
    cfg.compare_models = {'cad','phys'};
    if run_pi_fd
        cfg.compare_models = [cfg.compare_models, {'fd'}];
    end
end
Lcmp = cfg.compare_models;
if ~iscell(Lcmp), Lcmp = {Lcmp}; end
Lcmp = cellfun(@(x) lower(strtrim(char(string(x)))), Lcmp, 'UniformOutput', false);
id_run_cad  = any(strcmp(Lcmp, 'cad'));
id_run_min  = any(strcmp(Lcmp, 'min'));
id_run_rec  = any(strcmp(Lcmp, 'rec'));
id_run_phys = any(strcmp(Lcmp, 'phys'));
id_run_fd_m = any(strcmp(Lcmp, 'fd')) && run_pi_fd;

fprintf('[Step6/7] compare_models = %s\n', strjoin(Lcmp, ', '));
fprintf('[run_full_dynamics_validation] 准备进入步骤5：CAD/full 参数辨识 (full_fd)\n');

% 每步保存到 build 下独立文件，保证任意步骤结束后不重复计算
build_dir = fullfile(app_root, 'build');
if ~isfolder(build_dir), mkdir(build_dir); end
step2_mat = fullfile(build_dir, 'step2_preprocessed.mat');
step3_mat = fullfile(build_dir, 'step3_dataset.mat');
step4_mat = fullfile(build_dir, 'step4_res_min.mat');
step5_mat = fullfile(build_dir, 'step5_full_params.mat');
step6_mat = fullfile(build_dir, 'step6_id_results.mat');
step7_mat = fullfile(build_dir, 'step7_fd_results.mat');

%% 2) 步骤2：预处理（含预处理前后对比图 + 保存最终版）
load_step2 = false;
if ~cfg.force_recompute_step2 && isfile(step2_mat)
    try
        S2 = load(step2_mat);
        if isfield(S2, 'csv_file') && isequal(S2.csv_file, csv_file)
            data_before = S2.data_before;
            data_after = S2.data_after;
            prep_used = S2.prep_used;
            load_step2 = true;
            fprintf('[缓存] 已加载步骤2，跳过计算。\n');
            if cfg.use_preprocess
                plot_preprocess_before_after_from_data(data_before, data_after, prep_used, limb, 1:6);
            end
        end
    catch ME
        fprintf('加载步骤2失败 (%s)，将重新计算。\n', ME.message);
    end
end

if ~load_step2
    % 调用 run_id_preprocess_pipeline 获取预处理前后数据
    [data_before, data_after, prep_used] = run_id_preprocess_pipeline(csv_file, cfg.prep_opts);

    % 步骤2 保存最终版
    save(step2_mat, 'data_before', 'data_after', 'prep_used', 'csv_file');
    fprintf('步骤2已保存到: %s\n', step2_mat);

    % 绘制预处理前后对比图（仅当启用预处理时）
    if cfg.use_preprocess
        plot_preprocess_before_after_from_data(data_before, data_after, prep_used, limb, 1:6);
        fprintf('已绘制预处理前后轨迹对比图。\n');
    end
end

%% 3) 步骤3：连续窗 -> dataset
load_step3 = false;
if ~cfg.force_recompute_step3 && isfile(step3_mat)
    try
        S3 = load(step3_mat);
        if isfield(S3, 'csv_file') && isequal(S3.csv_file, csv_file) ...
                && isfield(S3, 'meta') && isfield(S3.meta, 'downsample_stride') ...
                && isequal(S3.meta.downsample_stride, cfg.downsample_stride)
            dataset = S3.dataset;
            meta = S3.meta;
            load_step3 = true;
            fprintf('[缓存] 已加载步骤3，跳过计算。\n');
        end
    catch ME
        fprintf('加载步骤3失败 (%s)，将重新计算。\n', ME.message);
    end
end
if ~load_step3
    win = cfg.window_opts;
    if cfg.use_preprocess
        if ~isfield(win, 't_start_s') || isempty(win.t_start_s), win.t_start_s = []; end
        if ~isfield(win, 't_end_s') || isempty(win.t_end_s), win.t_end_s = []; end
        if ~isfield(win, 'qd_lowpass_fc_Hz') || isempty(win.qd_lowpass_fc_Hz), win.qd_lowpass_fc_Hz = 0; end
        if ~isfield(win, 'qdd_smooth_half') || isempty(win.qdd_smooth_half), win.qdd_smooth_half = 0; end
    else
        if ~isfield(win, 't_start_s') || isempty(win.t_start_s), win.t_start_s = 2.1; end
        if ~isfield(win, 't_end_s') || isempty(win.t_end_s), win.t_end_s = 4.1; end
        if ~isfield(win, 'qd_lowpass_fc_Hz') || isempty(win.qd_lowpass_fc_Hz), win.qd_lowpass_fc_Hz = 0; end
        if ~isfield(win, 'qdd_smooth_half') || isempty(win.qdd_smooth_half), win.qdd_smooth_half = 0; end
    end
    avg_data = continuous_window_id_data(data_after.t, data_after.q, data_after.qd, data_after.tau_id, win, data_after.qdd);
    dataset = struct();
    dataset.t = avg_data.t_equiv(:);
    dataset.q = avg_data.q_bar;
    dataset.qd = avg_data.qd_bar;
    dataset.qdd = avg_data.qdd_bar;
    dataset.tau = avg_data.tau_bar;
    dataset.avg_data = avg_data;
    meta = struct();
    meta.csv_file = csv_file;
    meta.cfg = cfg;
    meta.prep_used = prep_used;
    meta.prep_aux = struct();
    meta.window_used = win;
    meta.downsample_stride = cfg.downsample_stride;

    % 3.x 降采样：在时间窗整理完成后，再对 dataset 同步降采样
    ds = cfg.downsample_stride;
    if ~isempty(ds) && isnumeric(ds) && isfinite(ds) && ds > 1 && size(dataset.q,1) >= ds
        idx_ds = 1:ds:size(dataset.t,1);
        dataset.t = dataset.t(idx_ds);
        dataset.q = dataset.q(idx_ds, :);
        dataset.qd = dataset.qd(idx_ds, :);
        dataset.qdd = dataset.qdd(idx_ds, :);
        dataset.tau = dataset.tau(idx_ds, :);
        % 同步 avg_data，避免导出时不一致
        dataset.avg_data.t_equiv = dataset.t;
        dataset.avg_data.q_bar = dataset.q;
        dataset.avg_data.qd_bar = dataset.qd;
        dataset.avg_data.qdd_bar = dataset.qdd;
        dataset.avg_data.tau_bar = dataset.tau;
        dataset.avg_data.tau_std = dataset.avg_data.tau_std(idx_ds, :);
        dataset.avg_data.phi = dataset.avg_data.phi(idx_ds, :);
        dataset.avg_data.per_cycle = dataset.avg_data.per_cycle; % 维度不严格保证，主要用于绘图
    end
    % 步骤3 保存最终版
    save(step3_mat, 'dataset', 'meta', 'csv_file');
    fprintf('步骤3已保存到: %s\n', step3_mat);
end

fprintf('数据集: M=%d, t=[%.3f, %.3f], downsample_stride=%d\n', ...
    size(dataset.q,1), dataset.t(1), dataset.t(end), cfg.downsample_stride);

%% 4) 步骤4：最小参数辨识
id_opts = struct();
id_opts.limb = limb;
id_opts.para_order = para_order;
id_opts.identify_min_opts = struct( ...
    'use_wls', cfg.min_id_opts.use_wls, ...
    'lambda', cfg.min_id_opts.lambda_ridge, ...
    'para_order', para_order);

load_step4 = false;
if ~cfg.force_recompute_step4 && isfile(step4_mat)
    try
        S4 = load(step4_mat);
        if isfield(S4, 'csv_file') && isequal(S4.csv_file, csv_file)
            res_min = S4.res_min;
            X_hat = res_min.X_hat;
            index_base = res_min.index_base;
            load_step4 = true;
            fprintf('[缓存] 已加载步骤4，跳过计算。\n');
        end
    catch ME
        fprintf('加载步骤4失败 (%s)，将重新计算。\n', ME.message);
    end
end
if ~load_step4
    res_min = identify_params_dispatch(dataset, 'min_ls', id_opts);
    X_hat = res_min.X_hat;
    index_base = res_min.index_base;
    save(step4_mat, 'res_min', 'X_hat', 'index_base', 'csv_file');
    fprintf('步骤4已保存到: %s\n', step4_mat);
end
step4_changed = ~load_step4;
fprintf('最小参数维度 p_min=%d\n', numel(X_hat));
fprintf('Step4设置: use_wls=%d, lambda_ridge=%.3e\n', ...
    cfg.min_id_opts.use_wls, cfg.min_id_opts.lambda_ridge);

%% 5) 步骤5：CAD/full 参数
load_step5 = false;
if ~cfg.force_recompute_step5 && ~step4_changed && isfile(step5_mat)
    try
        S5 = load(step5_mat);
        if isfield(S5, 'csv_file') && isequal(S5.csv_file, csv_file)
            pi_cad = S5.pi_cad;
            pi_rec = S5.pi_rec;
            pi_phys = S5.pi_phys;
            pi_fd = S5.pi_fd;
            load_step5 = true;
            fprintf('[缓存] 已加载步骤5，跳过计算。\n');
        end
    catch ME
        fprintf('加载步骤5失败 (%s)，将重新计算。\n', ME.message);
    end
end
if ~load_step5
    [robot_limb, n] = get_e1_limb_robot(limb); %#ok<ASGLU>
    pi_cad = get_limb_theta_from_URDF(robot_limb, para_order);
    Y_full = ReMatrix_E1_limb_URDF(limb, dataset.q, dataset.qd, dataset.qdd, 1, para_order);
    Y_min = Y_full(:, index_base);
    K = pinv(Y_min) * Y_full;
    beta_hat = X_hat(:);
    pi_rec = [];
    res_norm_rec = nan;
    if id_run_rec
        [pi_rec, res_norm_rec] = recover_full_params_from_beta(K, beta_hat, pi_cad, 1e-2);
        fprintf('pi_rec 恢复残差=%.3e\n', res_norm_rec);
    end

    pi_phys = [];
    if run_pi_phys && id_run_phys
        idx_phys_sparse = round(linspace(1, size(dataset.q, 1), min(12, size(dataset.q, 1))))';
        opts_phys = struct( ...
            'algorithm', cfg.phys_opts.algorithm, ...
            'max_iter', cfg.phys_opts.max_iter, ...
            'MaxFunctionEvaluations', cfg.phys_opts.MaxFunctionEvaluations, ...
            'display', cfg.phys_opts.display, ...
            'm_min_frac', cfg.phys_opts.m_min_frac, ...
            'm_max_frac', cfg.phys_opts.m_max_frac, ...
            'delta_c', cfg.phys_opts.delta_c, ...
            'eps_pd', cfg.phys_opts.eps_pd, ...
            'eps_M', cfg.phys_opts.eps_M, ...
            'h_box_abs', cfg.phys_opts.h_box_abs, ...
            'I_diag_lim_frac', cfg.phys_opts.I_diag_lim_frac, ...
            'w_m', cfg.phys_opts.w_m, ...
            'w_h', cfg.phys_opts.w_h, ...
            'w_I', cfg.phys_opts.w_I, ...
            'offdiag_max_frac', cfg.phys_opts.offdiag_max_frac, ...
            'diag_dev_frac', cfg.phys_opts.diag_dev_frac, ...
            'offdiag_dev_abs', cfg.phys_opts.offdiag_dev_abs, ...
            'lambda_M', cfg.phys_opts.lambda_M, ...
            'lambda_qd', cfg.phys_opts.lambda_qd, ...
            'qd_H_steps', cfg.phys_opts.qd_H_steps, ...
            't_sample', dataset.t(:), ...
            'q_sample', dataset.q, ...
            'qd_sample', dataset.qd, ...
            'tau_sample', dataset.tau, ...
            'q_M', dataset.q, ...
            'idx_M', idx_phys_sparse, ...
            'limb', limb, ...
            'para_order', para_order);
        [pi_phys, info_phys] = solve_full_params_physical( ...
            Y_full, Y_min, beta_hat, pi_cad, cfg.phys_opts.lambda_phys, opts_phys); %#ok<NASGU>
        if isfield(info_phys, 'J_qd')
            fprintf('[PHYS] 诊断 J_qd(H步串联qd一致, 关节加权)=%.6e, lambda_qd*J_qd=%.6e\n', ...
                info_phys.J_qd, cfg.phys_opts.lambda_qd * info_phys.J_qd);
            if isfield(info_phys, 'qd_err_joint_weights_used') && ~isempty(info_phys.qd_err_joint_weights_used)
                fprintf('[PHYS] J_qd 关节权重(已mean归一): %s\n', mat2str(info_phys.qd_err_joint_weights_used, 4));
            end
        end
    end

    pi_fd = [];
    if run_pi_fd && id_run_fd_m
        if use_fast_fd_id_tuning
            idx_qdd_sparse = (1:100:size(dataset.q,1)).';
            w_qdd_cfg = 5e-5;
            max_iter_cfg = 40;
            max_fun_eval_cfg = 12000;
            fprintf('[FD ID] 使用快速调参档: idx_qdd=1:100:N, w_qdd=5e-5, max_iter=40\n');
        else
            idx_qdd_sparse = (1:20:size(dataset.q,1)).';
            w_qdd_cfg = 1e-4;
            max_iter_cfg = 100;
            max_fun_eval_cfg = 70000;
            fprintf('[FD ID] 使用标准档: idx_qdd=1:20:N, w_qdd=1e-4, max_iter=100\n');
        end
        opts_full = struct();
        opts_full.limb = limb;
        opts_full.para_order = para_order;
        opts_full.pi_cad = pi_cad;
%         opts_full.identify_full_fd_opts = struct('w_tau',4,'w_qdd',4,'w_cad',1,'w_M',10, ...
%             'w_traj',5, 'alpha_q',1.0, 'alpha_qd',2.0, ...
%             'max_iter',50,'MaxFunctionEvaluations',1000,'display','iter', ...
%             'StepTolerance', 1e-2, 'ConstraintTolerance', 1e-2, 'OptimalityTolerance', 1e-2);
        opts_full.identify_full_fd_opts = struct( ...
            'w_tau', 4, ...
            'w_qdd', w_qdd_cfg, ...
            'w_cad', 1.5, ...
            'w_M', 0, ...
            'idx_qdd', idx_qdd_sparse, ...
            'qdd_fail_penalty', 1e6, ...
            'tau_loss_mode', 'global_mse', ...
            'tau_joint_weight_basis', 'rms', ...
            'tau_weight_eps', 1e-6, ...
            'h_lim_abs', 0.6, ...
            'I_lim_frac', [0.3, 6.0], ...
            'w_traj', 0, ...
            'traj_H', 5, ...  % FD 辨识轨迹窗长设为 5；当前 w_traj=0 时该项不进入目标
            'alpha_q', 1.0, ...
            'alpha_qd', 2.0, ...
            'max_iter', max_iter_cfg, ...
            'MaxFunctionEvaluations', max_fun_eval_cfg, ...
            'display', 'iter', ...
            'StepTolerance', 1e-6, ...
            'ConstraintTolerance', 1e-4, ...
            'OptimalityTolerance', 1e-4, ...
            'use_com_constraint', false);
        res_full = identify_params_dispatch(dataset, 'full_fd', opts_full);
        pi_fd = res_full.pi_fd;
        info_fd = res_full.info;
    end

    % 步骤5 保存最终版
    if exist('info_fd','var')
        save(step5_mat, 'pi_cad', 'pi_rec', 'pi_phys', 'pi_fd', 'csv_file', 'info_fd');
    else
        save(step5_mat, 'pi_cad', 'pi_rec', 'pi_phys', 'pi_fd', 'csv_file');
    end
    fprintf('步骤5已保存到: %s\n', step5_mat);

    if exist('info_fd','var') && isfield(info_fd,'J_tau')
        fprintf('\n[FD ID 解的目标分解(回归矩阵空间)] J_tau=%.6e, J_cad=%.6e\n', info_fd.J_tau, info_fd.J_cad);
    end
end
step5_changed = ~load_step5;
[~, n] = get_e1_limb_robot(limb);

%% 6) 步骤6：逆动力学对比
M = size(dataset.q, 1);
load_step6 = false;
if ~cfg.force_recompute_step6 && isfile(step6_mat)
    try
        S6 = load(step6_mat);
        if isfield(S6, 'csv_file') && isequal(S6.csv_file, csv_file) && isfield(S6, 'tau_meas') && size(S6.tau_meas,1)==M
            tau_meas = S6.tau_meas;
            tau_cad = S6.tau_cad;
            tau_min = S6.tau_min;
            tau_rec = S6.tau_rec;
            tau_phys = S6.tau_phys;
            tau_fd = S6.tau_fd;
            load_step6 = true;
            fprintf('[缓存] 已加载步骤6，跳过计算。\n');
        end
    catch ME
        fprintf('加载步骤6失败 (%s)，将重新计算。\n', ME.message);
    end
end
tau_meas = dataset.tau;
if ~load_step6
    tau_cad = zeros(M, n);
    tau_min = zeros(M, n);
    tau_rec = zeros(M, n);
    tau_phys = zeros(M, n);
    tau_fd = zeros(M, n);
end
if ~exist('tau_cad','var') || isempty(tau_cad), tau_cad = zeros(M, n); end
if ~exist('tau_min','var') || isempty(tau_min), tau_min = zeros(M, n); end
if ~exist('tau_rec','var') || isempty(tau_rec), tau_rec = zeros(M, n); end
if ~exist('tau_phys','var') || isempty(tau_phys), tau_phys = zeros(M, n); end
if ~exist('tau_fd','var') || isempty(tau_fd), tau_fd = zeros(M, n); end

recalc_id_cad  = (cfg.force_recompute_step6 || ~load_step6) && id_run_cad;
recalc_id_min  = (cfg.force_recompute_step6 || ~load_step6 || step4_changed) && id_run_min;
recalc_id_rec  = (cfg.force_recompute_step6 || ~load_step6 || step5_changed) && id_run_rec && ~isempty(pi_rec);
recalc_id_phys = (cfg.force_recompute_step6 || ~load_step6 || step5_changed) && id_run_phys && ~isempty(pi_phys);
recalc_id_fd   = (cfg.force_recompute_step6 || ~load_step6 || step5_changed) && id_run_fd_m && ~isempty(pi_fd);

if recalc_id_cad || recalc_id_min || recalc_id_rec || recalc_id_phys || recalc_id_fd
    model_cad_id = struct('type','full','limb',limb,'para_order',para_order,'pi_vec',pi_cad);
    model_min_id = struct('type','min','limb',limb,'para_order',para_order,'X_hat',X_hat,'index_base',index_base);
    model_rec_id = struct('type','full','limb',limb,'para_order',para_order,'pi_vec',pi_rec);
    if ~isempty(pi_phys), model_phys_id = struct('type','full','limb',limb,'para_order',para_order,'pi_vec',pi_phys); end
    if ~isempty(pi_fd), model_fd_id = struct('type','full','limb',limb,'para_order',para_order,'pi_vec',pi_fd); end

    fprintf('步骤6增量重算: CAD=%d, MIN=%d, REC=%d, PHYS=%d, FD=%d\n', ...
        recalc_id_cad, recalc_id_min, recalc_id_rec, recalc_id_phys, recalc_id_fd);
    for k = 1:M
        if recalc_id_cad, tau_cad(k,:) = inverse_dynamics_dispatch(dataset.q(k,:), dataset.qd(k,:), dataset.qdd(k,:), model_cad_id).'; end
        if recalc_id_min, tau_min(k,:) = inverse_dynamics_dispatch(dataset.q(k,:), dataset.qd(k,:), dataset.qdd(k,:), model_min_id).'; end
        if recalc_id_rec, tau_rec(k,:) = inverse_dynamics_dispatch(dataset.q(k,:), dataset.qd(k,:), dataset.qdd(k,:), model_rec_id).'; end
        if recalc_id_phys, tau_phys(k,:) = inverse_dynamics_dispatch(dataset.q(k,:), dataset.qd(k,:), dataset.qdd(k,:), model_phys_id).'; end
        if recalc_id_fd, tau_fd(k,:) = inverse_dynamics_dispatch(dataset.q(k,:), dataset.qd(k,:), dataset.qdd(k,:), model_fd_id).'; end
    end
    save(step6_mat, 'tau_meas', 'tau_cad', 'tau_min', 'tau_rec', 'tau_phys', 'tau_fd', 'csv_file');
    fprintf('步骤6已保存到: %s\n', step6_mat);
end

fprintf('\nID RMSE (vs tau_meas):\n');
if id_run_cad, fprintf('  CAD :'); fprintf(' %.3f', sqrt(mean((tau_cad-tau_meas).^2,1))); fprintf('\n'); end
if id_run_min, fprintf('  MIN :'); fprintf(' %.3f', sqrt(mean((tau_min-tau_meas).^2,1))); fprintf('\n'); end
if id_run_rec && ~isempty(pi_rec), fprintf('  REC :'); fprintf(' %.3f', sqrt(mean((tau_rec-tau_meas).^2,1))); fprintf('\n'); end
if id_run_phys && ~isempty(pi_phys), fprintf('  PHYS:'); fprintf(' %.3f', sqrt(mean((tau_phys-tau_meas).^2,1))); fprintf('\n'); end
if id_run_fd_m && ~isempty(pi_fd), fprintf('  FD  :'); fprintf(' %.3f', sqrt(mean((tau_fd-tau_meas).^2,1))); fprintf('\n'); end

fprintf('\nID Max|err| (vs tau_meas):\n');
if id_run_cad, fprintf('  CAD :'); fprintf(' %.3f', max(abs(tau_cad-tau_meas), [], 1)); fprintf('\n'); end
if id_run_min, fprintf('  MIN :'); fprintf(' %.3f', max(abs(tau_min-tau_meas), [], 1)); fprintf('\n'); end
if id_run_rec && ~isempty(pi_rec), fprintf('  REC :'); fprintf(' %.3f', max(abs(tau_rec-tau_meas), [], 1)); fprintf('\n'); end
if id_run_phys && ~isempty(pi_phys), fprintf('  PHYS:'); fprintf(' %.3f', max(abs(tau_phys-tau_meas), [], 1)); fprintf('\n'); end
if id_run_fd_m && ~isempty(pi_fd), fprintf('  FD  :'); fprintf(' %.3f', max(abs(tau_fd-tau_meas), [], 1)); fprintf('\n'); end

id_figs = struct();
if plot_id_compare
    plot_data = tau_meas;
    legend_txt = {'\tau_{meas}'};
    for ic = 1:numel(Lcmp)
        km = Lcmp{ic};
        switch km
            case 'cad'
                plot_data = [plot_data, tau_cad]; %#ok<AGROW>
                legend_txt{end+1} = 'Y\pi_{cad}'; %#ok<AGROW>
            case 'min'
                plot_data = [plot_data, tau_min]; %#ok<AGROW>
                legend_txt{end+1} = 'Y\pi_{min}'; %#ok<AGROW>
            case 'rec'
                if isempty(pi_rec), continue; end
                plot_data = [plot_data, tau_rec]; %#ok<AGROW>
                legend_txt{end+1} = 'Y\pi_{rec}'; %#ok<AGROW>
            case 'phys'
                if isempty(pi_phys), continue; end
                plot_data = [plot_data, tau_phys]; %#ok<AGROW>
                legend_txt{end+1} = 'Y\pi_{phys}'; %#ok<AGROW>
            case 'fd'
                if isempty(pi_fd), continue; end
                plot_data = [plot_data, tau_fd]; %#ok<AGROW>
                legend_txt{end+1} = 'Y\pi_{fd}'; %#ok<AGROW>
        end
    end
    id_figs = plot_compare_with_error_6dof((1:M)', tau_meas, '\tau_{meas}', ...
        plot_data(:, 7:end), legend_txt(2:end), 'torque', '逆动力学多模型');
end

%% 7) 步骤7：正动力学对比
M_fd = min(M, M_fd_max);
load_step7 = false;
if ~cfg.force_recompute_step7 && isfile(step7_mat)
    try
        S7 = load(step7_mat);
        if isfield(S7, 'csv_file') && isequal(S7.csv_file, csv_file) && isfield(S7, 't_fd') && numel(S7.t_fd)==M_fd
            t_fd = S7.t_fd;
            qd_ref = S7.qd_ref;
            qdd_ref = S7.qdd_ref;
            qd_cad = S7.qd_cad;
            qd_min = S7.qd_min;
            qd_rec = S7.qd_rec;
            qd_phys = S7.qd_phys;
            qd_fd = S7.qd_fd;
            qdd_cad = S7.qdd_cad;
            qdd_min = S7.qdd_min;
            qdd_rec = S7.qdd_rec;
            qdd_phys = S7.qdd_phys;
            qdd_fd = S7.qdd_fd;
            load_step7 = true;
            fprintf('[缓存] 已加载步骤7，跳过计算。\n');
        end
    catch ME
        fprintf('加载步骤7失败 (%s)，将重新计算。\n', ME.message);
    end
end
t_fd = dataset.t(1:M_fd);
qd_ref = dataset.qd(1:M_fd,:);
qdd_ref = dataset.qdd(1:M_fd,:);
if ~load_step7
    qdd_cad = zeros(M_fd,n); qdd_min = zeros(M_fd,n); qdd_rec = zeros(M_fd,n); qdd_phys = zeros(M_fd,n); qdd_fd = zeros(M_fd,n);
    qd_cad = zeros(M_fd,n); qd_min = zeros(M_fd,n); qd_rec = zeros(M_fd,n); qd_phys = zeros(M_fd,n); qd_fd = zeros(M_fd,n);
end
if ~exist('qdd_cad','var') || isempty(qdd_cad), qdd_cad = zeros(M_fd,n); end
if ~exist('qdd_min','var') || isempty(qdd_min), qdd_min = zeros(M_fd,n); end
if ~exist('qdd_rec','var') || isempty(qdd_rec), qdd_rec = zeros(M_fd,n); end
if ~exist('qdd_phys','var') || isempty(qdd_phys), qdd_phys = zeros(M_fd,n); end
if ~exist('qdd_fd','var') || isempty(qdd_fd), qdd_fd = zeros(M_fd,n); end
if ~exist('qd_cad','var') || isempty(qd_cad), qd_cad = zeros(M_fd,n); end
if ~exist('qd_min','var') || isempty(qd_min), qd_min = zeros(M_fd,n); end
if ~exist('qd_rec','var') || isempty(qd_rec), qd_rec = zeros(M_fd,n); end
if ~exist('qd_phys','var') || isempty(qd_phys), qd_phys = zeros(M_fd,n); end
if ~exist('qd_fd','var') || isempty(qd_fd), qd_fd = zeros(M_fd,n); end

recalc_fd_cad  = (cfg.force_recompute_step7 || ~load_step7) && id_run_cad;
recalc_fd_min  = (cfg.force_recompute_step7 || ~load_step7 || step4_changed) && id_run_min;
recalc_fd_rec  = (cfg.force_recompute_step7 || ~load_step7 || step5_changed) && id_run_rec && ~isempty(pi_rec);
recalc_fd_phys = (cfg.force_recompute_step7 || ~load_step7 || step5_changed) && id_run_phys && ~isempty(pi_phys);
recalc_fd_fd   = (cfg.force_recompute_step7 || ~load_step7 || step5_changed) && id_run_fd_m && ~isempty(pi_fd);

if recalc_fd_cad || recalc_fd_min || recalc_fd_rec || recalc_fd_phys || recalc_fd_fd
    model_cad_fd = struct('type','urdf','limb',limb,'para_order',para_order);
    model_min_fd = struct('type','min','limb',limb,'para_order',para_order,'X_hat',X_hat,'index_base',index_base);
    model_rec_fd = struct('type','full','limb',limb,'para_order',para_order,'pi_vec',pi_rec);
    if ~isempty(pi_phys), model_phys_fd = struct('type','full','limb',limb,'para_order',para_order,'pi_vec',pi_phys); end
    if ~isempty(pi_fd), model_fd_fd = struct('type','full','limb',limb,'para_order',para_order,'pi_vec',pi_fd); end

    fprintf('步骤7增量重算: CAD=%d, MIN=%d, REC=%d, PHYS=%d, FD=%d\n', ...
        recalc_fd_cad, recalc_fd_min, recalc_fd_rec, recalc_fd_phys, recalc_fd_fd);
    for k = 1:M_fd
        qk = dataset.q(k,:); qdk = dataset.qd(k,:); tauk = dataset.tau(k,:);
        if recalc_fd_cad, qdd_cad(k,:) = forward_dynamics_dispatch(qk, qdk, tauk, model_cad_fd, struct()).'; end
        if recalc_fd_min, qdd_min(k,:) = forward_dynamics_dispatch(qk, qdk, tauk, model_min_fd, struct()).'; end
        if recalc_fd_rec, qdd_rec(k,:) = forward_dynamics_dispatch(qk, qdk, tauk, model_rec_fd, struct()).'; end
        if recalc_fd_phys, qdd_phys(k,:) = forward_dynamics_dispatch(qk, qdk, tauk, model_phys_fd, struct()).'; end
        if recalc_fd_fd, qdd_fd(k,:) = forward_dynamics_dispatch(qk, qdk, tauk, model_fd_fd, struct()).'; end
    end
end

if recalc_fd_cad,  qd_cad = integrate_qd_from_qdd(t_fd, qd_ref(1,:), qdd_cad); end
if recalc_fd_min,  qd_min = integrate_qd_from_qdd(t_fd, qd_ref(1,:), qdd_min); end
if recalc_fd_rec,  qd_rec = integrate_qd_from_qdd(t_fd, qd_ref(1,:), qdd_rec); end
if recalc_fd_phys, qd_phys = integrate_qd_from_qdd(t_fd, qd_ref(1,:), qdd_phys); end
if recalc_fd_fd, qd_fd = integrate_qd_from_qdd(t_fd, qd_ref(1,:), qdd_fd); end

% 用于“速度对比/打印/Excel”的 qd 曲线：全程积分 vs H 步串联（由 cfg.fd_qd_compare_steps 控制）
if ~isfield(cfg, 'fd_qd_compare_steps') || isempty(cfg.fd_qd_compare_steps)
    H_qd_steps = inf;
else
    H_qd_steps = cfg.fd_qd_compare_steps;
end
use_qd_integral_display = ~(isscalar(H_qd_steps) && isnumeric(H_qd_steps) && isfinite(H_qd_steps) && H_qd_steps >= 1);
if use_qd_integral_display
    qd_disp_cad = qd_cad;
    qd_disp_min = qd_min;
    qd_disp_rec = qd_rec;
    qd_disp_phys = qd_phys;
    qd_disp_fd = qd_fd;
else
    H_eff = min(round(H_qd_steps), max(M_fd - 1, 1));
    qd_disp_cad  = build_qd_chain_from_measured_qd(t_fd, qd_ref, qdd_cad, H_eff);
    qd_disp_min  = build_qd_chain_from_measured_qd(t_fd, qd_ref, qdd_min, H_eff);
    qd_disp_rec  = build_qd_chain_from_measured_qd(t_fd, qd_ref, qdd_rec, H_eff);
    qd_disp_phys = build_qd_chain_from_measured_qd(t_fd, qd_ref, qdd_phys, H_eff);
    qd_disp_fd   = build_qd_chain_from_measured_qd(t_fd, qd_ref, qdd_fd, H_eff);
end

% 由积分速度得到关节角轨迹（初值与测量一致），用于 q 对比
q_ref_fd = dataset.q(1:M_fd, :);
q0_fd = q_ref_fd(1, :);
% 关节角 q 的生成：每一步使用上一时刻实测 q_ref(kk-1) 作为锚点
% q(kk) = q_ref(kk-1) + dt * qd_in(kk-1)
q_cad  = integrate_q_from_qd(t_fd, q0_fd, qd_cad, q_ref_fd);
q_min  = integrate_q_from_qd(t_fd, q0_fd, qd_min, q_ref_fd);
q_rec  = integrate_q_from_qd(t_fd, q0_fd, qd_rec, q_ref_fd);
if ~isempty(pi_phys)
    q_phys = integrate_q_from_qd(t_fd, q0_fd, qd_phys, q_ref_fd);
else
    q_phys = zeros(M_fd, n);
end
if ~isempty(pi_fd)
    q_fd = integrate_q_from_qd(t_fd, q0_fd, qd_fd, q_ref_fd);
else
    q_fd = zeros(M_fd, n);
end

if recalc_fd_cad || recalc_fd_min || recalc_fd_rec || recalc_fd_phys || recalc_fd_fd
    save(step7_mat, 't_fd', 'qd_ref', 'qdd_ref', 'qd_cad', 'qd_min', 'qd_rec', 'qd_phys', 'qd_fd', ...
        'qdd_cad', 'qdd_min', 'qdd_rec', 'qdd_phys', 'qdd_fd', 'csv_file');
    fprintf('步骤7已保存到: %s\n', step7_mat);
end

fprintf('\nFD RMSE (vs qdd_ref):\n');
if id_run_cad, fprintf('  CAD :'); fprintf(' %.3f', sqrt(mean((qdd_cad-qdd_ref).^2,1))); fprintf('\n'); end
if id_run_min, fprintf('  MIN :'); fprintf(' %.3f', sqrt(mean((qdd_min-qdd_ref).^2,1))); fprintf('\n'); end
if id_run_rec && ~isempty(pi_rec), fprintf('  REC :'); fprintf(' %.3f', sqrt(mean((qdd_rec-qdd_ref).^2,1))); fprintf('\n'); end
if id_run_phys && ~isempty(pi_phys), fprintf('  PHYS:'); fprintf(' %.3f', sqrt(mean((qdd_phys-qdd_ref).^2,1))); fprintf('\n'); end
if id_run_fd_m && ~isempty(pi_fd), fprintf('  FD  :'); fprintf(' %.3f', sqrt(mean((qdd_fd-qdd_ref).^2,1))); fprintf('\n'); end

if use_qd_integral_display
    fprintf('\nFD 速度RMSE (vs qd_ref，全程积分模型qdd→qd):\n');
else
    fprintf('\nFD 速度RMSE (vs qd_ref，H=%d 步串联预测，起点为实测 qd(k-H)):\n', round(H_qd_steps));
end
if id_run_cad, fprintf('  CAD :'); fprintf(' %.3f', sqrt(mean((qd_disp_cad-qd_ref).^2,1))); fprintf('\n'); end
if id_run_min, fprintf('  MIN :'); fprintf(' %.3f', sqrt(mean((qd_disp_min-qd_ref).^2,1))); fprintf('\n'); end
if id_run_rec && ~isempty(pi_rec), fprintf('  REC :'); fprintf(' %.3f', sqrt(mean((qd_disp_rec-qd_ref).^2,1))); fprintf('\n'); end
if id_run_phys && ~isempty(pi_phys), fprintf('  PHYS:'); fprintf(' %.3f', sqrt(mean((qd_disp_phys-qd_ref).^2,1))); fprintf('\n'); end
if id_run_fd_m && ~isempty(pi_fd), fprintf('  FD  :'); fprintf(' %.3f', sqrt(mean((qd_disp_fd-qd_ref).^2,1))); fprintf('\n'); end
if ~use_qd_integral_display
    fprintf('  (对照) 全程积分 qd RMSE —');
    if id_run_cad, fprintf(' CAD:'); fprintf(' %.3f', sqrt(mean((qd_cad-qd_ref).^2,1))); end
    if id_run_min, fprintf(' MIN:'); fprintf(' %.3f', sqrt(mean((qd_min-qd_ref).^2,1))); end
    if id_run_rec && ~isempty(pi_rec), fprintf(' REC:'); fprintf(' %.3f', sqrt(mean((qd_rec-qd_ref).^2,1))); end
    if id_run_phys && ~isempty(pi_phys), fprintf(' PHYS:'); fprintf(' %.3f', sqrt(mean((qd_phys-qd_ref).^2,1))); end
    if id_run_fd_m && ~isempty(pi_fd), fprintf(' FD:'); fprintf(' %.3f', sqrt(mean((qd_fd-qd_ref).^2,1))); end
    fprintf('\n');
end

fprintf('\nFD 位置RMSE (vs q_meas，由积分 qd 得到):\n');
if id_run_cad, fprintf('  CAD :'); fprintf(' %.3f', sqrt(mean((q_cad-q_ref_fd).^2,1))); fprintf('\n'); end
if id_run_min, fprintf('  MIN :'); fprintf(' %.3f', sqrt(mean((q_min-q_ref_fd).^2,1))); fprintf('\n'); end
if id_run_rec && ~isempty(pi_rec), fprintf('  REC :'); fprintf(' %.3f', sqrt(mean((q_rec-q_ref_fd).^2,1))); fprintf('\n'); end
if id_run_phys && ~isempty(pi_phys), fprintf('  PHYS:'); fprintf(' %.3f', sqrt(mean((q_phys-q_ref_fd).^2,1))); fprintf('\n'); end
if id_run_fd_m && ~isempty(pi_fd), fprintf('  FD  :'); fprintf(' %.3f', sqrt(mean((q_fd-q_ref_fd).^2,1))); fprintf('\n'); end

%% 7.1) 质量矩阵数值健康度（full 参数模型）
mass_stats = struct();
if id_run_cad && ~isempty(pi_cad)
    mass_stats.cad = collect_mass_matrix_stats(dataset, 1:M_fd, pi_cad, limb, para_order);
end
if id_run_rec && ~isempty(pi_rec)
    mass_stats.rec = collect_mass_matrix_stats(dataset, 1:M_fd, pi_rec, limb, para_order);
end
if id_run_phys && ~isempty(pi_phys)
    mass_stats.phys = collect_mass_matrix_stats(dataset, 1:M_fd, pi_phys, limb, para_order);
end
if id_run_fd_m && ~isempty(pi_fd)
    mass_stats.fd = collect_mass_matrix_stats(dataset, 1:M_fd, pi_fd, limb, para_order);
end

fprintf('\nM(q) 数值健康度（FD关键）:\n');
if isfield(mass_stats, 'cad')
    fprintf('  CAD : minEig[min/med]=%.3e / %.3e, cond[max/med]=%.3e / %.3e, fail_rate=%.2f%%\n', ...
        mass_stats.cad.min_eig_min, mass_stats.cad.min_eig_median, ...
        mass_stats.cad.cond_max, mass_stats.cad.cond_median, 100*mass_stats.cad.fail_rate);
end
if isfield(mass_stats, 'rec')
    fprintf('  REC : minEig[min/med]=%.3e / %.3e, cond[max/med]=%.3e / %.3e, fail_rate=%.2f%%\n', ...
        mass_stats.rec.min_eig_min, mass_stats.rec.min_eig_median, ...
        mass_stats.rec.cond_max, mass_stats.rec.cond_median, 100*mass_stats.rec.fail_rate);
end
if isfield(mass_stats, 'phys')
    fprintf('  PHYS: minEig[min/med]=%.3e / %.3e, cond[max/med]=%.3e / %.3e, fail_rate=%.2f%%\n', ...
        mass_stats.phys.min_eig_min, mass_stats.phys.min_eig_median, ...
        mass_stats.phys.cond_max, mass_stats.phys.cond_median, 100*mass_stats.phys.fail_rate);
end
if isfield(mass_stats, 'fd')
    fprintf('  FD  : minEig[min/med]=%.3e / %.3e, cond[max/med]=%.3e / %.3e, fail_rate=%.2f%%\n', ...
        mass_stats.fd.min_eig_min, mass_stats.fd.min_eig_median, ...
        mass_stats.fd.cond_max, mass_stats.fd.cond_median, 100*mass_stats.fd.fail_rate);
end

fd_figs = struct();
if plot_fd_compare
    plot_data_q = q_ref_fd;
    legend_txt_q = {'q_{meas}'};
    for ic = 1:numel(Lcmp)
        km = Lcmp{ic};
        switch km
            case 'cad'
                plot_data_q = [plot_data_q, q_cad]; %#ok<*AGROW>
                legend_txt_q{end+1} = 'int(FD(CAD))'; %#ok<*AGROW>
            case 'min'
                plot_data_q = [plot_data_q, q_min];
                legend_txt_q{end+1} = 'int(FD(MIN))';
            case 'rec'
                if isempty(pi_rec), continue; end
                plot_data_q = [plot_data_q, q_rec];
                legend_txt_q{end+1} = 'int(FD(REC))';
            case 'phys'
                if isempty(pi_phys), continue; end
                plot_data_q = [plot_data_q, q_phys];
                legend_txt_q{end+1} = 'int(FD(\pi_{phys}))';
            case 'fd'
                if isempty(pi_fd), continue; end
                plot_data_q = [plot_data_q, q_fd];
                legend_txt_q{end+1} = 'int(FD(\pi_{fd}))';
        end
    end
    fd_figs_q = plot_compare_with_error_6dof(t_fd, q_ref_fd, 'q_{meas}', ...
        plot_data_q(:, 7:end), legend_txt_q(2:end), 'q', '正动力学关节角多模型');

    plot_data_qd = qd_ref;
    legend_txt_qd = {'qd_{ref}'};
    if use_qd_integral_display
        qd_fig_title = '正动力学速度多模型(全程积分)';
    else
        qd_fig_title = sprintf('正动力学速度多模型(H=%d步串联)', round(H_qd_steps));
    end
    Hr = round(H_qd_steps);
    for ic = 1:numel(Lcmp)
        km = Lcmp{ic};
        switch km
            case 'cad'
                plot_data_qd = [plot_data_qd, qd_disp_cad];
                if use_qd_integral_display, legend_txt_qd{end+1} = 'int(FD(CAD))';
                else, legend_txt_qd{end+1} = sprintf('H=%d(FD(CAD))', Hr); end
            case 'min'
                plot_data_qd = [plot_data_qd, qd_disp_min];
                if use_qd_integral_display, legend_txt_qd{end+1} = 'int(FD(MIN))';
                else, legend_txt_qd{end+1} = sprintf('H=%d(FD(MIN))', Hr); end
            case 'rec'
                if isempty(pi_rec), continue; end
                plot_data_qd = [plot_data_qd, qd_disp_rec];
                if use_qd_integral_display, legend_txt_qd{end+1} = 'int(FD(REC))';
                else, legend_txt_qd{end+1} = sprintf('H=%d(FD(REC))', Hr); end
            case 'phys'
                if isempty(pi_phys), continue; end
                plot_data_qd = [plot_data_qd, qd_disp_phys];
                if use_qd_integral_display, legend_txt_qd{end+1} = 'int(FD(\pi_{phys}))';
                else, legend_txt_qd{end+1} = sprintf('H=%d(\\pi_{phys})', Hr); end
            case 'fd'
                if isempty(pi_fd), continue; end
                plot_data_qd = [plot_data_qd, qd_disp_fd];
                if use_qd_integral_display, legend_txt_qd{end+1} = 'int(FD(\pi_{fd}))';
                else, legend_txt_qd{end+1} = sprintf('H=%d(\\pi_{fd})', Hr); end
        end
    end
    fd_figs_qd = plot_compare_with_error_6dof(t_fd, qd_ref, 'qd_{ref}', ...
        plot_data_qd(:, 7:end), legend_txt_qd(2:end), 'qd', qd_fig_title);

    plot_data_qdd = qdd_ref;
    legend_txt_qdd = {'qdd_{ref}'};
    for ic = 1:numel(Lcmp)
        km = Lcmp{ic};
        switch km
            case 'cad'
                plot_data_qdd = [plot_data_qdd, qdd_cad];
                legend_txt_qdd{end+1} = 'FD(CAD)';
            case 'min'
                plot_data_qdd = [plot_data_qdd, qdd_min];
                legend_txt_qdd{end+1} = 'FD(MIN)';
            case 'rec'
                if isempty(pi_rec), continue; end
                plot_data_qdd = [plot_data_qdd, qdd_rec];
                legend_txt_qdd{end+1} = 'FD(REC)';
            case 'phys'
                if isempty(pi_phys), continue; end
                plot_data_qdd = [plot_data_qdd, qdd_phys];
                legend_txt_qdd{end+1} = 'FD(\pi_{phys})';
            case 'fd'
                if isempty(pi_fd), continue; end
                plot_data_qdd = [plot_data_qdd, qdd_fd];
                legend_txt_qdd{end+1} = 'FD(\pi_{fd})';
        end
    end
    fd_figs_qdd = plot_compare_with_error_6dof(t_fd, qdd_ref, 'qdd_{ref}', ...
        plot_data_qdd(:, 7:end), legend_txt_qdd(2:end), 'qdd', '正动力学多模型');
    fd_figs.q = fd_figs_q;
    fd_figs.qd = fd_figs_qd;
    fd_figs.qdd = fd_figs_qdd;
end

if save_excel
    id_rmse = struct();
    if id_run_cad, id_rmse.cad = sqrt(mean((tau_cad - tau_meas).^2,1)); end
    if id_run_min, id_rmse.min = sqrt(mean((tau_min - tau_meas).^2,1)); end
    if id_run_rec && ~isempty(pi_rec), id_rmse.rec = sqrt(mean((tau_rec - tau_meas).^2,1)); end
    if id_run_phys && ~isempty(pi_phys), id_rmse.phys = sqrt(mean((tau_phys - tau_meas).^2,1)); end
    if id_run_fd_m && ~isempty(pi_fd), id_rmse.fd = sqrt(mean((tau_fd - tau_meas).^2,1)); end
    id_maxerr = struct();
    if id_run_cad, id_maxerr.cad = max(abs(tau_cad - tau_meas), [], 1); end
    if id_run_min, id_maxerr.min = max(abs(tau_min - tau_meas), [], 1); end
    if id_run_rec && ~isempty(pi_rec), id_maxerr.rec = max(abs(tau_rec - tau_meas), [], 1); end
    if id_run_phys && ~isempty(pi_phys), id_maxerr.phys = max(abs(tau_phys - tau_meas), [], 1); end
    if id_run_fd_m && ~isempty(pi_fd), id_maxerr.fd = max(abs(tau_fd - tau_meas), [], 1); end

    fd_rmse = struct();
    if id_run_cad, fd_rmse.qd_cad = sqrt(mean((qd_disp_cad - qd_ref).^2,1)); end
    if id_run_min, fd_rmse.qd_min = sqrt(mean((qd_disp_min - qd_ref).^2,1)); end
    if id_run_rec && ~isempty(pi_rec), fd_rmse.qd_rec = sqrt(mean((qd_disp_rec - qd_ref).^2,1)); end
    if id_run_phys && ~isempty(pi_phys), fd_rmse.qd_phys = sqrt(mean((qd_disp_phys - qd_ref).^2,1)); end
    if id_run_fd_m && ~isempty(pi_fd), fd_rmse.qd_fd = sqrt(mean((qd_disp_fd - qd_ref).^2,1)); end
    if ~use_qd_integral_display
        if id_run_cad, fd_rmse.qd_int_cad = sqrt(mean((qd_cad - qd_ref).^2,1)); end
        if id_run_min, fd_rmse.qd_int_min = sqrt(mean((qd_min - qd_ref).^2,1)); end
        if id_run_rec && ~isempty(pi_rec), fd_rmse.qd_int_rec = sqrt(mean((qd_rec - qd_ref).^2,1)); end
        if id_run_phys && ~isempty(pi_phys), fd_rmse.qd_int_phys = sqrt(mean((qd_phys - qd_ref).^2,1)); end
        if id_run_fd_m && ~isempty(pi_fd), fd_rmse.qd_int_fd = sqrt(mean((qd_fd - qd_ref).^2,1)); end
    end
    if id_run_cad, fd_rmse.cad = sqrt(mean((qdd_cad - qdd_ref).^2,1)); end
    if id_run_min, fd_rmse.min = sqrt(mean((qdd_min - qdd_ref).^2,1)); end
    if id_run_rec && ~isempty(pi_rec), fd_rmse.rec = sqrt(mean((qdd_rec - qdd_ref).^2,1)); end
    if id_run_phys && ~isempty(pi_phys), fd_rmse.phys = sqrt(mean((qdd_phys - qdd_ref).^2,1)); end
    if id_run_fd_m && ~isempty(pi_fd), fd_rmse.fd = sqrt(mean((qdd_fd - qdd_ref).^2,1)); end
    % 积分 q 相对测量 q 的 RMSE（与控制台 “FD 位置RMSE” 一致）
    q_ref_fd_excel = dataset.q(1:M_fd, :);
    if id_run_cad, fd_rmse.pos_cad = sqrt(mean((q_cad - q_ref_fd_excel).^2,1)); end
    if id_run_min, fd_rmse.pos_min = sqrt(mean((q_min - q_ref_fd_excel).^2,1)); end
    if id_run_rec && ~isempty(pi_rec), fd_rmse.pos_rec = sqrt(mean((q_rec - q_ref_fd_excel).^2,1)); end
    if id_run_phys && ~isempty(pi_phys), fd_rmse.pos_phys = sqrt(mean((q_phys - q_ref_fd_excel).^2,1)); end
    if id_run_fd_m && ~isempty(pi_fd), fd_rmse.pos_fd = sqrt(mean((q_fd - q_ref_fd_excel).^2,1)); end
    fd_maxerr = struct();
    if id_run_cad, fd_maxerr.qd_cad = max(abs(qd_disp_cad - qd_ref), [], 1); end
    if id_run_min, fd_maxerr.qd_min = max(abs(qd_disp_min - qd_ref), [], 1); end
    if id_run_rec && ~isempty(pi_rec), fd_maxerr.qd_rec = max(abs(qd_disp_rec - qd_ref), [], 1); end
    if id_run_phys && ~isempty(pi_phys), fd_maxerr.qd_phys = max(abs(qd_disp_phys - qd_ref), [], 1); end
    if id_run_fd_m && ~isempty(pi_fd), fd_maxerr.qd_fd = max(abs(qd_disp_fd - qd_ref), [], 1); end
    if ~use_qd_integral_display
        if id_run_cad, fd_maxerr.qd_int_cad = max(abs(qd_cad - qd_ref), [], 1); end
        if id_run_min, fd_maxerr.qd_int_min = max(abs(qd_min - qd_ref), [], 1); end
        if id_run_rec && ~isempty(pi_rec), fd_maxerr.qd_int_rec = max(abs(qd_rec - qd_ref), [], 1); end
        if id_run_phys && ~isempty(pi_phys), fd_maxerr.qd_int_phys = max(abs(qd_phys - qd_ref), [], 1); end
        if id_run_fd_m && ~isempty(pi_fd), fd_maxerr.qd_int_fd = max(abs(qd_fd - qd_ref), [], 1); end
    end
    if id_run_cad, fd_maxerr.cad = max(abs(qdd_cad - qdd_ref), [], 1); end
    if id_run_min, fd_maxerr.min = max(abs(qdd_min - qdd_ref), [], 1); end
    if id_run_rec && ~isempty(pi_rec), fd_maxerr.rec = max(abs(qdd_rec - qdd_ref), [], 1); end
    if id_run_phys && ~isempty(pi_phys), fd_maxerr.phys = max(abs(qdd_phys - qdd_ref), [], 1); end
    if id_run_fd_m && ~isempty(pi_fd), fd_maxerr.fd = max(abs(qdd_fd - qdd_ref), [], 1); end
    if id_run_cad, fd_maxerr.pos_cad = max(abs(q_cad - q_ref_fd_excel), [], 1); end
    if id_run_min, fd_maxerr.pos_min = max(abs(q_min - q_ref_fd_excel), [], 1); end
    if id_run_rec && ~isempty(pi_rec), fd_maxerr.pos_rec = max(abs(q_rec - q_ref_fd_excel), [], 1); end
    if id_run_phys && ~isempty(pi_phys), fd_maxerr.pos_phys = max(abs(q_phys - q_ref_fd_excel), [], 1); end
    if id_run_fd_m && ~isempty(pi_fd), fd_maxerr.pos_fd = max(abs(q_fd - q_ref_fd_excel), [], 1); end

    meta_tbl = table(string(csv_file), size(dataset.q,1), M_fd, ...
        'VariableNames', {'csv_file','num_samples','num_fd_samples'});
    out_xlsx = fullfile(build_dir, 'full_dynamics_validation_summary.xlsx');
    export_opts = struct();
    export_opts.id_maxerr = id_maxerr;
    export_opts.fd_maxerr = fd_maxerr;

    % 7.1) 质量矩阵数值健康度（M(q)）写入 Excel，并给出每个字段含义
    % 数值定义来自 collect_mass_matrix_stats：
    %   min_eig_min/min_eig_median/cond_max/cond_median 统计的是 M(q) 的特征值/条件数
    %   fail_rate 为 forward_dynamics_full 失败比例
    metric_names = {'min_eig_min','min_eig_median','cond_max','cond_median','n_points','n_valid','n_fail','fail_rate'};
    metric_meaning = { ...
        'min_eig_min: 在采样点上 minEig(M(q)) 的最小值'; ...
        'min_eig_median: 在采样点上 minEig(M(q)) 的中位数'; ...
        'cond_max: 在采样点上 cond(M(q)) 的最大值'; ...
        'cond_median: 在采样点上 cond(M(q)) 的中位数'; ...
        'n_points: 采样点个数（用于健康度统计）'; ...
        'n_valid: 成功计算 M(q) 的采样点个数'; ...
        'n_fail: 计算失败的采样点个数'; ...
        'fail_rate: 失败比例 n_fail/n_points' ...
    };

    cad_vals  = zeros(numel(metric_names), 1);
    rec_vals  = zeros(numel(metric_names), 1);
    phys_vals = nan(numel(metric_names), 1);
    fd_vals   = nan(numel(metric_names), 1);
    for ii = 1:numel(metric_names)
        mn = metric_names{ii};
        if isfield(mass_stats, 'cad') && isfield(mass_stats.cad, mn) && ~isempty(mass_stats.cad.(mn))
            cad_vals(ii) = mass_stats.cad.(mn);
        else
            cad_vals(ii) = nan;
        end
        if isfield(mass_stats, 'rec') && isfield(mass_stats.rec, mn) && ~isempty(mass_stats.rec.(mn))
            rec_vals(ii) = mass_stats.rec.(mn);
        else
            rec_vals(ii) = nan;
        end
        if isfield(mass_stats, 'phys') && isfield(mass_stats.phys, mn) && ~isempty(mass_stats.phys.(mn))
            phys_vals(ii) = mass_stats.phys.(mn);
        else
            phys_vals(ii) = nan;
        end
        if isfield(mass_stats, 'fd') && isfield(mass_stats.fd, mn) && ~isempty(mass_stats.fd.(mn))
            fd_vals(ii) = mass_stats.fd.(mn);
        else
            fd_vals(ii) = nan;
        end
    end

    mass_health_table = table( ...
        string(metric_names(:)), cad_vals, rec_vals, phys_vals, fd_vals, string(metric_meaning(:)), ...
        'VariableNames', {'metric','CAD','MIN','PHYS','FD','meaning'});
    export_opts.mass_health_table = mass_health_table;
    % 7.2) 各模型辨识到的动力学参数（pi_cad/pi_rec/pi_phys/pi_fd）写入新 sheet
    try
        [robot_limb, ~] = get_e1_limb_robot(limb);
        bodyNames = cell(1, n);
        for i = 1:n
            bodyNames{i} = robot_limb.Bodies{i}.Name;
        end

        if para_order == 1
            comp_names = {'m','mx','my','mz','Ixx','Ixy','Ixz','Iyy','Iyz','Izz'};
        else
            comp_names = {'Ixx','Ixy','Ixz','Iyy','Iyz','Izz','mx','my','mz','m'};
        end

        L = 10 * n;
        % 统一把 param/pi_* 强制对齐到相同行数，避免 table() 报 “所有表变量必须具有相同的行数”
        param_name = cell(L, 1);
        for i = 1:n
            for j = 1:10
                idx = (i - 1) * 10 + j;
                param_name{idx} = sprintf('%s_%s', bodyNames{i}, comp_names{j});
            end
        end

        cad_vals  = nan(L, 1);
        rec_vals  = nan(L, 1);
        phys_vals = nan(L, 1);
        fd_vals   = nan(L, 1);

        if ~isempty(pi_cad)
            v = pi_cad(:);
            m = min(numel(v), L);
            cad_vals(1:m) = v(1:m);
        end
        if ~isempty(pi_rec)
            v = pi_rec(:);
            m = min(numel(v), L);
            rec_vals(1:m) = v(1:m);
        end
        if ~isempty(pi_phys)
            v = pi_phys(:);
            m = min(numel(v), L);
            phys_vals(1:m) = v(1:m);
        end
        if ~isempty(pi_fd)
            v = pi_fd(:);
            m = min(numel(v), L);
            fd_vals(1:m) = v(1:m);
        end

        dyn_params_table = table(param_name, cad_vals, rec_vals, phys_vals, fd_vals, ...
            'VariableNames', {'param','pi_cad','pi_rec','pi_phys','pi_fd'});
        export_opts.dyn_params_table = dyn_params_table;
    catch ME
        fprintf('写入动力学参数 sheet 失败：%s\n', ME.message);
    end

    id_plot_tbl = table();
    if export_id_plot_png_to_excel && plot_id_compare && isfield(id_figs, 'compare') && isfield(id_figs, 'error')
        id_plot_compare_png = fullfile(build_dir, 'id_compare_plot.png');
        id_plot_error_png = fullfile(build_dir, 'id_error_plot.png');
        try
            saveas(id_figs.compare, id_plot_compare_png);
            saveas(id_figs.error, id_plot_error_png);
            id_plot_tbl = table( ...
                string(id_plot_compare_png), ...
                string(id_plot_error_png), ...
                'VariableNames', {'id_compare_png', 'id_error_png'});
            fprintf('ID图路径准备写入Excel工作表 ID_PLOTS\n');
        catch ME
            fprintf('写入ID图到Excel失败：%s\n', ME.message);
        end
    end
    if ~isempty(id_plot_tbl)
        export_opts.id_plot_files = id_plot_tbl;
    end

    fd_plot_tbl = table();
    if plot_fd_compare && isfield(fd_figs, 'qdd') && isfield(fd_figs.qdd, 'compare') && isfield(fd_figs.qdd, 'error')
        fd_plot_compare_png = fullfile(build_dir, 'fd_compare_plot.png');
        fd_plot_error_png = fullfile(build_dir, 'fd_error_plot.png');
        fd_q_compare_png = '';
        fd_q_error_png = '';
        try
            saveas(fd_figs.qdd.compare, fd_plot_compare_png);
            saveas(fd_figs.qdd.error, fd_plot_error_png);
            if isfield(fd_figs, 'q') && isfield(fd_figs.q, 'compare') && isfield(fd_figs.q, 'error')
                fd_q_compare_png = fullfile(build_dir, 'fd_q_compare_plot.png');
                fd_q_error_png = fullfile(build_dir, 'fd_q_error_plot.png');
                saveas(fd_figs.q.compare, fd_q_compare_png);
                saveas(fd_figs.q.error, fd_q_error_png);
            end
            if ~isempty(fd_q_compare_png)
                fd_plot_tbl = table( ...
                    string(fd_plot_compare_png), ...
                    string(fd_plot_error_png), ...
                    string(fd_q_compare_png), ...
                    string(fd_q_error_png), ...
                    'VariableNames', {'fd_compare_png', 'fd_error_png', 'fd_q_compare_png', 'fd_q_error_png'});
            else
                fd_plot_tbl = table( ...
                    string(fd_plot_compare_png), ...
                    string(fd_plot_error_png), ...
                    'VariableNames', {'fd_compare_png', 'fd_error_png'});
            end
            fprintf('FD图路径准备写入Excel工作表 FD_PLOTS\n');
        catch ME
            fprintf('写入FD图到Excel失败：%s\n', ME.message);
        end
    end
    if ~isempty(fd_plot_tbl)
        export_opts.fd_plot_files = fd_plot_tbl;
    end

    % 方案 C：统一结果汇总（追加一行）
    try
        if ~isfield(cfg, 'do_append_result_summary') || cfg.do_append_result_summary
            summary_file = fullfile(build_dir, 'scheme_result_summary');

            row_for_joint = [];
            if isfield(meta.prep_used, 'row_for_joint')
                row_for_joint = meta.prep_used.row_for_joint;
            end
            row_for_joint = row_for_joint(:).';
            % use_friction_56：方案对比用指标。仅 row_for_joint 像「5/6 用摩擦行」不够——若未读表或未启用补偿则应为 false
            pu_sum = meta.prep_used;
            pattern_joint56_friction = false;
            if numel(row_for_joint) >= 6
                pattern_joint56_friction = all(row_for_joint(1:4) == 0) && any(row_for_joint(5:6) ~= 0);
            end
            do_comp_sum = isfield(pu_sum, 'do_compensation') && pu_sum.do_compensation;
            has_friction_params = isfield(pu_sum, 'friction_params') && ~isempty(pu_sum.friction_params);
            use_friction_56 = pattern_joint56_friction && do_comp_sum && has_friction_params;

            use_mix_params = (para_order == 2);
            feasible = true;
            if id_run_rec && exist('res_norm_rec','var') && ~isempty(res_norm_rec) && isfinite(res_norm_rec)
                feasible = (res_norm_rec < 1); % 经验阈值：越小越可接受
            end

            result_struct = struct();
            result_struct.scheme_name = 'full_dynamics_validation';
            result_struct.limb = limb;
            result_struct.para_order = para_order;
            result_struct.use_friction_56 = use_friction_56;
            result_struct.use_mix_params = use_mix_params;
            result_struct.feasible = feasible;

            % ---------- 辨识/预处理设置（用于方案对比） ----------
            % 预处理补偿相关
            try
                if isfield(meta, 'prep_used') && isstruct(meta.prep_used)
                    pu = meta.prep_used;
                    if isfield(pu, 'do_compensation'), result_struct.prep_do_compensation = pu.do_compensation; end
                    if isfield(pu, 'load_friction_from_summary'), result_struct.prep_load_friction_from_summary = pu.load_friction_from_summary; end
                    if isfield(pu, 'row_for_joint'), result_struct.prep_row_for_joint = row_for_joint; end
                end
            catch
            end

            % Step4：最小参数辨识
            if isfield(cfg, 'min_id_opts') && isstruct(cfg.min_id_opts)
                if isfield(cfg.min_id_opts, 'use_wls'), result_struct.min_use_wls = cfg.min_id_opts.use_wls; end
                if isfield(cfg.min_id_opts, 'lambda_ridge'), result_struct.min_lambda_ridge = cfg.min_id_opts.lambda_ridge; end
            end

            % Step5：PHYS 物理约束辨识
            if isfield(cfg, 'phys_opts') && isstruct(cfg.phys_opts)
                if isfield(cfg.phys_opts, 'lambda_phys'), result_struct.phys_lambda_phys = cfg.phys_opts.lambda_phys; end
                if isfield(cfg.phys_opts, 'lambda_qd'), result_struct.phys_lambda_qd = cfg.phys_opts.lambda_qd; end
                if isfield(cfg.phys_opts, 'lambda_M'), result_struct.phys_lambda_M = cfg.phys_opts.lambda_M; end
                if isfield(cfg.phys_opts, 'algorithm'), result_struct.phys_algorithm = cfg.phys_opts.algorithm; end
                if isfield(cfg.phys_opts, 'max_iter'), result_struct.phys_max_iter = cfg.phys_opts.max_iter; end
            end

            % 验证侧：qd 展示步数/积分方式
            if isfield(cfg, 'fd_qd_compare_steps')
                result_struct.fd_qd_compare_steps = cfg.fd_qd_compare_steps;
            end
            result_struct.run_pi_phys = run_pi_phys;
            result_struct.run_pi_fd = run_pi_fd;
            result_struct.use_fast_fd_id_tuning = use_fast_fd_id_tuning;
            result_struct.compare_models = strjoin(Lcmp, ',');
            % ---------------------------------------------------
            pref_summary = {'phys','rec','min','cad','fd'};
            result_struct.id_rmse = summary_pick_mean_row(id_rmse, pref_summary, Lcmp);
            result_struct.fd_qdd_rmse = summary_pick_mean_fd(fd_rmse, pref_summary, Lcmp, 'qdd');
            result_struct.fd_qd_rmse_h5 = summary_pick_mean_fd(fd_rmse, pref_summary, Lcmp, 'qd');
            result_struct.fd_qd_rmse_full = summary_pick_mean_fd(fd_rmse, pref_summary, Lcmp, 'qd_int');
            result_struct.fd_q_rmse_full = summary_pick_mean_fd(fd_rmse, pref_summary, Lcmp, 'pos');
            result_struct.mass_minEig_min = summary_pick_mass_scalar(mass_stats, pref_summary, Lcmp, 'min_eig_min');
            result_struct.mass_cond_med = summary_pick_mass_scalar(mass_stats, pref_summary, Lcmp, 'cond_median');
            result_struct.notes = '';

            % 辨识参数（完整保存在 .mat；.csv 中会写 norm/n）
            result_struct.pi_cad = pi_cad(:);
            result_struct.pi_rec = pi_rec(:);
            result_struct.pi_phys = [];
            result_struct.pi_fd = [];
            if exist('pi_phys','var') && ~isempty(pi_phys), result_struct.pi_phys = pi_phys(:); end
            if exist('pi_fd','var') && ~isempty(pi_fd), result_struct.pi_fd = pi_fd(:); end

            append_result_summary(summary_file, result_struct);
            fprintf('结果汇总已追加到: %s\n', [summary_file '.mat/.csv']);
        end
    catch ME
        fprintf('追加结果汇总失败：%s\n', ME.message);
    end

    export_validation_summary_excel(out_xlsx, id_rmse, fd_rmse, meta_tbl, export_opts);
    fprintf('Excel汇总已保存到: %s\n', out_xlsx);

    % 附加保存 M(q) 数值指标（便于后续批量筛选）
    out_mass_mat = fullfile(build_dir, 'full_dynamics_mass_matrix_stats.mat');
    save(out_mass_mat, 'mass_stats', 'csv_file');
    fprintf('质量矩阵指标已保存到: %s\n', out_mass_mat);
end

%% 8) 最终输出（兼容后续脚本）
out_mat = fullfile(build_dir, 'min_param_id_result.mat');
prep_opts = meta.prep_used;
avg_data = dataset.avg_data; %#ok<NASGU>
metrics = res_min.metrics; %#ok<NASGU>
save(out_mat, 'X_hat', 'index_base', 'avg_data', 'metrics', 'prep_opts');
fprintf('最终结果已保存到: %s\n', out_mat);
fprintf('\n===== 各步骤保存位置 (build/) =====\n');
fprintf('  步骤2 预处理: %s\n', step2_mat);
fprintf('  步骤3 数据集: %s\n', step3_mat);
fprintf('  步骤4 最小参数: %s\n', step4_mat);
fprintf('  步骤5 全参数: %s\n', step5_mat);
fprintf('  步骤6 逆动力学: %s\n', step6_mat);
fprintf('  步骤7 正动力学: %s\n', step7_mat);
fprintf('  最终结果: %s\n', out_mat);
fprintf('===== 全流程对比完成 =====\n');

function qd_out = build_qd_chain_from_measured_qd(t_vec, qd_meas, qdd_model, H)
% H 步串联：qd_pred(k)=qd_meas(k-H)+sum_{j=0}^{H-1} dt(s+j)*qdd(s+j)，s=k-H；前 H 行与测量一致。
% H=1 时与 PHYS 目标中的单步 J_qd 一致。各 qdd_model(j) 仍对应该时刻的实测状态算出的模型加速度。
    t_vec = t_vec(:);
    Mloc = size(qd_meas, 1);
    nloc = size(qd_meas, 2);
    qd_out = zeros(Mloc, nloc);
    if Mloc < 1, return; end
    H = max(1, round(H));
    H = min(H, max(Mloc - 1, 1));
    qd_out(1:min(H, Mloc), :) = qd_meas(1:min(H, Mloc), :);
    if Mloc <= H, return; end
    dt_med = median(diff(t_vec));
    if dt_med <= 0 || isnan(dt_med), dt_med = 0.002; end
    for k = (H + 1):Mloc
        s = k - H;
        qv = qd_meas(s, :);
        for j = 0:(H - 1)
            dt_k = t_vec(s + j + 1) - t_vec(s + j);
            if ~isfinite(dt_k) || dt_k <= 0, dt_k = dt_med; end
            qv = qv + dt_k * qdd_model(s + j, :);
        end
        qd_out(k, :) = qv;
    end
end

function qd_out = integrate_qd_from_qdd(t_vec, qd0, qdd_in)
Mloc = size(qdd_in, 1);
nloc = size(qdd_in, 2);
qd_out = zeros(Mloc, nloc);
if Mloc <= 0, return; end
qd_out(1,:) = qd0;
if Mloc == 1, return; end
dt_med = median(diff(t_vec));
if dt_med <= 0 || isnan(dt_med), dt_med = 0.002; end
for kk = 2:Mloc
    dt_k = t_vec(kk) - t_vec(kk-1);
    if dt_k <= 0 || isnan(dt_k), dt_k = dt_med; end
    qd_out(kk,:) = qd_out(kk-1,:) + dt_k * qdd_in(kk-1,:);
end
end

function q_out = integrate_q_from_qd(t_vec, q0, qd_in, q_anchor)
% 与 integrate_qd_from_qdd 同构：q(k)=q(k-1)+dt*qd(k-1)
% 若传入 q_anchor（第4个参数），则使用 q_anchor(k-1) 作为锚点：
%   q(k) = q_anchor(k-1) + dt * qd_in(k-1)
Mloc = size(qd_in, 1);
nloc = size(qd_in, 2);
q_out = zeros(Mloc, nloc);
if Mloc <= 0, return; end
q_out(1,:) = q0;
if Mloc == 1, return; end
dt_med = median(diff(t_vec));
if dt_med <= 0 || isnan(dt_med), dt_med = 0.002; end
for kk = 2:Mloc
    dt_k = t_vec(kk) - t_vec(kk-1);
    if dt_k <= 0 || isnan(dt_k), dt_k = dt_med; end
    if nargin >= 4 && ~isempty(q_anchor)
        q_out(kk,:) = q_anchor(kk-1,:) + dt_k * qd_in(kk-1,:);
    else
        q_out(kk,:) = q_out(kk-1,:) + dt_k * qd_in(kk-1,:);
    end
end
end

function v = summary_pick_mean_row(rmse_st, pref, Lcmp)
    v = nan;
    for ii = 1:numel(pref)
        k = pref{ii};
        if ~any(strcmp(Lcmp, k)), continue; end
        if ~isfield(rmse_st, k), continue; end
        row = rmse_st.(k);
        if isempty(row), continue; end
        v = mean(row(:));
        return;
    end
end

function fn = summary_fd_fieldname(kind, k)
    switch kind
        case 'qdd'
            fn = k;
        case 'qd'
            if strcmp(k, 'phys'), fn = 'qd_phys';
            elseif strcmp(k, 'fd'), fn = 'qd_fd';
            else, fn = ['qd_' k];
            end
        case 'qd_int'
            if strcmp(k, 'phys'), fn = 'qd_int_phys';
            elseif strcmp(k, 'fd'), fn = 'qd_int_fd';
            else, fn = ['qd_int_' k];
            end
        case 'pos'
            if strcmp(k, 'phys'), fn = 'pos_phys';
            elseif strcmp(k, 'fd'), fn = 'pos_fd';
            else, fn = ['pos_' k];
            end
        otherwise
            fn = '';
    end
end

function v = summary_pick_mean_fd(fd_rmse, pref, Lcmp, kind)
    v = nan;
    for ii = 1:numel(pref)
        k = pref{ii};
        if ~any(strcmp(Lcmp, k)), continue; end
        fn = summary_fd_fieldname(kind, k);
        if isempty(fn) || ~isfield(fd_rmse, fn), continue; end
        row = fd_rmse.(fn);
        if isempty(row), continue; end
        v = mean(row(:));
        return;
    end
end

function v = summary_pick_mass_scalar(mass_stats, pref, Lcmp, fld)
    v = nan;
    for ii = 1:numel(pref)
        k = pref{ii};
        if ~any(strcmp(Lcmp, k)), continue; end
        if ~isfield(mass_stats, k), continue; end
        if ~isfield(mass_stats.(k), fld), continue; end
        v = mass_stats.(k).(fld);
        return;
    end
end

function stats = collect_mass_matrix_stats(dataset, idx_list, pi_vec, limb, para_order)
% collect_mass_matrix_stats 统计 full 参数模型下 M(q) 的数值性质
    idx_list = idx_list(:);
    n_pts = numel(idx_list);
    min_eigs = nan(n_pts, 1);
    conds = nan(n_pts, 1);
    n_fail = 0;
    opts_fd = struct('solver', 'Msym', 'regularize_min_eig', 1e-8, 'cond_warn', 1e15);

    for ii = 1:n_pts
        k = idx_list(ii);
        qk = dataset.q(k,:);
        qdk = dataset.qd(k,:);
        tauk = dataset.tau(k,:);
        try
            [qdd_k, d] = forward_dynamics_full(qk, qdk, tauk, pi_vec, limb, para_order, opts_fd);
            if any(~isfinite(qdd_k(:))) || isempty(d) || ~isfield(d, 'min_eig_Msym') || ~isfield(d, 'condMsym')
                n_fail = n_fail + 1;
                continue;
            end
            min_eigs(ii) = d.min_eig_Msym;
            conds(ii) = d.condMsym;
        catch
            n_fail = n_fail + 1;
        end
    end

    valid = isfinite(min_eigs) & isfinite(conds);
    if any(valid)
        stats.min_eig_min = min(min_eigs(valid));
        stats.min_eig_median = median(min_eigs(valid));
        stats.cond_max = max(conds(valid));
        stats.cond_median = median(conds(valid));
    else
        stats.min_eig_min = nan;
        stats.min_eig_median = nan;
        stats.cond_max = nan;
        stats.cond_median = nan;
    end
    stats.n_points = n_pts;
    stats.n_valid = sum(valid);
    stats.n_fail = n_fail;
    stats.fail_rate = n_fail / max(n_pts, 1);
end
