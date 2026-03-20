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
csv_file = fullfile(app_root, 'data', 'excitation', 'PD-M1-v0_multi_joint_20260305-195432_exctra_real.csv');
if ~isfile(csv_file)
    error('CSV 不存在: %s', csv_file);
end

cfg = struct();
cfg.use_preprocess = true;
cfg.prep_opts = struct( ...
    't_start_s', 2.1, ...
    't_end_s', 4.1, ...
    'q_lowpass_fc_Hz', 25, ...
    'q_lowpass_order', 2, ...
    'tau_lowpass_fc_Hz', 25, ...
    'tau_lowpass_order', 2, ...
    'do_compensation', false, ...
    'do_plot', false);
cfg.window_opts = struct('t_start_s', [], 't_end_s', [], 'qd_lowpass_fc_Hz', 0, 'qdd_smooth_half', 0);

% 每一步是否强制重新计算（忽略缓存）
cfg.force_recompute_step2 = false;
cfg.force_recompute_step3 = false;
cfg.force_recompute_step4 = false;
cfg.force_recompute_step5 = true;
cfg.force_recompute_step6 = true;
cfg.force_recompute_step7 = true;

limb = 'left_leg';
para_order = 1;
run_pi_phys = true;
run_pi_fd = true;
M_fd_max = 200;
save_excel = true;
use_fast_fd_id_tuning = true; % true=快速调参档, false=标准档

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
        if isfield(S3, 'csv_file') && isequal(S3.csv_file, csv_file)
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
    % 步骤3 保存最终版
    save(step3_mat, 'dataset', 'meta', 'csv_file');
    fprintf('步骤3已保存到: %s\n', step3_mat);
end

fprintf('数据集: M=%d, t=[%.3f, %.3f]\n', size(dataset.q,1), dataset.t(1), dataset.t(end));

%% 4) 步骤4：最小参数辨识
id_opts = struct();
id_opts.limb = limb;
id_opts.para_order = para_order;
id_opts.identify_min_opts = struct('use_wls', false, 'para_order', para_order);

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
fprintf('最小参数维度 p_min=%d\n', numel(X_hat));

%% 5) 步骤5：CAD/full 参数
load_step5 = false;
if ~cfg.force_recompute_step5 && isfile(step5_mat)
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
    [pi_rec, res_norm_rec] = recover_full_params_from_beta(K, beta_hat, pi_cad, 1e-2);
    fprintf('pi_rec 恢复残差=%.3e\n', res_norm_rec);

    pi_phys = [];
    if run_pi_phys
        opts_phys = struct('algorithm','sqp','max_iter',1000,'MaxFunctionEvaluations',20000,'display','iter');
        [pi_phys, info_phys] = solve_full_params_physical(Y_full, Y_min, beta_hat, pi_cad, 10, opts_phys); %#ok<NASGU>
    end

    pi_fd = [];
    if run_pi_fd
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
            'traj_H', 2, ...
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
if ~load_step6
    tau_meas = dataset.tau;
    tau_cad = zeros(M, n);
    tau_min = zeros(M, n);
    tau_rec = zeros(M, n);
    tau_phys = zeros(M, n);
    tau_fd = zeros(M, n);

    model_cad_id = struct('type','full','limb',limb,'para_order',para_order,'pi_vec',pi_cad);
    model_min_id = struct('type','min','limb',limb,'para_order',para_order,'X_hat',X_hat,'index_base',index_base);
    model_rec_id = struct('type','full','limb',limb,'para_order',para_order,'pi_vec',pi_rec);
    if ~isempty(pi_phys), model_phys_id = struct('type','full','limb',limb,'para_order',para_order,'pi_vec',pi_phys); end
    if ~isempty(pi_fd), model_fd_id = struct('type','full','limb',limb,'para_order',para_order,'pi_vec',pi_fd); end

    for k = 1:M
        tau_cad(k,:) = inverse_dynamics_dispatch(dataset.q(k,:), dataset.qd(k,:), dataset.qdd(k,:), model_cad_id).';
        tau_min(k,:) = inverse_dynamics_dispatch(dataset.q(k,:), dataset.qd(k,:), dataset.qdd(k,:), model_min_id).';
        tau_rec(k,:) = inverse_dynamics_dispatch(dataset.q(k,:), dataset.qd(k,:), dataset.qdd(k,:), model_rec_id).';
        if ~isempty(pi_phys), tau_phys(k,:) = inverse_dynamics_dispatch(dataset.q(k,:), dataset.qd(k,:), dataset.qdd(k,:), model_phys_id).'; end
        if ~isempty(pi_fd), tau_fd(k,:) = inverse_dynamics_dispatch(dataset.q(k,:), dataset.qd(k,:), dataset.qdd(k,:), model_fd_id).'; end
    end

    % 步骤6 保存最终版
    save(step6_mat, 'tau_meas', 'tau_cad', 'tau_min', 'tau_rec', 'tau_phys', 'tau_fd', 'csv_file');
    fprintf('步骤6已保存到: %s\n', step6_mat);
end

fprintf('\nID RMSE (vs tau_meas):\n');
fprintf('  CAD :'); fprintf(' %.3f', sqrt(mean((tau_cad-tau_meas).^2,1))); fprintf('\n');
fprintf('  MIN :'); fprintf(' %.3f', sqrt(mean((tau_min-tau_meas).^2,1))); fprintf('\n');
fprintf('  REC :'); fprintf(' %.3f', sqrt(mean((tau_rec-tau_meas).^2,1))); fprintf('\n');
if ~isempty(pi_phys), fprintf('  PHYS:'); fprintf(' %.3f', sqrt(mean((tau_phys-tau_meas).^2,1))); fprintf('\n'); end
if ~isempty(pi_fd), fprintf('  FD  :'); fprintf(' %.3f', sqrt(mean((tau_fd-tau_meas).^2,1))); fprintf('\n'); end

fprintf('\nID Max|err| (vs tau_meas):\n');
fprintf('  CAD :'); fprintf(' %.3f', max(abs(tau_cad-tau_meas), [], 1)); fprintf('\n');
fprintf('  MIN :'); fprintf(' %.3f', max(abs(tau_min-tau_meas), [], 1)); fprintf('\n');
fprintf('  REC :'); fprintf(' %.3f', max(abs(tau_rec-tau_meas), [], 1)); fprintf('\n');
if ~isempty(pi_phys), fprintf('  PHYS:'); fprintf(' %.3f', max(abs(tau_phys-tau_meas), [], 1)); fprintf('\n'); end
if ~isempty(pi_fd), fprintf('  FD  :'); fprintf(' %.3f', max(abs(tau_fd-tau_meas), [], 1)); fprintf('\n'); end

plot_data = [tau_meas, tau_cad, tau_min, tau_rec];
legend_txt = {'\tau_{meas}','Y\pi_{cad}','Y_{min}X_{hat}','Y\pi_{rec}'};
if ~isempty(pi_phys), plot_data = [plot_data, tau_phys]; legend_txt{end+1} = 'Y\pi_{phys}'; end %#ok<AGROW>
if ~isempty(pi_fd), plot_data = [plot_data, tau_fd]; legend_txt{end+1} = 'Y\pi_{fd}'; end %#ok<AGROW>
plot_compare_with_error_6dof((1:M)', tau_meas, '\tau_{meas}', ...
    plot_data(:, 7:end), legend_txt(2:end), 'torque', '逆动力学多模型');

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
if ~load_step7
    t_fd = dataset.t(1:M_fd);
    qd_ref = dataset.qd(1:M_fd,:);
    qdd_ref = dataset.qdd(1:M_fd,:);
    qdd_cad = zeros(M_fd,n);
    qdd_min = zeros(M_fd,n);
    qdd_rec = zeros(M_fd,n);
    qdd_phys = zeros(M_fd,n);
    qdd_fd = zeros(M_fd,n);

    model_cad_fd = struct('type','urdf','limb',limb,'para_order',para_order);
    model_min_fd = struct('type','min','limb',limb,'para_order',para_order,'X_hat',X_hat,'index_base',index_base);
    model_rec_fd = struct('type','full','limb',limb,'para_order',para_order,'pi_vec',pi_rec);
    if ~isempty(pi_phys), model_phys_fd = struct('type','full','limb',limb,'para_order',para_order,'pi_vec',pi_phys); end
    if ~isempty(pi_fd), model_fd_fd = struct('type','full','limb',limb,'para_order',para_order,'pi_vec',pi_fd); end

    for k = 1:M_fd
        qk = dataset.q(k,:); qdk = dataset.qd(k,:); tauk = dataset.tau(k,:);
        qdd_cad(k,:) = forward_dynamics_dispatch(qk, qdk, tauk, model_cad_fd, struct()).';
        qdd_min(k,:) = forward_dynamics_dispatch(qk, qdk, tauk, model_min_fd, struct()).';
        qdd_rec(k,:) = forward_dynamics_dispatch(qk, qdk, tauk, model_rec_fd, struct()).';
        if ~isempty(pi_phys), qdd_phys(k,:) = forward_dynamics_dispatch(qk, qdk, tauk, model_phys_fd, struct()).'; end
        if ~isempty(pi_fd), qdd_fd(k,:) = forward_dynamics_dispatch(qk, qdk, tauk, model_fd_fd, struct()).'; end
    end

    % 由 qdd 积分得到 qd（用于正动力学速度对比）
    qd_cad = zeros(M_fd, n); qd_cad(1,:) = qd_ref(1,:);
    qd_min = zeros(M_fd, n); qd_min(1,:) = qd_ref(1,:);
    qd_rec = zeros(M_fd, n); qd_rec(1,:) = qd_ref(1,:);
    qd_phys = zeros(M_fd, n); qd_phys(1,:) = qd_ref(1,:);
    qd_fd = zeros(M_fd, n); qd_fd(1,:) = qd_ref(1,:);
    for k = 2:M_fd
        dt_k = t_fd(k) - t_fd(k-1);
        if dt_k <= 0 || isnan(dt_k), dt_k = median(diff(t_fd)); end
        qd_cad(k,:) = qd_cad(k-1,:) + dt_k * qdd_cad(k-1,:);
        qd_min(k,:) = qd_min(k-1,:) + dt_k * qdd_min(k-1,:);
        qd_rec(k,:) = qd_rec(k-1,:) + dt_k * qdd_rec(k-1,:);
        if ~isempty(pi_phys), qd_phys(k,:) = qd_phys(k-1,:) + dt_k * qdd_phys(k-1,:); end
        if ~isempty(pi_fd), qd_fd(k,:) = qd_fd(k-1,:) + dt_k * qdd_fd(k-1,:); end
    end

    % 步骤7 保存最终版
    save(step7_mat, 't_fd', 'qd_ref', 'qdd_ref', 'qd_cad', 'qd_min', 'qd_rec', 'qd_phys', 'qd_fd', ...
        'qdd_cad', 'qdd_min', 'qdd_rec', 'qdd_phys', 'qdd_fd', 'csv_file');
    fprintf('步骤7已保存到: %s\n', step7_mat);
end

fprintf('\nFD RMSE (vs qdd_ref):\n');
fprintf('  CAD :'); fprintf(' %.3f', sqrt(mean((qdd_cad-qdd_ref).^2,1))); fprintf('\n');
fprintf('  MIN :'); fprintf(' %.3f', sqrt(mean((qdd_min-qdd_ref).^2,1))); fprintf('\n');
fprintf('  REC :'); fprintf(' %.3f', sqrt(mean((qdd_rec-qdd_ref).^2,1))); fprintf('\n');
if ~isempty(pi_phys), fprintf('  PHYS:'); fprintf(' %.3f', sqrt(mean((qdd_phys-qdd_ref).^2,1))); fprintf('\n'); end
if ~isempty(pi_fd), fprintf('  FD  :'); fprintf(' %.3f', sqrt(mean((qdd_fd-qdd_ref).^2,1))); fprintf('\n'); end

fprintf('\nFD 速度RMSE (vs qd_ref):\n');
fprintf('  CAD :'); fprintf(' %.3f', sqrt(mean((qd_cad-qd_ref).^2,1))); fprintf('\n');
fprintf('  MIN :'); fprintf(' %.3f', sqrt(mean((qd_min-qd_ref).^2,1))); fprintf('\n');
fprintf('  REC :'); fprintf(' %.3f', sqrt(mean((qd_rec-qd_ref).^2,1))); fprintf('\n');
if ~isempty(pi_phys), fprintf('  PHYS:'); fprintf(' %.3f', sqrt(mean((qd_phys-qd_ref).^2,1))); fprintf('\n'); end
if ~isempty(pi_fd), fprintf('  FD  :'); fprintf(' %.3f', sqrt(mean((qd_fd-qd_ref).^2,1))); fprintf('\n'); end

plot_data_qd = [qd_ref, qd_cad, qd_min, qd_rec];
legend_txt_qd = {'qd_{ref}','int(FD(CAD))','int(FD(\beta))','int(FD(\pi_{rec}))'};
if ~isempty(pi_phys), plot_data_qd = [plot_data_qd, qd_phys]; legend_txt_qd{end+1} = 'int(FD(\pi_{phys}))'; end %#ok<AGROW>
if ~isempty(pi_fd), plot_data_qd = [plot_data_qd, qd_fd]; legend_txt_qd{end+1} = 'int(FD(\pi_{fd}))'; end %#ok<AGROW>
plot_compare_with_error_6dof(t_fd, qd_ref, 'qd_{ref}', ...
    plot_data_qd(:, 7:end), legend_txt_qd(2:end), 'qdd', '正动力学速度多模型');

plot_data = [qdd_ref, qdd_cad, qdd_min, qdd_rec];
legend_txt = {'qdd_{ref}','FD(CAD)','FD(\beta)','FD(\pi_{rec})'};
if ~isempty(pi_phys), plot_data = [plot_data, qdd_phys]; legend_txt{end+1} = 'FD(\pi_{phys})'; end %#ok<AGROW>
if ~isempty(pi_fd), plot_data = [plot_data, qdd_fd]; legend_txt{end+1} = 'FD(\pi_{fd})'; end %#ok<AGROW>
plot_compare_with_error_6dof(t_fd, qdd_ref, 'qdd_{ref}', ...
    plot_data(:, 7:end), legend_txt(2:end), 'qdd', '正动力学多模型');

if save_excel
    id_rmse = struct();
    id_rmse.cad = sqrt(mean((tau_cad - tau_meas).^2,1));
    id_rmse.min = sqrt(mean((tau_min - tau_meas).^2,1));
    id_rmse.rec = sqrt(mean((tau_rec - tau_meas).^2,1));
    if ~isempty(pi_phys), id_rmse.phys = sqrt(mean((tau_phys - tau_meas).^2,1)); end
    if ~isempty(pi_fd), id_rmse.fd = sqrt(mean((tau_fd - tau_meas).^2,1)); end

    fd_rmse = struct();
    fd_rmse.qd_cad = sqrt(mean((qd_cad - qd_ref).^2,1));
    fd_rmse.qd_min = sqrt(mean((qd_min - qd_ref).^2,1));
    fd_rmse.qd_rec = sqrt(mean((qd_rec - qd_ref).^2,1));
    if ~isempty(pi_phys), fd_rmse.qd_phys = sqrt(mean((qd_phys - qd_ref).^2,1)); end
    if ~isempty(pi_fd), fd_rmse.qd_fd = sqrt(mean((qd_fd - qd_ref).^2,1)); end
    fd_rmse.cad = sqrt(mean((qdd_cad - qdd_ref).^2,1));
    fd_rmse.min = sqrt(mean((qdd_min - qdd_ref).^2,1));
    fd_rmse.rec = sqrt(mean((qdd_rec - qdd_ref).^2,1));
    if ~isempty(pi_phys), fd_rmse.phys = sqrt(mean((qdd_phys - qdd_ref).^2,1)); end
    if ~isempty(pi_fd), fd_rmse.fd = sqrt(mean((qdd_fd - qdd_ref).^2,1)); end

    meta_tbl = table(string(csv_file), size(dataset.q,1), M_fd, ...
        'VariableNames', {'csv_file','num_samples','num_fd_samples'});
    out_xlsx = fullfile(build_dir, 'full_dynamics_validation_summary.xlsx');
    export_validation_summary_excel(out_xlsx, id_rmse, fd_rmse, meta_tbl);
    fprintf('Excel汇总已保存到: %s\n', out_xlsx);
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

