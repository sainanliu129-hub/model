%% run_mixed_params_schemeA  方案 A：混合参数方案验证（脚本版，无函数）
%
% 轨迹输入（cfg.dataset_source）：
%   'mat' — 读 build/step3_dataset.mat 的 dataset
%   'csv' — 读 cfg.csv_file（或弹窗选文件），经 run_id_preprocess_pipeline + continuous_window_id_data 得到 dataset
% 辨识/CAD 参数：仍只从 build/step5_full_params.mat 读 pi_cad、pi_phys_best/pi_phys（不改）
%
% 组 1：全 CAD
% 组 2：全辨识（默认用 pi_phys_best；若不存在则回退到 pi_phys）
% 组 3：1~4 辨识，5~6 CAD
% 组 4：1~4 CAD，5~6 辨识
% 组 5：仅关节 5 用 CAD 块（41:50），其余关节用辨识
%
% 只打印关注指标：
%   1) ID RMSE（vs tau_meas）
%   2) FD RMSE（vs qdd_ref）
%   3) H=cfg.H_steps 步串联预测 qd RMSE（vs qd_ref）
%   4) 全程积分 qd/q RMSE（vs qd_ref / vs q_ref_fd）

clc; clear; close all;

% ---------- 基本配置 ----------
cfg = struct();
cfg.limb = 'left_leg';
cfg.para_order = 1;
cfg.block_dim = 10;   % 每关节 10 维参数块
cfg.n_joints = 6;     % 6 关节
cfg.H_steps = 5;      % 你最关注的串联步数
cfg.M_fd_max = inf;   % FD 只取前 M_fd_max 个点；inf=全量
cfg.compute_id = true;
cfg.compute_fd = true;
cfg.verbose = true;
cfg.do_plot = true;   % 是否绘制多组对比图（ID/FD）
cfg.use_single_step_q = true;         % q 使用单步积分（q(k)=q_ref(k-1)+dt*qd_pred(k-1)）
cfg.report_full_integral = false;     % 是否输出“全程积分 qd/q”结果（按当前需求默认关闭）
% 可选：只验证指定组；空={} 表示验证全部可用组
% 允许组名：
%   'all_cad','all_id','mix_14id_56cad','mix_14cad_56id','mix_j5cad_rest_id'
cfg.selected_groups = {'all_cad','all_id'};             
cfg.export_best_urdf = true;         % 是否按最优组导出新 URDF（默认关闭）
cfg.best_metric = 'fd_qd_H';          % 'fd_qd_H' | 'fd_qdd' | 'id'
cfg.force_psd_in_urdf = true;         % 写入 URDF 前惯量投影到 PSD
cfg.source_urdf = '';                 % 空则自动 get_e1_urdf_path()
cfg.out_urdf_name = '';               % 空则自动命名

% CSV 预处理是否做摩擦/转子等补偿（false：仅低通+窗，与 run_id_preprocess_pipeline 的 do_compensation 一致）
% 注意：dataset_source='mat' 时 dataset 已在 step3 里固定；若要完全无补偿轨迹请用 CSV 且此处 false 重算。
cfg.do_friction_compensation = false;

% ---------- 轨迹数据来源（二选一）----------
% 'mat' : 从 build/step3_dataset.mat 读 dataset（与全流程验证一致）
% 'csv' : 从 CSV 经预处理 + continuous_window 构造 dataset（与 run_compare_id_fd_from_selected_csv 口径一致）
cfg.dataset_source = 'csv';
% CSV 模式：填写完整路径，例如 fullfile(app_root,'data','excitation','xxx.csv')
app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
if exist('ensure_body_gravity_para_iden_path', 'file')
    ensure_body_gravity_para_iden_path();
end
cfg.csv_file = fullfile(app_root, 'data', 'excitation', 'single_run', 'PD-M1-v0_multi_joint_20260325-140557_safe_real.csv');
% CSV 模式且 csv_file 为空时：true 则弹出文件选择框
cfg.select_csv_by_dialog = false;
% CSV 模式：是否从 build/step2_preprocessed.mat / step3_dataset.mat 复用 prep / 窗配置
cfg.reuse_step2_prep = false;
cfg.reuse_step3_window = false;
% CSV 模式默认预处理（无摩擦补偿时 do_compensation/load_friction_from_summary 均为 false）
cfg.prep_opts = struct( ...
    't_start_s', [], ...
    't_end_s', [], ...
    'q_lowpass_fc_Hz', 25, ...
    'q_lowpass_order', 2, ...
    'tau_lowpass_fc_Hz', 25, ...
    'tau_lowpass_order', 2, ...
    'do_compensation', false, ...
    'load_friction_from_summary', false, ...
    'row_for_joint', [0 0 0 0 0 0], ...
    'do_plot', true);
cfg.window_opts = struct('t_start_s', [], 't_end_s', [], 'qd_lowpass_fc_Hz', 0, 'qdd_smooth_half', 0);

% ---------- 路径与依赖 ----------
script_dir = fileparts(mfilename('fullpath'));
app_root = fullfile(script_dir, '..');  % Body_GravityPara_Iden/
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
if exist('ensure_body_gravity_para_iden_path', 'file') == 2
    ensure_body_gravity_para_iden_path();
end

build_dir = fullfile(app_root, 'build');
step2_mat = fullfile(build_dir, 'step2_preprocessed.mat');
step3_mat = fullfile(build_dir, 'step3_dataset.mat');
step5_mat = fullfile(build_dir, 'step5_full_params.mat');

if ~isfile(step5_mat)
    error('缺少 step5_full_params.mat: %s', step5_mat);
end

% ---------- 加载轨迹 dataset ----------
dsrc = lower(strtrim(char(string(cfg.dataset_source))));
if isempty(dsrc)
    dsrc = 'mat';
end

if strcmp(dsrc, 'mat')
    if ~isfile(step3_mat)
        error('缺少 step3_dataset.mat: %s（或将 cfg.dataset_source 设为 ''csv'' 并从 CSV 构建）', step3_mat);
    end
    S3 = load(step3_mat);
    if ~isfield(S3, 'dataset')
        error('step3_dataset.mat 缺少 dataset 字段。');
    end
    dataset = S3.dataset;
    if cfg.verbose
        fprintf('[run_mixed_params_schemeA] 轨迹来源: MAT -> %s\n', step3_mat);
    end
elseif strcmp(dsrc, 'csv')
    prep_opts = cfg.prep_opts;
    window_opts = cfg.window_opts;
    if cfg.reuse_step2_prep && isfile(step2_mat)
        S2 = load(step2_mat);
        if isfield(S2, 'prep_used') && ~isempty(S2.prep_used)
            prep_opts = S2.prep_used;
        end
    end
    if cfg.reuse_step3_window && isfile(step3_mat)
        S3w = load(step3_mat);
        if isfield(S3w, 'meta') && isfield(S3w.meta, 'window_used') && ~isempty(S3w.meta.window_used)
            window_opts = S3w.meta.window_used;
        end
    end
    % 摩擦补偿：cfg.do_friction_compensation=false 时强制关闭（覆盖 reuse_step2 带入的 prep）
    if isfield(cfg, 'do_friction_compensation') && ~cfg.do_friction_compensation
        prep_opts.do_compensation = false;
        prep_opts.load_friction_from_summary = false;
    end
    csv_file = cfg.csv_file;
    if isempty(csv_file)
        if cfg.select_csv_by_dialog
            [fn, fp] = uigetfile('*.csv', '选择混合方案验证用的 CSV 轨迹');
            if isequal(fn, 0)
                error('未选择 CSV，已取消。');
            end
            csv_file = fullfile(fp, fn);
        else
            error(['CSV 模式：请在 cfg.csv_file 中填写 CSV 完整路径，' ...
                '或设置 cfg.select_csv_by_dialog = true 以弹出选择框。']);
        end
    end
    if ~isfile(csv_file)
        error('CSV 不存在: %s', csv_file);
    end
    if cfg.verbose
        fprintf('[run_mixed_params_schemeA] 轨迹来源: CSV -> %s\n', csv_file);
    end
    [~, data_after, ~] = run_id_preprocess_pipeline(csv_file, prep_opts);
    avg_data = continuous_window_id_data(data_after.t, data_after.q, data_after.qd, data_after.tau_id, window_opts, data_after.qdd);
    dataset = struct();
    dataset.t = avg_data.t_equiv(:);
    dataset.q = avg_data.q_bar;
    dataset.qd = avg_data.qd_bar;
    dataset.qdd = avg_data.qdd_bar;
    dataset.tau = avg_data.tau_bar;
else
    error('cfg.dataset_source 应为 ''mat'' 或 ''csv''，当前为: %s', cfg.dataset_source);
end

if isfield(cfg, 'do_friction_compensation') && ~cfg.do_friction_compensation && strcmpi(dsrc, 'mat') && cfg.verbose
    fprintf('[run_mixed_params_schemeA] 提示：当前 dataset 来自 MAT（step3），力矩是否含摩擦补偿取决于当初生成 step3 的设置；若要无补偿请改用 CSV 且 cfg.do_friction_compensation=false。\n');
end

S5 = load(step5_mat);

if ~isfield(S5, 'pi_cad') || isempty(S5.pi_cad)
    error('step5_full_params.mat 缺少 pi_cad。');
end
pi_cad = S5.pi_cad(:);

% pi_id_best：优先 pi_phys_best，否则回退 pi_phys
pi_id_best = [];
if isfield(S5, 'pi_phys_best') && ~isempty(S5.pi_phys_best)
    pi_id_best = S5.pi_phys_best(:);
elseif isfield(S5, 'pi_phys') && ~isempty(S5.pi_phys)
    pi_id_best = S5.pi_phys(:);
    if cfg.verbose
        fprintf('[run_mixed_params_schemeA] 未找到 pi_phys_best，回退使用 pi_phys。\n');
    end
else
    error('step5_full_params.mat 未找到可用的辨识参数（pi_phys_best/pi_phys）。');
end

[M, n] = size(dataset.q);
if n ~= cfg.n_joints
    error('dataset 关节维度 n=%d 与 cfg.n_joints=%d 不一致。', n, cfg.n_joints);
end

exp_len = cfg.block_dim * cfg.n_joints;
if numel(pi_cad) ~= exp_len || numel(pi_id_best) ~= exp_len
    error('pi 长度不匹配：pi_cad=%d, pi_id_best=%d, 期望=%d', numel(pi_cad), numel(pi_id_best), exp_len);
end

t = dataset.t(:);
if numel(dataset.qd) ~= M*n || numel(dataset.qdd) ~= M*n || numel(dataset.tau) ~= M*n
    % 维度异常时提前报错（避免后续算出错）
    error('dataset(q/qd/qdd/tau) 维度异常，请检查 step3 的输出口径。');
end

M_fd = min(M, cfg.M_fd_max);
t_fd = t(1:M_fd);
qd_ref = dataset.qd(1:M_fd, :);
qdd_ref = dataset.qdd(1:M_fd, :);
q_ref_fd = dataset.q(1:M_fd, :);
tau_meas = dataset.tau; % ID 用全量 tau_meas（与 run_full_dynamics_validation 一致）

dt_med = median(diff(t_fd));
if ~isfinite(dt_med) || dt_med <= 0
    dt_med = 0.002;
end

% ---------- 生成 4 组 pi ----------
cut_14 = cfg.block_dim * 4;  % 1~4
pi_all_cad = pi_cad;
pi_all_id = pi_id_best;

pi_mix_14id_56cad = pi_cad;
pi_mix_14id_56cad(1:cut_14) = pi_id_best(1:cut_14);

pi_mix_14cad_56id = pi_id_best;
pi_mix_14cad_56id(1:cut_14) = pi_cad(1:cut_14);

% 仅关节 5 使用 CAD 参数块（joint k → (k-1)*block_dim + (1:block_dim)）
idx_j5 = (5 - 1) * cfg.block_dim + (1:cfg.block_dim);
pi_mix_j5cad_rest_id = pi_id_best;
pi_mix_j5cad_rest_id(idx_j5) = pi_cad(idx_j5);

pi_groups = struct();
pi_groups.all_cad = pi_all_cad;
pi_groups.all_id = pi_all_id;
pi_groups.mix_14id_56cad = pi_mix_14id_56cad;
pi_groups.mix_14cad_56id = pi_mix_14cad_56id;
pi_groups.mix_j5cad_rest_id = pi_mix_j5cad_rest_id;

all_group_names = fieldnames(pi_groups);
% 仅保留用户本次要验证的组
if isfield(cfg, 'selected_groups') && ~isempty(cfg.selected_groups)
    req = cellstr(string(cfg.selected_groups(:)));
    group_names = {};
    for ii = 1:numel(req)
        if isfield(pi_groups, req{ii})
            group_names{end+1} = req{ii}; %#ok<AGROW>
        else
            error('cfg.selected_groups 包含未知组名: %s。可选: %s', req{ii}, strjoin(all_group_names, ', '));
        end
    end
else
    group_names = all_group_names;
end
n_grp = numel(group_names);
if n_grp < 1
    error('未选中任何可验证组。请检查 cfg.selected_groups。');
end
if cfg.verbose
    fprintf('[run_mixed_params_schemeA] 本次验证组: %s\n', strjoin(group_names, ', '));
end

% 串联步数（各组相同，表头用）
H_eff = max(1, round(cfg.H_steps));
H_eff = min(H_eff, max(M_fd - 1, 1));

% 汇总矩阵：行=组，列=j1..jn
id_mat = nan(n_grp, n);
fd_qdd_mat = nan(n_grp, n);
fd_qd_H_mat = nan(n_grp, n);
fd_qd_int_mat = nan(n_grp, n);
fd_q_step_mat = nan(n_grp, n);
tau_pred_cell = cell(n_grp, 1);
qdd_pred_cell = cell(n_grp, 1);
qd_chain_cell = cell(n_grp, 1);
q_step_cell = cell(n_grp, 1);

for gi = 1:n_grp
    gname = group_names{gi};
    pi_vec = pi_groups.(gname);

    if cfg.verbose
        fprintf('计算组 %s (%d/%d) ...\n', gname, gi, n_grp);
    end

    % ---------- 构造 full 模型 ----------
    model_full = struct('type', 'full', 'limb', cfg.limb, 'para_order', cfg.para_order, 'pi_vec', pi_vec);
    forward_opts = struct(); % 可在这里扩展为诊断/求解器参数

    % =========================
    % 1) ID RMSE
    % =========================
    if cfg.compute_id
        tau_pred = zeros(M, n);
        for k = 1:M
            tau_pred(k, :) = inverse_dynamics_dispatch(dataset.q(k, :), dataset.qd(k, :), dataset.qdd(k, :), model_full).';
        end
        id_mat(gi, :) = sqrt(mean((tau_pred - tau_meas).^2, 1));
        tau_pred_cell{gi} = tau_pred;
    end

    % =========================
    % 2) FD 相关指标
    % =========================
    if cfg.compute_fd
        qdd_pred = zeros(M_fd, n);
        for k = 1:M_fd
            qdd_pred(k, :) = forward_dynamics_dispatch(dataset.q(k, :), dataset.qd(k, :), dataset.tau(k, :), model_full, forward_opts).';
        end

        fd_qdd_mat(gi, :) = sqrt(mean((qdd_pred - qdd_ref).^2, 1));

        % H 步串联预测 qd
        qd_chain = zeros(M_fd, n);
        qd_chain(1:min(H_eff, M_fd), :) = qd_ref(1:min(H_eff, M_fd), :);
        if M_fd > H_eff
            for k = (H_eff + 1):M_fd
                s = k - H_eff;
                qv = qd_ref(s, :);
                for j = 0:(H_eff - 1)
                    dt_k = t_fd(s + j + 1) - t_fd(s + j);
                    if ~isfinite(dt_k) || dt_k <= 0
                        dt_k = dt_med;
                    end
                    qv = qv + dt_k * qdd_pred(s + j, :);
                end
                qd_chain(k, :) = qv;
            end
        end

        fd_qd_H_mat(gi, :) = sqrt(mean((qd_chain - qd_ref).^2, 1));

        % 全程积分 qd（可选，仅用于兼容旧口径）
        if cfg.report_full_integral
            qd_int = zeros(M_fd, n);
            qd_int(1, :) = qd_ref(1, :);
            for kk = 2:M_fd
                dt_k = t_fd(kk) - t_fd(kk-1);
                if ~isfinite(dt_k) || dt_k <= 0
                    dt_k = dt_med;
                end
                qd_int(kk, :) = qd_int(kk-1, :) + dt_k * qdd_pred(kk-1, :);
            end
            fd_qd_int_mat(gi, :) = sqrt(mean((qd_int - qd_ref).^2, 1));
        end

        % q 单步积分：每步锚定上一时刻实测 q_ref
        q_step = zeros(M_fd, n);
        q_step(1, :) = q_ref_fd(1, :);
        for kk = 2:M_fd
            dt_k = t_fd(kk) - t_fd(kk-1);
            if ~isfinite(dt_k) || dt_k <= 0
                dt_k = dt_med;
            end
            q_step(kk, :) = q_ref_fd(kk-1, :) + dt_k * qd_chain(kk-1, :);
        end
        fd_q_step_mat(gi, :) = sqrt(mean((q_step - q_ref_fd).^2, 1));
        qdd_pred_cell{gi} = qdd_pred;
        qd_chain_cell{gi} = qd_chain;
        q_step_cell{gi} = q_step;
    end
end

% ---------- 表格输出 ----------
joint_vnames = cell(1, n);
for jj = 1:n
    joint_vnames{jj} = sprintf('j%d', jj);
end

Gcat = categorical(group_names(:));
T_id = [table(Gcat, 'VariableNames', {'group'}), array2table(id_mat, 'VariableNames', joint_vnames)];
T_id.mean_joint = mean(id_mat, 2, 'omitnan');

T_fd_qdd = [table(Gcat, 'VariableNames', {'group'}), array2table(fd_qdd_mat, 'VariableNames', joint_vnames)];
T_fd_qdd.mean_joint = mean(fd_qdd_mat, 2, 'omitnan');

T_fd_qd_H = [table(Gcat, 'VariableNames', {'group'}), array2table(fd_qd_H_mat, 'VariableNames', joint_vnames)];
T_fd_qd_H.mean_joint = mean(fd_qd_H_mat, 2, 'omitnan');

T_fd_qd_int = [table(Gcat, 'VariableNames', {'group'}), array2table(fd_qd_int_mat, 'VariableNames', joint_vnames)];
T_fd_qd_int.mean_joint = mean(fd_qd_int_mat, 2, 'omitnan');

T_fd_q_step = [table(Gcat, 'VariableNames', {'group'}), array2table(fd_q_step_mat, 'VariableNames', joint_vnames)];
T_fd_q_step.mean_joint = mean(fd_q_step_mat, 2, 'omitnan');

fprintf('\n');
if cfg.compute_id
    fprintf('=== ID RMSE（vs tau_meas），单位与 tau 一致 ===\n');
    disp(T_id);
end
if cfg.compute_fd
    fprintf('\n=== FD RMSE qdd（vs qdd_ref），rad/s^2 ===\n');
    disp(T_fd_qdd);
    fprintf('\n=== FD RMSE qd（H=%d 步串联 vs qd_ref），rad/s ===\n', H_eff);
    disp(T_fd_qd_H);
    if cfg.report_full_integral
        fprintf('\n=== FD RMSE qd（全程积分 vs qd_ref），rad/s ===\n');
        disp(T_fd_qd_int);
    end
    fprintf('\n=== FD RMSE q（单步积分 vs q_meas），rad ===\n');
    disp(T_fd_q_step);
end

% ---------- 绘图输出 ----------
if cfg.do_plot
    if cfg.compute_id
        plot_data_tau = tau_meas;
        legend_tau = {'\tau_{meas}'};
        for gi = 1:n_grp
            if ~isempty(tau_pred_cell{gi})
                plot_data_tau = [plot_data_tau, tau_pred_cell{gi}]; %#ok<AGROW>
                legend_tau{end+1} = sprintf('Y\\pi(%s)', group_names{gi}); %#ok<AGROW>
            end
        end
        plot_compare_with_error_6dof((1:M)', tau_meas, '\tau_{meas}', ...
            plot_data_tau(:, 7:end), legend_tau(2:end), 'torque', '混合方案A-ID多组对比');
    end

    if cfg.compute_fd
        % qdd 对比
        plot_data_qdd = qdd_ref;
        legend_qdd = {'qdd_{ref}'};
        for gi = 1:n_grp
            if ~isempty(qdd_pred_cell{gi})
                plot_data_qdd = [plot_data_qdd, qdd_pred_cell{gi}]; %#ok<AGROW>
                legend_qdd{end+1} = sprintf('FD(%s)', group_names{gi}); %#ok<AGROW>
            end
        end
        plot_compare_with_error_6dof(t_fd, qdd_ref, 'qdd_{ref}', ...
            plot_data_qdd(:, 7:end), legend_qdd(2:end), 'qdd', '混合方案A-FD加速度多组对比');

        % H 步串联 qd 对比
        plot_data_qd = qd_ref;
        legend_qd = {'qd_{ref}'};
        for gi = 1:n_grp
            if ~isempty(qd_chain_cell{gi})
                plot_data_qd = [plot_data_qd, qd_chain_cell{gi}]; %#ok<AGROW>
                legend_qd{end+1} = sprintf('H=%d(%s)', H_eff, group_names{gi}); %#ok<AGROW>
            end
        end
        plot_compare_with_error_6dof(t_fd, qd_ref, 'qd_{ref}', ...
            plot_data_qd(:, 7:end), legend_qd(2:end), 'qd', sprintf('混合方案A-FD速度多组对比(H=%d)', H_eff));

        % q 单步积分对比
        plot_data_q = q_ref_fd;
        legend_q = {'q_{meas}'};
        for gi = 1:n_grp
            if ~isempty(q_step_cell{gi})
                plot_data_q = [plot_data_q, q_step_cell{gi}]; %#ok<AGROW>
                legend_q{end+1} = sprintf('stepInt(%s)', group_names{gi}); %#ok<AGROW>
            end
        end
        plot_compare_with_error_6dof(t_fd, q_ref_fd, 'q_{meas}', ...
            plot_data_q(:, 7:end), legend_q(2:end), 'q', '混合方案A-FD关节角多组对比(单步积分)');
    end
end

% ---------- 导出“最优组”URDF（仅改动力学参数） ----------
if cfg.export_best_urdf
    switch lower(string(cfg.best_metric))
        case "fd_qd_h"
            score = T_fd_qd_H.mean_joint;
            metric_name = sprintf('FD qd RMSE (H=%d)', H_eff);
        case "fd_qdd"
            score = T_fd_qdd.mean_joint;
            metric_name = 'FD qdd RMSE';
        case "id"
            score = T_id.mean_joint;
            metric_name = 'ID torque RMSE';
        otherwise
            error('cfg.best_metric 不支持: %s（可选 fd_qd_H/fd_qdd/id）', cfg.best_metric);
    end

    [best_score, best_idx] = min(score);
    best_group = group_names{best_idx};
    best_pi = pi_groups.(best_group);
    fprintf('\n[URDF导出] 最优组=%s, 指标=%s, mean_joint=%.6f\n', best_group, metric_name, best_score);

    source_urdf = cfg.source_urdf;
    if isempty(source_urdf)
        source_urdf = get_e1_urdf_path();
    end
    if ~isfile(source_urdf)
        error('源 URDF 不存在: %s', source_urdf);
    end

    out_dir = fileparts(source_urdf);
    if isempty(cfg.out_urdf_name)
        out_name = sprintf('E1_%s_%s_best_%s_updated.urdf', cfg.limb, best_group, lower(char(string(cfg.best_metric))));
    else
        out_name = cfg.out_urdf_name;
    end
    out_urdf = fullfile(out_dir, out_name);

    % limb 对应 body 名称顺序需与 pi 分块顺序一致
    body_names = get_limb_body_names(cfg.limb);
    n_body = numel(body_names);
    if numel(best_pi) ~= 10 * n_body
        error('最优 pi 维度与 limb body 数不匹配：pi=%d, body=%d', numel(best_pi), n_body);
    end

    link_params = repmat(struct('name', '', 'mass', 0, 'com', [0 0 0], ...
        'ixx', 0, 'ixy', 0, 'ixz', 0, 'iyy', 0, 'iyz', 0, 'izz', 0), n_body, 1);
    for ii = 1:n_body
        idx = (ii - 1) * 10 + (1:10);
        p = best_pi(idx);
        [m, com_xyz, Ixx, Iyy, Izz, Iyz, Ixz, Ixy] = pi_block_to_body_inertia(p, cfg.para_order, cfg.force_psd_in_urdf);
        link_params(ii).name = body_names{ii};
        link_params(ii).mass = m;
        link_params(ii).com = com_xyz(:).';
        link_params(ii).ixx = Ixx;
        link_params(ii).ixy = Ixy;
        link_params(ii).ixz = Ixz;
        link_params(ii).iyy = Iyy;
        link_params(ii).iyz = Iyz;
        link_params(ii).izz = Izz;
    end

    % 优先使用 XML 方式，确保“其余参数不变”（仅修改 link/inertial 相关字段）
    write_updated_urdf_by_xml(source_urdf, out_urdf, link_params);
    fprintf('[URDF导出] 已写出: %s\n', out_urdf);
end

fprintf('\n[run_mixed_params_schemeA] 完成。\n');

