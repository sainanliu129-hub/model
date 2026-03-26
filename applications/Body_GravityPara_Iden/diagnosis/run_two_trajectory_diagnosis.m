%% run_two_trajectory_diagnosis
% 两条轨迹 A/B 对比诊断：
% 1) 轨迹特征对比（范围/RMS/过零/低速/高加）
% 2) 可辨识性对比（cond/svd/列范数/参数块能量）
% 3) CAD vs 辨识 的全时域与分段误差（ID torque / FD qdd）
% 4) 轨迹 B 结论标签（激励不足/摩擦敏感/惯性敏感/耦合敏感/泛化失配）

clc; clear; close all;

script_dir = fileparts(mfilename('fullpath'));
app_root = fullfile(script_dir, '..');
addpath(fullfile(app_root, 'principle'));
addpath(fullfile(app_root, 'test'));
addpath(app_root);
if exist('ensure_body_gravity_para_iden_path', 'file') == 2
    ensure_body_gravity_para_iden_path();
end

%% ---------- 配置 ----------
cfg = struct();
cfg.limb = 'left_leg';
cfg.para_order = 1;
cfg.n_joints = 6;
cfg.block_dim = 10;
cfg.max_samples_fd = inf;
cfg.high_acc_percentile = 90;
cfg.low_speed_eps = 0.08;      % rad/s
cfg.zero_cross_eps = 0.02;     % rad/s
cfg.hist_bins_1d = 16;
cfg.hist_bins_2d = 12;
cfg.joint_pairs = [2 3; 3 4; 4 5];
cfg.qdd_from_qd_if_missing = true;

% 与主流程一致的预处理口径（可按需覆盖）
cfg.prep_opts = struct( ...
    't_start_s', [], ...
    't_end_s', [], ...
    'q_lowpass_fc_Hz', 25, ...
    'q_lowpass_order', 2, ...
    'tau_lowpass_fc_Hz', 25, ...
    'tau_lowpass_order', 2, ...
    'do_compensation', false, ...
    'load_friction_from_summary', false, ...
    'do_plot', false);
cfg.window_opts = struct('t_start_s', [], 't_end_s', [], 'qd_lowpass_fc_Hz', 0, 'qdd_smooth_half', 0);

% 轨迹输入（请改成你的两条真实轨迹）
% 建议语义：A=训练轨迹（用于辨识），B=验证轨迹（不参与辨识）
cfg.trajA_name = 'train_A';
cfg.trajA_csv = fullfile(app_root, 'data', 'excitation', 'PD-M1-v0_multi_joint_20260305-195729_real_run.csv');
cfg.trajB_name = 'valid_B';
cfg.trajB_csv = fullfile(app_root, 'data', 'excitation', 'PD-M1-v0_multi_joint_20260305-195432_exctra_real.csv');

% 参数输入：默认取 step5（CAD + 辨识）
cfg.step5_mat = fullfile(app_root, 'build', 'step5_full_params.mat');
cfg.iden_param_field_priority = {'pi_phys_best', 'pi_phys', 'pi_rec'};

% 输出目录
cfg.out_dir = fullfile(app_root, 'build', 'two_traj_diagnosis');
if ~isfolder(cfg.out_dir)
    mkdir(cfg.out_dir);
end

%% ---------- 加载参数 ----------
if ~isfile(cfg.step5_mat)
    error('缺少参数文件: %s', cfg.step5_mat);
end
S5 = load(cfg.step5_mat);
if ~isfield(S5, 'pi_cad') || isempty(S5.pi_cad)
    error('step5 缺少 pi_cad。');
end
pi_cad = S5.pi_cad(:);
pi_iden = pick_iden_param(S5, cfg.iden_param_field_priority);
if isempty(pi_iden)
    error('step5 中未找到辨识参数（候选: %s）', strjoin(cfg.iden_param_field_priority, ', '));
end
exp_len = cfg.n_joints * cfg.block_dim;
if numel(pi_cad) ~= exp_len || numel(pi_iden) ~= exp_len
    error('参数维度不匹配，期望=%d, 实际 CAD=%d, IDEN=%d', exp_len, numel(pi_cad), numel(pi_iden));
end

%% ---------- 构造两条轨迹 dataset ----------
dsA = build_dataset_from_csv(cfg.trajA_csv, cfg.prep_opts, cfg.window_opts, cfg.qdd_from_qd_if_missing);
dsB = build_dataset_from_csv(cfg.trajB_csv, cfg.prep_opts, cfg.window_opts, cfg.qdd_from_qd_if_missing);
assert(size(dsA.q, 2) == cfg.n_joints && size(dsB.q, 2) == cfg.n_joints, '关节维度不匹配。');

%% ---------- 轨迹特征 ----------
featA = calc_trajectory_features(dsA, cfg);
featB = calc_trajectory_features(dsB, cfg);
T_featA = struct2table(featA.joint);
T_featA.traj = repmat(string(cfg.trajA_name), height(T_featA), 1);
T_featB = struct2table(featB.joint);
T_featB.traj = repmat(string(cfg.trajB_name), height(T_featB), 1);
T_feature = [T_featA; T_featB];

%% ---------- 可辨识性诊断（Y / Ymin） ----------
diagA = calc_regressor_diag(dsA, cfg);
diagB = calc_regressor_diag(dsB, cfg);
T_ident = table( ...
    string({cfg.trajA_name; cfg.trajB_name}), ...
    [diagA.cond_Y; diagB.cond_Y], ...
    [diagA.cond_Ymin; diagB.cond_Ymin], ...
    [diagA.rank_Y; diagB.rank_Y], ...
    [diagA.effective_dim_95; diagB.effective_dim_95], ...
    'VariableNames', {'traj', 'cond_Y', 'cond_Ymin', 'rank_Y', 'effective_dim95_X'});

T_colnorm = table((1:numel(diagA.col_norm)).', diagA.col_norm(:), diagB.col_norm(:), ...
    'VariableNames', {'col_idx', 'colnorm_A', 'colnorm_B'});

blk = (1:cfg.n_joints).';
T_block_energy = table(blk, diagA.block_energy(:), diagB.block_energy(:), ...
    'VariableNames', {'joint_block', 'energy_A', 'energy_B'});

T_svd = table((1:numel(diagA.sv_Ymin)).', diagA.sv_Ymin(:), diagB.sv_Ymin(:), ...
    'VariableNames', {'k', 'sv_A', 'sv_B'});

%% ---------- 模型误差（全时域 + 分段） ----------
model_cad = struct('type', 'full', 'limb', cfg.limb, 'para_order', cfg.para_order, 'pi_vec', pi_cad);
model_idn = struct('type', 'full', 'limb', cfg.limb, 'para_order', cfg.para_order, 'pi_vec', pi_iden);

resA = eval_model_on_dataset(dsA, model_cad, model_idn, cfg);
resB = eval_model_on_dataset(dsB, model_cad, model_idn, cfg);

T_overall = build_overall_error_table(cfg, resA, resB);
T_seg = build_segment_error_table(cfg, resA, resB);
T_gen = build_generalization_table(cfg, resA, resB);

%% ---------- 标签判定 ----------
label_info = classify_trajB_reason(cfg, diagA, diagB, featA, featB, resA, resB);

%% ---------- 保存输出 ----------
out_mat = fullfile(cfg.out_dir, 'two_traj_diagnosis_results.mat');
out_xlsx = fullfile(cfg.out_dir, 'two_traj_diagnosis_summary.xlsx');
save(out_mat, 'cfg', 'featA', 'featB', 'diagA', 'diagB', 'resA', 'resB', ...
    'T_feature', 'T_ident', 'T_colnorm', 'T_block_energy', 'T_svd', 'T_overall', 'T_seg', 'T_gen', 'label_info');

writetable(T_feature, out_xlsx, 'Sheet', 'traj_features');
writetable(T_ident, out_xlsx, 'Sheet', 'identifiability');
writetable(T_colnorm, out_xlsx, 'Sheet', 'col_norm');
writetable(T_block_energy, out_xlsx, 'Sheet', 'block_energy');
writetable(T_svd, out_xlsx, 'Sheet', 'svd_Ymin');
writetable(T_overall, out_xlsx, 'Sheet', 'error_overall');
writetable(T_seg, out_xlsx, 'Sheet', 'error_segmented');
writetable(T_gen, out_xlsx, 'Sheet', 'generalization');

T_label = struct2table(label_info);
writetable(T_label, out_xlsx, 'Sheet', 'trajB_label');

fprintf('\n=== 两轨迹对比诊断完成 ===\n');
fprintf('训练轨迹 A: %s\n', cfg.trajA_csv);
fprintf('验证轨迹 B: %s\n', cfg.trajB_csv);
fprintf('输出 MAT:  %s\n', out_mat);
fprintf('输出 Excel:%s\n', out_xlsx);
fprintf('轨迹 B 标签: %s\n', label_info.primary_label);
fprintf('解释: %s\n', label_info.reason_text);
fprintf('泛化差值(验证-训练, ID): %.6f\n', label_info.generalization_gap_ID_B_minus_A);
fprintf('泛化差值(验证-训练, FDqdd): %.6f\n', label_info.generalization_gap_FD_B_minus_A);

%% ======================= local functions =======================
function pi_iden = pick_iden_param(S5, fields)
pi_iden = [];
for i = 1:numel(fields)
    fn = fields{i};
    if isfield(S5, fn) && ~isempty(S5.(fn))
        pi_iden = S5.(fn)(:);
        return;
    end
end
end

function ds = build_dataset_from_csv(csv_file, prep_opts, window_opts, qdd_from_qd_if_missing)
if ~isfile(csv_file)
    error('轨迹文件不存在: %s', csv_file);
end
[~, data_after, ~] = run_id_preprocess_pipeline(csv_file, prep_opts);
avg_data = continuous_window_id_data(data_after.t, data_after.q, data_after.qd, data_after.tau_id, window_opts, data_after.qdd);
ds = struct();
ds.t = avg_data.t_equiv(:);
ds.q = avg_data.q_bar;
ds.qd = avg_data.qd_bar;
ds.qdd = avg_data.qdd_bar;
ds.tau = avg_data.tau_bar;
if isempty(ds.qdd) && qdd_from_qd_if_missing
    ds.qdd = derive_qdd_from_qd(ds.t, ds.qd);
end
if isempty(ds.qdd)
    error('qdd 为空，且未启用 qdd_from_qd_if_missing。');
end
end

function qdd = derive_qdd_from_qd(t, qd)
M = size(qd, 1);
n = size(qd, 2);
qdd = zeros(M, n);
if M <= 1
    return;
end
dt_med = median(diff(t));
if ~isfinite(dt_med) || dt_med <= 0
    dt_med = 0.002;
end
for j = 1:n
    for k = 1:M
        if k == 1
            dt = t(2) - t(1); if ~isfinite(dt) || dt <= 0, dt = dt_med; end
            qdd(k,j) = (qd(2,j) - qd(1,j)) / dt;
        elseif k == M
            dt = t(M) - t(M-1); if ~isfinite(dt) || dt <= 0, dt = dt_med; end
            qdd(k,j) = (qd(M,j) - qd(M-1,j)) / dt;
        else
            dt = t(k+1) - t(k-1); if ~isfinite(dt) || dt <= 0, dt = 2*dt_med; end
            qdd(k,j) = (qd(k+1,j) - qd(k-1,j)) / dt;
        end
    end
end
end

function feat = calc_trajectory_features(ds, cfg)
q = ds.q; qd = ds.qd; qdd = ds.qdd;
n = size(q, 2);

J = repmat(struct( ...
    'joint', 0, ...
    'range_q', 0, 'range_qd', 0, 'range_qdd', 0, ...
    'rms_q', 0, 'rms_qd', 0, 'rms_qdd', 0, ...
    'zero_cross_qd', 0, 'sign_change_qdd', 0, ...
    'low_speed_ratio', 0, 'high_acc_ratio', 0, ...
    'hist_coverage_q', 0), n, 1);

for j = 1:n
    J(j).joint = j;
    J(j).range_q = range(q(:,j));
    J(j).range_qd = range(qd(:,j));
    J(j).range_qdd = range(qdd(:,j));
    J(j).rms_q = rms(q(:,j));
    J(j).rms_qd = rms(qd(:,j));
    J(j).rms_qdd = rms(qdd(:,j));
    J(j).zero_cross_qd = count_sign_change(qd(:,j), cfg.zero_cross_eps);
    J(j).sign_change_qdd = count_sign_change(qdd(:,j), 0);
    J(j).low_speed_ratio = mean(abs(qd(:,j)) < cfg.low_speed_eps);
    thr_a = prctile(abs(qdd(:,j)), cfg.high_acc_percentile);
    J(j).high_acc_ratio = mean(abs(qdd(:,j)) > thr_a);
    J(j).hist_coverage_q = calc_1d_coverage(q(:,j), cfg.hist_bins_1d);
end

pair_cov = nan(size(cfg.joint_pairs, 1), 1);
for p = 1:size(cfg.joint_pairs, 1)
    a = cfg.joint_pairs(p, 1);
    b = cfg.joint_pairs(p, 2);
    if a <= n && b <= n
        pair_cov(p) = calc_2d_coverage(q(:,a), q(:,b), cfg.hist_bins_2d);
    end
end

X = [zscore(q), zscore(qd), zscore(qdd)];
X(~isfinite(X)) = 0;
s = svd(X, 'econ');
e = cumsum(s.^2) / max(sum(s.^2), eps);
eff95 = find(e >= 0.95, 1, 'first');
if isempty(eff95), eff95 = numel(s); end

feat = struct();
feat.joint = J;
feat.corr_q = corrcoef(q);
feat.corr_qd = corrcoef(qd);
feat.corr_qdd = corrcoef(qdd);
feat.effective_dim_95 = eff95;
feat.pair_coverage = pair_cov;
end

function d = calc_regressor_diag(ds, cfg)
Y = ReMatrix_E1_limb_URDF(cfg.limb, ds.q, ds.qd, ds.qdd, 1, cfg.para_order);
d = struct();
d.rank_Y = rank(Y);
d.cond_Y = safe_cond(Y);

[Yn, col_norm] = normalize_columns(Y);
% 使用 QR pivoting 提取独立列；若失败则回退 rref，确保 idx 始终可索引
try
    [~, ~, piv] = qr(Yn, 'vector');
    r = rank(Yn);
    if r < 1
        idx = 1:size(Yn, 2);
    else
        idx = sort(piv(1:r));
    end
catch
    [~, idx] = rref(Yn);
end
idx = idx(:).';
idx = idx(isfinite(idx) & idx >= 1 & idx <= size(Yn, 2));
idx = unique(round(idx), 'stable');
if isempty(idx)
    idx = 1:size(Yn, 2);
end
Ymin = Yn(:, idx);
d.cond_Ymin = safe_cond(Ymin);
d.col_norm = col_norm(:);
d.index_base = idx(:);
d.sv_Ymin = svd(Ymin, 'econ');

n = cfg.n_joints;
b = cfg.block_dim;
be = zeros(n,1);
for j = 1:n
    id = (j-1)*b + (1:b);
    id = id(id <= size(Y,2));
    be(j) = norm(Y(:, id), 'fro');
end
d.block_energy = be;

X = [zscore(ds.q), zscore(ds.qd), zscore(ds.qdd)];
X(~isfinite(X)) = 0;
sX = svd(X, 'econ');
e = cumsum(sX.^2) / max(sum(sX.^2), eps);
k95 = find(e >= 0.95, 1, 'first');
if isempty(k95), k95 = numel(sX); end
d.effective_dim_95 = k95;
end

function c = safe_cond(A)
if isempty(A)
    c = inf;
    return;
end
c = cond(A);
if ~isfinite(c) || c <= 0
    c = inf;
end
end

function [Yn, col_norm] = normalize_columns(Y)
col_norm = sqrt(sum(Y.^2, 1));
col_norm(col_norm < 1e-12) = 1;
Yn = Y ./ col_norm;
end

function res = eval_model_on_dataset(ds, model_cad, model_idn, cfg)
M = size(ds.q, 1);
n = size(ds.q, 2);
M_fd = min(M, cfg.max_samples_fd);

tau_cad = zeros(M, n);
tau_idn = zeros(M, n);
for k = 1:M
    tau_cad(k, :) = inverse_dynamics_dispatch(ds.q(k,:), ds.qd(k,:), ds.qdd(k,:), model_cad).';
    tau_idn(k, :) = inverse_dynamics_dispatch(ds.q(k,:), ds.qd(k,:), ds.qdd(k,:), model_idn).';
end

qdd_cad = zeros(M_fd, n);
qdd_idn = zeros(M_fd, n);
for k = 1:M_fd
    qdd_cad(k, :) = forward_dynamics_dispatch(ds.q(k,:), ds.qd(k,:), ds.tau(k,:), model_cad, struct()).';
    qdd_idn(k, :) = forward_dynamics_dispatch(ds.q(k,:), ds.qd(k,:), ds.tau(k,:), model_idn, struct()).';
end

seg = build_segments(ds, cfg, M_fd);
res = struct();
res.t = ds.t;
res.tau_meas = ds.tau;
res.qdd_ref = ds.qdd(1:M_fd, :);
res.tau_cad = tau_cad;
res.tau_idn = tau_idn;
res.qdd_cad = qdd_cad;
res.qdd_idn = qdd_idn;
res.segment = seg;
res.err = calc_rmse_pack(res, seg);
end

function seg = build_segments(ds, cfg, M_fd)
qd = ds.qd(1:M_fd, :);
qdd = ds.qdd(1:M_fd, :);
M = size(qd, 1);
n = size(qd, 2);
low = false(M, n);
high = false(M, n);
zero = false(M, n);
for j = 1:n
    low(:, j) = abs(qd(:, j)) < cfg.low_speed_eps;
    thr = prctile(abs(qdd(:, j)), cfg.high_acc_percentile);
    high(:, j) = abs(qdd(:, j)) > thr;
    zc = false(M, 1);
    if M > 1
        zc(2:end) = (qd(1:end-1, j) .* qd(2:end, j)) < 0;
    end
    zero(:, j) = zc | (abs(qd(:, j)) < cfg.zero_cross_eps);
end
seg = struct();
seg.all = true(M, n);
seg.low_speed = low;
seg.high_acc = high;
seg.zero_cross = zero;
end

function E = calc_rmse_pack(res, seg)
E = struct();
E.id_all_cad = rmse_masked(res.tau_cad, res.tau_meas, seg.all);
E.id_all_idn = rmse_masked(res.tau_idn, res.tau_meas, seg.all);
E.fd_all_cad = rmse_masked(res.qdd_cad, res.qdd_ref, seg.all);
E.fd_all_idn = rmse_masked(res.qdd_idn, res.qdd_ref, seg.all);

E.id_low_cad = rmse_masked(res.tau_cad(1:size(seg.low_speed,1),:), res.tau_meas(1:size(seg.low_speed,1),:), seg.low_speed);
E.id_low_idn = rmse_masked(res.tau_idn(1:size(seg.low_speed,1),:), res.tau_meas(1:size(seg.low_speed,1),:), seg.low_speed);
E.fd_low_cad = rmse_masked(res.qdd_cad, res.qdd_ref, seg.low_speed);
E.fd_low_idn = rmse_masked(res.qdd_idn, res.qdd_ref, seg.low_speed);

E.id_high_cad = rmse_masked(res.tau_cad(1:size(seg.high_acc,1),:), res.tau_meas(1:size(seg.high_acc,1),:), seg.high_acc);
E.id_high_idn = rmse_masked(res.tau_idn(1:size(seg.high_acc,1),:), res.tau_meas(1:size(seg.high_acc,1),:), seg.high_acc);
E.fd_high_cad = rmse_masked(res.qdd_cad, res.qdd_ref, seg.high_acc);
E.fd_high_idn = rmse_masked(res.qdd_idn, res.qdd_ref, seg.high_acc);

E.id_zero_cad = rmse_masked(res.tau_cad(1:size(seg.zero_cross,1),:), res.tau_meas(1:size(seg.zero_cross,1),:), seg.zero_cross);
E.id_zero_idn = rmse_masked(res.tau_idn(1:size(seg.zero_cross,1),:), res.tau_meas(1:size(seg.zero_cross,1),:), seg.zero_cross);
E.fd_zero_cad = rmse_masked(res.qdd_cad, res.qdd_ref, seg.zero_cross);
E.fd_zero_idn = rmse_masked(res.qdd_idn, res.qdd_ref, seg.zero_cross);
end

function v = rmse_masked(pred, ref, mask)
n = size(pred, 2);
v = nan(1, n);
for j = 1:n
    idx = mask(:, j);
    if any(idx)
        d = pred(idx, j) - ref(idx, j);
        v(j) = sqrt(mean(d.^2));
    end
end
end

function T = build_overall_error_table(cfg, resA, resB)
joint = (1:cfg.n_joints).';
TA = table(joint, ...
    resA.err.id_all_cad(:), resA.err.id_all_idn(:), ...
    resA.err.fd_all_cad(:), resA.err.fd_all_idn(:), ...
    'VariableNames', {'joint', 'ID_CAD', 'ID_IDEN', 'FDqdd_CAD', 'FDqdd_IDEN'});
TB = table(joint, ...
    resB.err.id_all_cad(:), resB.err.id_all_idn(:), ...
    resB.err.fd_all_cad(:), resB.err.fd_all_idn(:), ...
    'VariableNames', {'joint', 'ID_CAD', 'ID_IDEN', 'FDqdd_CAD', 'FDqdd_IDEN'});
TA.traj = repmat("A", height(TA), 1);
TB.traj = repmat("B", height(TB), 1);
T = [TA; TB];
T.delta_ID = T.ID_IDEN - T.ID_CAD;
T.delta_FDqdd = T.FDqdd_IDEN - T.FDqdd_CAD;
end

function T = build_generalization_table(cfg, resA, resB)
joint = (1:cfg.n_joints).';
delta_train = (resA.err.id_all_idn - resA.err.id_all_cad).';
delta_valid = (resB.err.id_all_idn - resB.err.id_all_cad).';
gap_id = delta_valid - delta_train;

delta_train_fd = (resA.err.fd_all_idn - resA.err.fd_all_cad).';
delta_valid_fd = (resB.err.fd_all_idn - resB.err.fd_all_cad).';
gap_fd = delta_valid_fd - delta_train_fd;

T = table(joint, delta_train, delta_valid, gap_id, delta_train_fd, delta_valid_fd, gap_fd, ...
    'VariableNames', {'joint', 'delta_ID_trainA', 'delta_ID_validB', 'gap_ID_B_minus_A', ...
    'delta_FDqdd_trainA', 'delta_FDqdd_validB', 'gap_FDqdd_B_minus_A'});
end

function T = build_segment_error_table(cfg, resA, resB)
seg_names = {'low', 'high', 'zero'};
rows = {};
for tt = 1:2
    if tt == 1
        tr = 'A'; r = resA;
    else
        tr = 'B'; r = resB;
    end
    for si = 1:numel(seg_names)
        sn = seg_names{si};
        switch sn
            case 'low'
                id_cad = r.err.id_low_cad; id_idn = r.err.id_low_idn;
                fd_cad = r.err.fd_low_cad; fd_idn = r.err.fd_low_idn;
            case 'high'
                id_cad = r.err.id_high_cad; id_idn = r.err.id_high_idn;
                fd_cad = r.err.fd_high_cad; fd_idn = r.err.fd_high_idn;
            case 'zero'
                id_cad = r.err.id_zero_cad; id_idn = r.err.id_zero_idn;
                fd_cad = r.err.fd_zero_cad; fd_idn = r.err.fd_zero_idn;
        end
        for j = 1:cfg.n_joints
            rows(end+1, :) = {string(tr), string(sn), j, id_cad(j), id_idn(j), fd_cad(j), fd_idn(j)}; %#ok<AGROW>
        end
    end
end
T = cell2table(rows, 'VariableNames', {'traj', 'segment', 'joint', 'ID_CAD', 'ID_IDEN', 'FDqdd_CAD', 'FDqdd_IDEN'});
T.delta_ID = T.ID_IDEN - T.ID_CAD;
T.delta_FDqdd = T.FDqdd_IDEN - T.FDqdd_CAD;
end

function label = classify_trajB_reason(cfg, dA, dB, fA, fB, rA, rB)
score = struct('excitation_insufficient', 0, 'friction_sensitive', 0, ...
    'inertia_sensitive', 0, 'coupling_sensitive', 0, 'generalization_mismatch', 0);
notes = {};

% 1) 激励/可辨识性
if dB.cond_Ymin > 1.2 * dA.cond_Ymin
    score.excitation_insufficient = score.excitation_insufficient + 2;
    notes{end+1} = 'B 的 cond(Ymin) 明显更差'; %#ok<AGROW>
end
if median(dB.col_norm) < 0.9 * median(dA.col_norm)
    score.excitation_insufficient = score.excitation_insufficient + 1;
    notes{end+1} = 'B 的回归列能量整体偏低'; %#ok<AGROW>
end

% 2) 摩擦敏感（低速/过零段）
d_id_low = mean(rB.err.id_low_idn - rB.err.id_low_cad, 'omitnan');
d_id_zero = mean(rB.err.id_zero_idn - rB.err.id_zero_cad, 'omitnan');
d_fd_low = mean(rB.err.fd_low_idn - rB.err.fd_low_cad, 'omitnan');
d_fd_zero = mean(rB.err.fd_zero_idn - rB.err.fd_zero_cad, 'omitnan');
if (d_id_low > 0 && d_id_zero > 0) || (d_fd_low > 0 && d_fd_zero > 0)
    score.friction_sensitive = score.friction_sensitive + 2;
    notes{end+1} = 'B 在低速/过零段辨识相对 CAD 更差'; %#ok<AGROW>
end
if mean([fB.joint.low_speed_ratio]) > mean([fA.joint.low_speed_ratio]) * 1.1
    score.friction_sensitive = score.friction_sensitive + 1;
    notes{end+1} = 'B 低速占比更高'; %#ok<AGROW>
end

% 3) 惯性敏感（高加段）
d_id_high = mean(rB.err.id_high_idn - rB.err.id_high_cad, 'omitnan');
d_fd_high = mean(rB.err.fd_high_idn - rB.err.fd_high_cad, 'omitnan');
if d_id_high > 0 || d_fd_high > 0
    score.inertia_sensitive = score.inertia_sensitive + 2;
    notes{end+1} = 'B 在高加速度段辨识相对 CAD 更差'; %#ok<AGROW>
end

% 4) 耦合敏感
if fB.effective_dim_95 < fA.effective_dim_95
    score.coupling_sensitive = score.coupling_sensitive + 1;
    notes{end+1} = 'B 有效激励维数更低（更共线）'; %#ok<AGROW>
end
if mean(fB.pair_coverage, 'omitnan') < mean(fA.pair_coverage, 'omitnan')
    score.coupling_sensitive = score.coupling_sensitive + 1;
    notes{end+1} = 'B 关键关节组合覆盖更低'; %#ok<AGROW>
end

% 5) 泛化失配
d_id_all_B = mean(rB.err.id_all_idn - rB.err.id_all_cad, 'omitnan');
d_fd_all_B = mean(rB.err.fd_all_idn - rB.err.fd_all_cad, 'omitnan');
d_id_all_A = mean(rA.err.id_all_idn - rA.err.id_all_cad, 'omitnan');
d_fd_all_A = mean(rA.err.fd_all_idn - rA.err.fd_all_cad, 'omitnan');
if (d_id_all_B > d_id_all_A) && (d_fd_all_B > d_fd_all_A)
    score.generalization_mismatch = score.generalization_mismatch + 2;
    notes{end+1} = 'B 相对 A 呈现全面退化趋势'; %#ok<AGROW>
end

labels = {'激励不足型', '摩擦敏感型', '惯性敏感型', '耦合敏感型', '泛化失配型'};
vals = [score.excitation_insufficient, score.friction_sensitive, score.inertia_sensitive, ...
    score.coupling_sensitive, score.generalization_mismatch];
[mx, idx] = max(vals);
if mx <= 0
    primary = '未判定（证据不足）';
else
    primary = labels{idx};
end

label = struct();
label.primary_label = string(primary);
label.score_excitation = score.excitation_insufficient;
label.score_friction = score.friction_sensitive;
label.score_inertia = score.inertia_sensitive;
label.score_coupling = score.coupling_sensitive;
label.score_generalization = score.generalization_mismatch;
label.reason_text = string(strjoin(notes, '; '));
label.delta_ID_all_B = d_id_all_B;
label.delta_FD_all_B = d_fd_all_B;
label.delta_ID_all_A = d_id_all_A;
label.delta_FD_all_A = d_fd_all_A;
label.generalization_gap_ID_B_minus_A = d_id_all_B - d_id_all_A;
label.generalization_gap_FD_B_minus_A = d_fd_all_B - d_fd_all_A;
end

function nzc = count_sign_change(x, epsv)
x = x(:);
s = zeros(size(x));
s(x > epsv) = 1;
s(x < -epsv) = -1;
nzc = sum((s(1:end-1) .* s(2:end)) < 0);
end

function c = calc_1d_coverage(x, nbin)
if isempty(x)
    c = nan;
    return;
end
xmin = min(x); xmax = max(x);
if xmax <= xmin
    c = 1;
    return;
end
edges = linspace(xmin, xmax, nbin + 1);
h = histcounts(x, edges);
c = nnz(h > 0) / numel(h);
end

function c = calc_2d_coverage(x, y, nbin)
if isempty(x) || isempty(y)
    c = nan;
    return;
end
xe = linspace(min(x), max(x), nbin + 1);
ye = linspace(min(y), max(y), nbin + 1);
if numel(unique(xe)) < 2 || numel(unique(ye)) < 2
    c = 1;
    return;
end
h = histcounts2(x, y, xe, ye);
c = nnz(h > 0) / numel(h);
end
