%% run_compare_id_fd_from_selected_csv
% 选择任意 CSV，复用已有辨识参数（step4/step5），做 ID/FD 对比。
% 参数来源：
%   - build/step4_res_min.mat : X_hat, index_base
%   - build/step5_full_params.mat : pi_cad, pi_rec, pi_phys, pi_fd

clc; clear; close all;

app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
if exist('ensure_body_gravity_para_iden_path', 'file')
    ensure_body_gravity_para_iden_path();
end

%% 1) 基本配置
limb = 'left_leg';
para_order = 1;
do_plot = true;
M_fd_max = inf;

build_dir = fullfile(app_root, 'build');
step2_mat = fullfile(build_dir, 'step2_preprocessed.mat');
step3_mat = fullfile(build_dir, 'step3_dataset.mat');
step4_mat = fullfile(build_dir, 'step4_res_min.mat');
step5_mat = fullfile(build_dir, 'step5_full_params.mat');

if ~isfile(step4_mat) || ~isfile(step5_mat)
    error('缺少参数文件：请先确保 %s 和 %s 存在。', step4_mat, step5_mat);
end

%% 2) 选择 CSV 文件（可改成固定路径）
[fn, fp] = uigetfile('*.csv', '选择要对比的 CSV 文件');
if isequal(fn, 0)
    error('未选择 CSV，已取消。');
end
csv_file = fullfile(fp, fn);
fprintf('已选择 CSV: %s\n', csv_file);

%% 3) 读取历史参数
S4 = load(step4_mat);
S5 = load(step5_mat);
if ~isfield(S4, 'X_hat') || ~isfield(S4, 'index_base')
    error('step4_res_min.mat 缺少 X_hat/index_base');
end
X_hat = S4.X_hat(:);
index_base = S4.index_base(:);

pi_cad = [];
pi_rec = [];
pi_phys = [];
pi_fd = [];
if isfield(S5, 'pi_cad') && ~isempty(S5.pi_cad), pi_cad = S5.pi_cad(:); end
if isfield(S5, 'pi_rec') && ~isempty(S5.pi_rec), pi_rec = S5.pi_rec(:); end
if isfield(S5, 'pi_phys') && ~isempty(S5.pi_phys), pi_phys = S5.pi_phys(:); end
if isfield(S5, 'pi_fd') && ~isempty(S5.pi_fd), pi_fd = S5.pi_fd(:); end

if isempty(pi_cad)
    [robot_limb, ~] = get_e1_limb_robot(limb);
    pi_cad = get_limb_theta_from_URDF(robot_limb, para_order);
    pi_cad = pi_cad(:);
end

%% 4) 预处理与连续窗（优先复用历史配置）
prep_opts = struct( ...
    't_start_s', 2.1, ...
    't_end_s', 4.1, ...
    'q_lowpass_fc_Hz', 25, ...
    'q_lowpass_order', 2, ...
    'tau_lowpass_fc_Hz', 25, ...
    'tau_lowpass_order', 2, ...
    'do_compensation', false, ...
    'do_plot', false);
if isfile(step2_mat)
    S2 = load(step2_mat);
    if isfield(S2, 'prep_used') && ~isempty(S2.prep_used)
        prep_opts = S2.prep_used;
    end
end

window_opts = struct('t_start_s', [], 't_end_s', [], 'qd_lowpass_fc_Hz', 0, 'qdd_smooth_half', 0);
if isfile(step3_mat)
    S3 = load(step3_mat);
    if isfield(S3, 'meta') && isfield(S3.meta, 'window_used') && ~isempty(S3.meta.window_used)
        window_opts = S3.meta.window_used;
    end
end

[~, data_after, ~] = run_id_preprocess_pipeline(csv_file, prep_opts);
avg_data = continuous_window_id_data(data_after.t, data_after.q, data_after.qd, data_after.tau_id, window_opts, data_after.qdd);

dataset = struct();
dataset.t = avg_data.t_equiv(:);
dataset.q = avg_data.q_bar;
dataset.qd = avg_data.qd_bar;
dataset.qdd = avg_data.qdd_bar;
dataset.tau = avg_data.tau_bar;

M = size(dataset.q, 1);
[~, n] = get_e1_limb_robot(limb);
M_fd = min(M, M_fd_max);
fprintf('对比数据集: M=%d, FD样本=%d\n', M, M_fd);

%% 5) 逆动力学对比（tau）
tau_meas = dataset.tau;
tau_cad = zeros(M, n);
tau_min = zeros(M, n);
tau_rec = zeros(M, n);
tau_phys = zeros(M, n);
tau_fd = zeros(M, n);

model_cad_id = struct('type','full','limb',limb,'para_order',para_order,'pi_vec',pi_cad);
model_min_id = struct('type','min','limb',limb,'para_order',para_order,'X_hat',X_hat,'index_base',index_base);
if ~isempty(pi_rec), model_rec_id = struct('type','full','limb',limb,'para_order',para_order,'pi_vec',pi_rec); end
if ~isempty(pi_phys), model_phys_id = struct('type','full','limb',limb,'para_order',para_order,'pi_vec',pi_phys); end
if ~isempty(pi_fd), model_fd_id = struct('type','full','limb',limb,'para_order',para_order,'pi_vec',pi_fd); end

for k = 1:M
    tau_cad(k,:) = inverse_dynamics_dispatch(dataset.q(k,:), dataset.qd(k,:), dataset.qdd(k,:), model_cad_id).';
    tau_min(k,:) = inverse_dynamics_dispatch(dataset.q(k,:), dataset.qd(k,:), dataset.qdd(k,:), model_min_id).';
    if ~isempty(pi_rec), tau_rec(k,:) = inverse_dynamics_dispatch(dataset.q(k,:), dataset.qd(k,:), dataset.qdd(k,:), model_rec_id).'; end
    if ~isempty(pi_phys), tau_phys(k,:) = inverse_dynamics_dispatch(dataset.q(k,:), dataset.qd(k,:), dataset.qdd(k,:), model_phys_id).'; end
    if ~isempty(pi_fd), tau_fd(k,:) = inverse_dynamics_dispatch(dataset.q(k,:), dataset.qd(k,:), dataset.qdd(k,:), model_fd_id).'; end
end

fprintf('\nID RMSE (vs tau_meas):\n');
fprintf('  CAD :'); fprintf(' %.3f', sqrt(mean((tau_cad-tau_meas).^2,1))); fprintf('\n');
fprintf('  MIN :'); fprintf(' %.3f', sqrt(mean((tau_min-tau_meas).^2,1))); fprintf('\n');
if ~isempty(pi_rec), fprintf('  REC :'); fprintf(' %.3f', sqrt(mean((tau_rec-tau_meas).^2,1))); fprintf('\n'); end
if ~isempty(pi_phys), fprintf('  PHYS:'); fprintf(' %.3f', sqrt(mean((tau_phys-tau_meas).^2,1))); fprintf('\n'); end
if ~isempty(pi_fd), fprintf('  FD  :'); fprintf(' %.3f', sqrt(mean((tau_fd-tau_meas).^2,1))); fprintf('\n'); end

fprintf('\nID Max|err| (vs tau_meas):\n');
fprintf('  CAD :'); fprintf(' %.3f', max(abs(tau_cad-tau_meas), [], 1)); fprintf('\n');
fprintf('  MIN :'); fprintf(' %.3f', max(abs(tau_min-tau_meas), [], 1)); fprintf('\n');
if ~isempty(pi_rec), fprintf('  REC :'); fprintf(' %.3f', max(abs(tau_rec-tau_meas), [], 1)); fprintf('\n'); end
if ~isempty(pi_phys), fprintf('  PHYS:'); fprintf(' %.3f', max(abs(tau_phys-tau_meas), [], 1)); fprintf('\n'); end
if ~isempty(pi_fd), fprintf('  FD  :'); fprintf(' %.3f', max(abs(tau_fd-tau_meas), [], 1)); fprintf('\n'); end

%% 6) 正动力学对比（qdd, qd）
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
if ~isempty(pi_rec), model_rec_fd = struct('type','full','limb',limb,'para_order',para_order,'pi_vec',pi_rec); end
if ~isempty(pi_phys), model_phys_fd = struct('type','full','limb',limb,'para_order',para_order,'pi_vec',pi_phys); end
if ~isempty(pi_fd), model_fd_fd = struct('type','full','limb',limb,'para_order',para_order,'pi_vec',pi_fd); end

for k = 1:M_fd
    qk = dataset.q(k,:); qdk = dataset.qd(k,:); tauk = dataset.tau(k,:);
    qdd_cad(k,:) = forward_dynamics_dispatch(qk, qdk, tauk, model_cad_fd, struct()).';
    qdd_min(k,:) = forward_dynamics_dispatch(qk, qdk, tauk, model_min_fd, struct()).';
    if ~isempty(pi_rec), qdd_rec(k,:) = forward_dynamics_dispatch(qk, qdk, tauk, model_rec_fd, struct()).'; end
    if ~isempty(pi_phys), qdd_phys(k,:) = forward_dynamics_dispatch(qk, qdk, tauk, model_phys_fd, struct()).'; end
    if ~isempty(pi_fd), qdd_fd(k,:) = forward_dynamics_dispatch(qk, qdk, tauk, model_fd_fd, struct()).'; end
end

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
    if ~isempty(pi_rec), qd_rec(k,:) = qd_rec(k-1,:) + dt_k * qdd_rec(k-1,:); end
    if ~isempty(pi_phys), qd_phys(k,:) = qd_phys(k-1,:) + dt_k * qdd_phys(k-1,:); end
    if ~isempty(pi_fd), qd_fd(k,:) = qd_fd(k-1,:) + dt_k * qdd_fd(k-1,:); end
end

fprintf('\nFD RMSE (vs qdd_ref):\n');
fprintf('  CAD :'); fprintf(' %.3f', sqrt(mean((qdd_cad-qdd_ref).^2,1))); fprintf('\n');
fprintf('  MIN :'); fprintf(' %.3f', sqrt(mean((qdd_min-qdd_ref).^2,1))); fprintf('\n');
if ~isempty(pi_rec), fprintf('  REC :'); fprintf(' %.3f', sqrt(mean((qdd_rec-qdd_ref).^2,1))); fprintf('\n'); end
if ~isempty(pi_phys), fprintf('  PHYS:'); fprintf(' %.3f', sqrt(mean((qdd_phys-qdd_ref).^2,1))); fprintf('\n'); end
if ~isempty(pi_fd), fprintf('  FD  :'); fprintf(' %.3f', sqrt(mean((qdd_fd-qdd_ref).^2,1))); fprintf('\n'); end

fprintf('\nFD 速度RMSE (vs qd_ref):\n');
fprintf('  CAD :'); fprintf(' %.3f', sqrt(mean((qd_cad-qd_ref).^2,1))); fprintf('\n');
fprintf('  MIN :'); fprintf(' %.3f', sqrt(mean((qd_min-qd_ref).^2,1))); fprintf('\n');
if ~isempty(pi_rec), fprintf('  REC :'); fprintf(' %.3f', sqrt(mean((qd_rec-qd_ref).^2,1))); fprintf('\n'); end
if ~isempty(pi_phys), fprintf('  PHYS:'); fprintf(' %.3f', sqrt(mean((qd_phys-qd_ref).^2,1))); fprintf('\n'); end
if ~isempty(pi_fd), fprintf('  FD  :'); fprintf(' %.3f', sqrt(mean((qd_fd-qd_ref).^2,1))); fprintf('\n'); end

if do_plot
    plot_data_tau = [tau_meas, tau_cad, tau_min];
    legend_tau = {'\tau_{meas}','Y\pi_{cad}','Y_{min}X_{hat}'};
    if ~isempty(pi_rec), plot_data_tau = [plot_data_tau, tau_rec]; legend_tau{end+1} = 'Y\pi_{rec}'; end %#ok<AGROW>
    if ~isempty(pi_phys), plot_data_tau = [plot_data_tau, tau_phys]; legend_tau{end+1} = 'Y\pi_{phys}'; end %#ok<AGROW>
    if ~isempty(pi_fd), plot_data_tau = [plot_data_tau, tau_fd]; legend_tau{end+1} = 'Y\pi_{fd}'; end %#ok<AGROW>
    plot_compare_with_error_6dof((1:M)', tau_meas, '\tau_{meas}', plot_data_tau(:, 7:end), legend_tau(2:end), 'torque', '逆动力学多模型对比');

    plot_data_qdd = [qdd_ref, qdd_cad, qdd_min];
    legend_qdd = {'qdd_{ref}','FD(CAD)','FD(\beta)'};
    if ~isempty(pi_rec), plot_data_qdd = [plot_data_qdd, qdd_rec]; legend_qdd{end+1} = 'FD(\pi_{rec})'; end %#ok<AGROW>
    if ~isempty(pi_phys), plot_data_qdd = [plot_data_qdd, qdd_phys]; legend_qdd{end+1} = 'FD(\pi_{phys})'; end %#ok<AGROW>
    if ~isempty(pi_fd), plot_data_qdd = [plot_data_qdd, qdd_fd]; legend_qdd{end+1} = 'FD(\pi_{fd})'; end %#ok<AGROW>
    plot_compare_with_error_6dof(t_fd, qdd_ref, 'qdd_{ref}', plot_data_qdd(:, 7:end), legend_qdd(2:end), 'qdd', '正动力学多模型对比');
end

fprintf('\n完成：已基于选择的 CSV 对历史参数做正逆动力学对比。\n');
