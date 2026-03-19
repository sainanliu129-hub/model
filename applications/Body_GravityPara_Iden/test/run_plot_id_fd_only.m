function R = run_plot_id_fd_only(cfg)
% run_plot_id_fd_only  统一 ID/FD 对比与统计（薄封装）

if nargin < 1, cfg = struct(); end

app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
if exist('ensure_body_gravity_para_iden_path', 'file')
    ensure_body_gravity_para_iden_path();
end

cfg = set_default(cfg, 'models', {'pi_fd','cad','beta'});
cfg = set_default(cfg, 'limb', 'left_leg');
cfg = set_default(cfg, 'para_order', 1);
cfg = set_default(cfg, 'N_plot', inf);
cfg = set_default(cfg, 'N_fd', 500);
cfg = set_default(cfg, 'do_plot', true);
cfg = set_default(cfg, 'save_excel', false);
cfg = set_default(cfg, 'input_mat', fullfile(app_root, 'build', 'id_param_sets.mat'));

S = load(cfg.input_mat);
avg_data = S.avg_data;
X_hat = S.X_hat(:);
index_base = S.index_base(:);

[robot_limb, n] = get_e1_limb_robot(cfg.limb);
pi_cad = get_limb_theta_from_URDF(robot_limb, cfg.para_order);
pi_fd = pick_field(S, 'pi_fd', []);
pi_rec = pick_field(S, 'pi_rec', []);
pi_phys = pick_field(S, 'pi_phys', []);

M = size(avg_data.q_bar,1);
N_plot = min(M, cfg.N_plot);
N_fd = min(M, cfg.N_fd);
t = avg_data.t_equiv(:);
q = avg_data.q_bar; qd = avg_data.qd_bar; qdd = avg_data.qdd_bar; tau = avg_data.tau_bar;

% ID
id_models = {};
id_preds = [];
if any(strcmp(cfg.models,'cad')), id_models{end+1} = struct('name','Y\pi_{cad}','m',struct('type','full','limb',cfg.limb,'para_order',cfg.para_order,'pi_vec',pi_cad)); end
if any(strcmp(cfg.models,'beta')), id_models{end+1} = struct('name','Y_{min}X_{hat}','m',struct('type','min','limb',cfg.limb,'para_order',cfg.para_order,'X_hat',X_hat,'index_base',index_base)); end
if any(strcmp(cfg.models,'pi_fd')) && ~isempty(pi_fd), id_models{end+1} = struct('name','Y\pi_{fd}','m',struct('type','full','limb',cfg.limb,'para_order',cfg.para_order,'pi_vec',pi_fd)); end
if any(strcmp(cfg.models,'pi_rec')) && ~isempty(pi_rec), id_models{end+1} = struct('name','Y\pi_{rec}','m',struct('type','full','limb',cfg.limb,'para_order',cfg.para_order,'pi_vec',pi_rec)); end
if any(strcmp(cfg.models,'pi_phys')) && ~isempty(pi_phys), id_models{end+1} = struct('name','Y\pi_{phys}','m',struct('type','full','limb',cfg.limb,'para_order',cfg.para_order,'pi_vec',pi_phys)); end

R = struct(); R.id = struct('rmse', struct()); R.fd = struct('rmse', struct(), 'rmse_qd', struct());
for i = 1:numel(id_models)
    pred = zeros(N_plot, n);
    for k = 1:N_plot
        pred(k,:) = inverse_dynamics_dispatch(q(k,:), qd(k,:), qdd(k,:), id_models{i}.m).';
    end
    id_preds = [id_preds, pred]; %#ok<AGROW>
    key = matlab.lang.makeValidName(id_models{i}.name);
    R.id.rmse.(key) = sqrt(mean((pred - tau(1:N_plot,:)).^2, 1));
end
if cfg.do_plot && ~isempty(id_models)
    id_names = cell(1, numel(id_models));
    for i = 1:numel(id_models), id_names{i} = id_models{i}.name; end
    plot_compare_with_error_6dof(t(1:N_plot), tau(1:N_plot,:), '\tau_{meas/id}', id_preds, ...
        id_names, 'torque', 'ID对比');
end

% FD
fd_models = {};
if any(strcmp(cfg.models,'cad')), fd_models{end+1} = struct('name','FD(CAD)','m',struct('type','urdf','limb',cfg.limb,'para_order',cfg.para_order)); end
if any(strcmp(cfg.models,'beta')), fd_models{end+1} = struct('name','FD(\beta)','m',struct('type','min','limb',cfg.limb,'para_order',cfg.para_order,'X_hat',X_hat,'index_base',index_base)); end
if any(strcmp(cfg.models,'pi_fd')) && ~isempty(pi_fd), fd_models{end+1} = struct('name','FD(\pi_{fd})','m',struct('type','full','limb',cfg.limb,'para_order',cfg.para_order,'pi_vec',pi_fd)); end
if any(strcmp(cfg.models,'pi_rec')) && ~isempty(pi_rec), fd_models{end+1} = struct('name','FD(\pi_{rec})','m',struct('type','full','limb',cfg.limb,'para_order',cfg.para_order,'pi_vec',pi_rec)); end
if any(strcmp(cfg.models,'pi_phys')) && ~isempty(pi_phys), fd_models{end+1} = struct('name','FD(\pi_{phys})','m',struct('type','full','limb',cfg.limb,'para_order',cfg.para_order,'pi_vec',pi_phys)); end

fd_preds = [];
for i = 1:numel(fd_models)
    pred = zeros(N_fd, n);
    for k = 1:N_fd
        pred(k,:) = forward_dynamics_dispatch(q(k,:), qd(k,:), tau(k,:), fd_models{i}.m, struct()).';
    end
    fd_preds = [fd_preds, pred]; %#ok<AGROW>
    key = matlab.lang.makeValidName(fd_models{i}.name);
    R.fd.rmse.(key) = sqrt(mean((pred - qdd(1:N_fd,:)).^2, 1));
end

% qdd 积分得到 qd，用于正动力学速度误差分析
fd_preds_qd = [];
qd_ref_fd = qd(1:N_fd, :);
for i = 1:numel(fd_models)
    qdd_pred_i = fd_preds(:, (i-1)*n+1:i*n);
    qd_pred_i = integrate_qdd_to_qd(t(1:N_fd), qd_ref_fd(1,:), qdd_pred_i);
    fd_preds_qd = [fd_preds_qd, qd_pred_i]; %#ok<AGROW>
    key = matlab.lang.makeValidName(fd_models{i}.name);
    R.fd.rmse_qd.(key) = sqrt(mean((qd_pred_i - qd_ref_fd).^2, 1));
end
if cfg.do_plot && ~isempty(fd_models)
    fd_names = cell(1, numel(fd_models));
    for i = 1:numel(fd_models), fd_names{i} = fd_models{i}.name; end
    plot_compare_with_error_6dof(t(1:N_fd), qd_ref_fd, 'qd_{id轨迹}', fd_preds_qd, ...
        fd_names, 'qdd', 'FD速度对比');
    plot_compare_with_error_6dof(t(1:N_fd), qdd(1:N_fd,:), 'qdd_{id轨迹}', fd_preds, ...
        fd_names, 'qdd', 'FD对比');
end

if cfg.save_excel
    out_xlsx = fullfile(app_root, 'build', 'plot_id_fd_only_summary.xlsx');
    meta_tbl = table(string(cfg.input_mat), N_plot, N_fd, 'VariableNames', {'input_mat','N_plot','N_fd'});
    export_validation_summary_excel(out_xlsx, R.id.rmse, R.fd.rmse, meta_tbl);
end
end

function s = set_default(s, name, val)
if ~isfield(s, name) || isempty(s.(name)), s.(name) = val; end
end

function v = pick_field(s, name, default_v)
if isfield(s, name) && ~isempty(s.(name)), v = s.(name); else, v = default_v; end
end

function qd_pred = integrate_qdd_to_qd(t, qd0, qdd_pred)
N = size(qdd_pred, 1);
n = size(qdd_pred, 2);
qd_pred = zeros(N, n);
qd_pred(1, :) = qd0;
if N <= 1, return; end
dt_med = median(diff(t));
for k = 2:N
    dt = t(k) - t(k-1);
    if dt <= 0 || isnan(dt), dt = dt_med; end
    qd_pred(k, :) = qd_pred(k-1, :) + dt * qdd_pred(k-1, :);
end
end

