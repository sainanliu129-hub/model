function run_batch_id_summary(opts)
% run_batch_id_summary  批量生成“逆动力学拟合与误差汇总”，并保存参数/误差表与图
%
% - 汇总的“参数类型”：
%   1) CAD 全参:               pi_cad
%   2) 最小参数:               (index_base, X_hat)
%   3) 最小参数(CAD映射):      beta_cad = K*pi_cad
%   4) β→π 简单恢复:           pi_rec = recover_full_params_from_beta
%   5) 物理约束 full 参数:      pi_phys = solve_full_params_physical
%   6) 可选：FD 导向 full 参数: pi_fd（若缓存中存在）
%
% - 输出：
%   build/id_param_summary.csv               按 link 的参数摘要（m / c / minEigI / condI）
%   build/id_torque_error_summary.csv        各“参数类型”的力矩 RMSE / maxerr 摘要
%   build/plots/id_*.png                     对应每种“参数类型”的 τ 拟合曲线对比图
%   build/id_param_sets.mat                  各类型参数的结构化保存，便于后续 FD 复用
%
% 用法：
%   run_batch_id_summary();
%   run_batch_id_summary(struct('lambda_rec', 1e-2));
%
if nargin < 1, opts = struct(); end
if ~isfield(opts, 'lambda_rec'), opts.lambda_rec = 1e-2; end   % recover_full_params_from_beta 的正则
if ~isfield(opts, 'phys_lambda'), opts.phys_lambda = 10; end   % solve_full_params_physical 的正则系数

clc;
app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
ensure_body_gravity_para_iden_path();

build_dir = fullfile(app_root, 'build');
plot_dir  = fullfile(build_dir, 'plots');
if ~isfolder(build_dir), mkdir(build_dir); end
if ~isfolder(plot_dir),  mkdir(plot_dir);  end

% 1) 载入最小参数结果
min_res = load(fullfile(app_root, 'min_param_id_result.mat'));  % 包含 X_hat, index_base, avg_data
X_hat = min_res.X_hat(:);
index_base = min_res.index_base(:);
avg_data = min_res.avg_data;

% 2) 构造 limb/URDF/CAD
limb = 'left_leg'; para_order = 1;
[robot_limb, n] = get_e1_limb_robot(limb);
pi_cad = get_limb_theta_from_URDF(robot_limb, para_order);
pi_cad = pi_cad(:);

% 3) 用辨识轨迹批量生成回归矩阵
q_id   = avg_data.q_bar;
qd_id  = avg_data.qd_bar;
qdd_id = avg_data.qdd_bar;
tau_id = avg_data.tau_bar;   % 若预处理为 use_preprocess_id，则是 τ_id
M = size(q_id,1);
Y_full = ReMatrix_E1_limb_URDF(limb, q_id, qd_id, qdd_id, 1, para_order);
Y_min  = Y_full(:, index_base);
K = pinv(Y_min) * Y_full;
beta_cad = K * pi_cad;

% 4) 计算多种“参数类型”的 τ 预测，并画图/误差
types = {}; preds = struct();
% 4.1 CAD
types{end+1} = 'Y*pi_cad';
preds.(types{end}) = reshape(Y_full * pi_cad, M, n);
% 4.2 最小参数（辨识）
types{end+1} = 'Ymin*X_hat';
preds.(types{end}) = reshape(Y_min * X_hat, M, n);
% 4.3 最小参数（CAD映射）
types{end+1} = 'Ymin*beta_cad';
preds.(types{end}) = reshape(Y_min * beta_cad, M, n);
% 4.4 β→π 简单恢复
pi_rec = recover_full_params_from_beta(K, X_hat, pi_cad, opts.lambda_rec);
types{end+1} = 'Y*pi_rec';
preds.(types{end}) = reshape(Y_full * pi_rec, M, n);
% 4.5 物理约束 full 参数（稳健）
[pi_phys, info_phys] = solve_full_params_physical(Y_full, Y_min, X_hat, pi_cad, opts.phys_lambda, struct());
types{end+1} = 'Y*pi_phys';
preds.(types{end}) = reshape(Y_full * pi_phys, M, n);
% 4.6 若缓存中存在 pi_fd，也纳入
cache_path = fullfile(build_dir, 'fd_cache.mat');
pi_fd = [];
if isfile(cache_path)
    S = load(cache_path);
    if isfield(S, 'pi_fd'), pi_fd = S.pi_fd(:); end
end
if ~isempty(pi_fd)
    types{end+1} = 'Y*pi_fd';
    preds.(types{end}) = reshape(Y_full * pi_fd, M, n);
end

% 5) 误差统计与作图
rows_err = {};
plot_paths = containers.Map();
for i = 1:numel(types)
    tag = types{i};
    tau_pred = preds.(tag);
    err = tau_pred - tau_id;
    rmse = sqrt(mean(err.^2, 1));
    mxe  = max(abs(err), [], 1);
    rows_err{end+1, 1} = tag; %#ok<AGROW>
    rows_err{end,   2} = rmse;
    rows_err{end,   3} = mxe;
    % 绘图
    try
        h = figure('Name', ['ID_torque_compare_' tag], 'Visible', 'off');
        plot_compare_6dof((1:M)', [tau_id, tau_pred], 'torque', {'\tau_{meas/id}', tag});
        sgtitle(['逆动力学：' strrep(tag, '_', '\_')]);
        out_png = fullfile(plot_dir, ['id_' strrep(tag, '*', '') '.png']);
        saveas(h, out_png);
        close(h);
        plot_paths(tag) = out_png;
    catch
        plot_paths(tag) = '';
    end
end

% 6) 误差表写盘（每行一个“类型”，列为 per-joint 向量以字符串形式保存）
to_vec_str = @(v) sprintf('[%s]', strjoin(arrayfun(@(x) sprintf('%.4f', x), v(:).', 'UniformOutput', false), ', '));
T_err = table('Size', [numel(types), 4], ...
    'VariableTypes', {'string','string','string','string'}, ...
    'VariableNames', {'type','rmse_per_joint','maxerr_per_joint','plot_path'});
for i = 1:numel(types)
    T_err.type(i) = string(types{i});
    T_err.rmse_per_joint(i) = string(to_vec_str(rows_err{i,2}));
    T_err.maxerr_per_joint(i) = string(to_vec_str(rows_err{i,3}));
    key = types{i};
    pth = ''; if isKey(plot_paths, key), pth = plot_paths(key); end
    T_err.plot_path(i) = string(pth);
end
out_err_csv = fullfile(build_dir, 'id_torque_error_summary.csv');
writetable(T_err, out_err_csv);
fprintf('[ID] 已写入误差表: %s\n', out_err_csv);

% 7) 参数摘要表（按 link）
n_links = numel(pi_cad) / 10;
mk_summary = @(pvec) per_link_summary(pvec);
sum_cad  = mk_summary(pi_cad);
sum_rec  = mk_summary(pi_rec);
sum_phys = mk_summary(pi_phys);
S_param = table((1:n_links).', sum_cad.m, sum_rec.m, sum_phys.m, ...
    sum_cad.c(:,1), sum_rec.c(:,1), sum_phys.c(:,1), ...
    sum_cad.c(:,2), sum_rec.c(:,2), sum_phys.c(:,2), ...
    sum_cad.c(:,3), sum_rec.c(:,3), sum_phys.c(:,3), ...
    sum_cad.minEigI, sum_rec.minEigI, sum_phys.minEigI, ...
    sum_cad.condI, sum_rec.condI, sum_phys.condI, ...
    'VariableNames', {'link', ...
    'm_cad','m_rec','m_phys', ...
    'cx_cad','cx_rec','cx_phys', ...
    'cy_cad','cy_rec','cy_phys', ...
    'cz_cad','cz_rec','cz_phys', ...
    'minEigI_cad','minEigI_rec','minEigI_phys', ...
    'condI_cad','condI_rec','condI_phys'});
out_param_csv = fullfile(build_dir, 'id_param_summary.csv');
writetable(S_param, out_param_csv);
fprintf('[ID] 已写入参数摘要表: %s\n', out_param_csv);

% 8) 保存参数集合（便于 FD 使用最新）
save(fullfile(build_dir, 'id_param_sets.mat'), ...
    'pi_cad','pi_rec','pi_phys','pi_fd','X_hat','index_base','beta_cad','K','limb','para_order','avg_data');
fprintf('[ID] 已保存参数集合到: %s\n', fullfile(build_dir, 'id_param_sets.mat'));

end

% ===== 辅助：每 link 摘要 =====
function S = per_link_summary(pi_vec)
pi_vec = pi_vec(:);
L = numel(pi_vec) / 10;
m = zeros(L,1); c = zeros(L,3); minEigI = zeros(L,1); condI = zeros(L,1);
for i = 1:L
    idx = (i-1)*10 + (1:10);
    b = pi_vec(idx);
    m(i) = b(1);
    if abs(b(1)) > 1e-12
        c(i,:) = (b(2:4)/b(1)).';
    else
        c(i,:) = [0 0 0];
    end
    I = [b(5) b(6) b(7); b(6) b(8) b(9); b(7) b(9) b(10)];
    I = 0.5*(I+I.');
    e = eig(I);
    minEigI(i) = min(real(e));
    condI(i) = cond(I + 1e-12*eye(3));
end
S = struct('m', m, 'c', c, 'minEigI', minEigI, 'condI', condI);
end

