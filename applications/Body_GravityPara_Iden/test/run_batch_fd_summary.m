function run_batch_fd_summary(opts)
% run_batch_fd_summary  批量生成“正动力学对比与误差汇总”，使用最新参数表/缓存
%
% - 读取：
%   build/id_param_sets.mat    包含 pi_cad / pi_rec / pi_phys / pi_fd / X_hat / index_base / avg_data / limb / para_order
%   或 build/fd_cache.mat      若需要裁剪轨迹（优先使用 cache 的 q_id/qd_id/qdd_id/tau_id）
%
% - 输出：
%   build/fd_acc_error_summary.csv      各“参数类型”的加速度 RMSE / maxerr 摘要
%   build/plots/fd_*.png                对应每种“参数类型”的 qdd 对比图
%
% 用法：
%   run_batch_fd_summary();
%
if nargin < 1, opts = struct(); end
if ~isfield(opts, 'M_fd'), opts.M_fd = inf; end

clc;
app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
ensure_body_gravity_para_iden_path();

build_dir = fullfile(app_root, 'build');
plot_dir  = fullfile(build_dir, 'plots');
if ~isfolder(build_dir), mkdir(build_dir); end
if ~isfolder(plot_dir),  mkdir(plot_dir);  end

Sparam = load(fullfile(build_dir, 'id_param_sets.mat')); % 由 run_batch_id_summary 生成
limb = Sparam.limb; para_order = Sparam.para_order;
[robot_limb, n] = get_e1_limb_robot(limb);

% 轨迹数据优先从缓存读取；否则从 avg_data 取整段
cache_path = fullfile(build_dir, 'fd_cache.mat');
if isfile(cache_path)
    Sc = load(cache_path);
    q_id   = Sc.q_id;  qd_id  = Sc.qd_id;
    qdd_id = Sc.qdd_id; tau_id = Sc.tau_id;
else
    q_id   = Sparam.avg_data.q_bar;
    qd_id  = Sparam.avg_data.qd_bar;
    qdd_id = Sparam.avg_data.qdd_bar;
    tau_id = Sparam.avg_data.tau_bar;
end
M = size(q_id, 1);
M_fd = min(M, opts.M_fd);

% 可用的“参数类型”
types = {};
types{end+1} = 'FD(CAD)';      % forwardDynamics(robot, ...)
types{end+1} = 'FD(beta)';     % forward_dynamics_min
has_rec = isfield(Sparam, 'pi_rec')  && ~isempty(Sparam.pi_rec);
has_phys= isfield(Sparam, 'pi_phys') && ~isempty(Sparam.pi_phys);
has_fd  = isfield(Sparam, 'pi_fd')   && ~isempty(Sparam.pi_fd);
if has_rec,  types{end+1} = 'FD(pi_rec)';  end
if has_phys, types{end+1} = 'FD(pi_phys)'; end
if has_fd,   types{end+1} = 'FD(pi_fd)';   end

% 计算 qdd 预测
qdd_pred = struct();
for k = 1:M_fd
    qk = q_id(k, :);
    qdk = qd_id(k, :);
    tauk = tau_id(k, :);
    % CAD
    qdd_pred.('FD(CAD)')(k, :) = forwardDynamics(robot_limb, qk, qdk, tauk).';
    % beta
    qdd_pred.('FD(beta)')(k, :) = forward_dynamics_min(qk.', qdk.', tauk(:), Sparam.X_hat, Sparam.index_base, limb, para_order).';
    % pi_rec
    if has_rec
        qdd_pred.('FD(pi_rec)')(k, :) = forward_dynamics_full(qk, qdk, tauk, Sparam.pi_rec, limb, para_order).';
    end
    % pi_phys
    if has_phys
        qdd_pred.('FD(pi_phys)')(k, :) = forward_dynamics_full(qk, qdk, tauk, Sparam.pi_phys, limb, para_order).';
    end
    % pi_fd
    if has_fd
        qdd_pred.('FD(pi_fd)')(k, :) = forward_dynamics_full(qk, qdk, tauk, Sparam.pi_fd, limb, para_order).';
    end
end

% 误差统计与作图
rows_err = {};
plot_paths = containers.Map();
for i = 1:numel(types)
    tag = types{i};
    qddp = qdd_pred.(tag);
    err = qddp - qdd_id(1:M_fd, :);
    rmse = sqrt(mean(err.^2, 1));
    mxe  = max(abs(err), [], 1);
    rows_err{end+1, 1} = tag; %#ok<AGROW>
    rows_err{end,   2} = rmse;
    rows_err{end,   3} = mxe;
    % 绘图
    try
        h = figure('Name', ['FD_qdd_compare_' tag], 'Visible', 'off');
        vars = {qdd_id(1:M_fd,:), qddp};
        names = {'qdd_{id轨迹}', tag};
        plot_compare_6dof((1:M_fd)', horzcat(vars{:}), 'qdd', names);
        sgtitle(['正动力学：' strrep(tag, '_', '\_')]);
        out_png = fullfile(plot_dir, ['fd_' strrep(tag, '*', '') '.png']);
        saveas(h, out_png);
        close(h);
        plot_paths(tag) = out_png;
    catch
        plot_paths(tag) = '';
    end
end

% 写出误差表
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
out_err_csv = fullfile(build_dir, 'fd_acc_error_summary.csv');
writetable(T_err, out_err_csv);
fprintf('[FD] 已写入误差表: %s\n', out_err_csv);

end

