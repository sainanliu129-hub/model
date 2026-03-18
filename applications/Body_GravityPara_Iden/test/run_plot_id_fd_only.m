function R = run_plot_id_fd_only(cfg)
% run_plot_id_fd_only 统一“正逆动力学验证/画图”入口（函数）
%
% 目标：不跑任何辨识/优化，只读 build/id_param_sets.mat（或 min_param_id_result.mat）
%      按 cfg.models 选择要对比的模型，并绘图/输出误差统计。
%
% 用法（默认）：
%   run_plot_id_fd_only();
%
% 用法（自定义）：
%   cfg = struct(); cfg.models = {'pi_fd','beta','cad'}; cfg.N_fd=300; cfg.do_plot=false;
%   R = run_plot_id_fd_only(cfg);

    if nargin < 1, cfg = struct(); end

    clc;
    app_root = fullfile(fileparts(mfilename('fullpath')), '..');
    addpath(fullfile(app_root, 'principle'));
    addpath(app_root);
    ensure_body_gravity_para_iden_path();

    cfg = set_default(cfg, 'models', {'pi_fd','cad','beta'});
    cfg = set_default(cfg, 'limb', 'left_leg');
    cfg = set_default(cfg, 'para_order', 1);
    cfg = set_default(cfg, 'N_plot', inf);
    cfg = set_default(cfg, 'N_fd', 500);
    cfg = set_default(cfg, 'do_plot', true);

    models = string(cfg.models);
    limb = cfg.limb;
    para_order = cfg.para_order;

    % 输入文件（允许外部指定，方便“辨识轨迹A + 对比轨迹B”）
    build_dir = fullfile(app_root, 'build');
    cfg = set_default(cfg, 'input_mat', fullfile(build_dir, 'id_param_sets.mat'));
    set_mat = cfg.input_mat;
    min_mat = fullfile(app_root, 'min_param_id_result.mat');
    fd_mat  = fullfile(build_dir, 'pi_fd_result.mat');

    Sset = struct();
    if isfile(set_mat)
        Sset = load(set_mat);
    elseif isfile(min_mat)
        Sset = load(min_mat);
    else
        error('未找到 %s 或 %s。请先跑 run_min_param_id_from_csv 或 run_identify_pi_fd_only。', set_mat, min_mat);
    end

    if isfield(Sset, 'avg_data')
        avg_data = Sset.avg_data;
        X_hat = Sset.X_hat(:);
        index_base = Sset.index_base(:);
    else
        error('输入 mat 缺少 avg_data/X_hat/index_base。请确认来源文件。');
    end

    t_all   = avg_data.t_equiv(:);
    q_all   = avg_data.q_bar;
    qd_all  = avg_data.qd_bar;
    qdd_all = avg_data.qdd_bar;
    tau_all = avg_data.tau_bar;

    M = size(q_all, 1);
    N_plot = min(M, cfg.N_plot);
    N_fd   = min(M, cfg.N_fd);

    % 参数集合（CAD/β 必有；π_fd/π_rec/π_phys 可能没有）
    [robot_limb, n] = get_e1_limb_robot(limb);
    pi_cad = get_limb_theta_from_URDF(robot_limb, para_order);
    pi_cad = pi_cad(:);

    pi_fd = [];
    pi_rec = [];
    pi_phys = [];
    if isfield(Sset, 'pi_fd') && ~isempty(Sset.pi_fd), pi_fd = Sset.pi_fd(:); end
    if isempty(pi_fd) && isfile(fd_mat)
        Sd = load(fd_mat);
        if isfield(Sd, 'pi_fd') && ~isempty(Sd.pi_fd), pi_fd = Sd.pi_fd(:); end
    end
    if isfield(Sset, 'pi_rec') && ~isempty(Sset.pi_rec), pi_rec = Sset.pi_rec(:); end
    if isfield(Sset, 'pi_phys') && ~isempty(Sset.pi_phys), pi_phys = Sset.pi_phys(:); end

    % --------------------
    % ID: tau
    % --------------------
    tau_id = tau_all(1:N_plot, :);
    preds_tau = struct();
    names_tau = {};
    mats_tau = {};

    if any(models == "cad")
        preds_tau.cad = zeros(N_plot, n);
        names_tau{end+1} = 'Y\pi_{cad}'; %#ok<AGROW>
    end
    if any(models == "beta")
        preds_tau.beta = zeros(N_plot, n);
        names_tau{end+1} = 'Y_{min}X_{hat}'; %#ok<AGROW>
    end
    if any(models == "pi_fd") && ~isempty(pi_fd)
        preds_tau.pi_fd = zeros(N_plot, n);
        names_tau{end+1} = 'Y\pi_{fd}'; %#ok<AGROW>
    end
    if any(models == "pi_rec") && ~isempty(pi_rec)
        preds_tau.pi_rec = zeros(N_plot, n);
        names_tau{end+1} = 'Y\pi_{rec}'; %#ok<AGROW>
    end
    if any(models == "pi_phys") && ~isempty(pi_phys)
        preds_tau.pi_phys = zeros(N_plot, n);
        names_tau{end+1} = 'Y\pi_{phys}'; %#ok<AGROW>
    end

    for k = 1:N_plot
        qk   = q_all(k, :);
        qdk  = qd_all(k, :);
        qddk = qdd_all(k, :);
        Y_one = ReMatrix_E1_limb_URDF(limb, qk, qdk, qddk, 1, para_order);
        if isfield(preds_tau, 'cad'),  preds_tau.cad(k,:)  = (Y_one * pi_cad).'; end
        if isfield(preds_tau, 'beta'), preds_tau.beta(k,:) = (Y_one(:, index_base) * X_hat).'; end
        if isfield(preds_tau, 'pi_fd'),   preds_tau.pi_fd(k,:)   = (Y_one * pi_fd).'; end
        if isfield(preds_tau, 'pi_rec'),  preds_tau.pi_rec(k,:)  = (Y_one * pi_rec).'; end
        if isfield(preds_tau, 'pi_phys'), preds_tau.pi_phys(k,:) = (Y_one * pi_phys).'; end
    end

    % 组装绘图矩阵（tau_id 在第 1 组）
    mats_tau{1} = tau_id; %#ok<AGROW>
    leg_tau = {'\tau_{meas/id}'};
    fields_order = {'pi_fd','cad','beta','pi_rec','pi_phys'};
    for i = 1:numel(fields_order)
        f = fields_order{i};
        if isfield(preds_tau, f)
            mats_tau{end+1} = preds_tau.(f); %#ok<AGROW>
            leg_tau{end+1} = map_name(f); %#ok<AGROW>
        end
    end

    % 误差统计
    R = struct();
    R.id = struct();
    R.id.rmse = struct();
    if isfield(preds_tau, 'cad'),  R.id.rmse.cad  = sqrt(mean((preds_tau.cad  - tau_id).^2, 1)); end
    if isfield(preds_tau, 'beta'), R.id.rmse.beta = sqrt(mean((preds_tau.beta - tau_id).^2, 1)); end
    if isfield(preds_tau, 'pi_fd'),   R.id.rmse.pi_fd   = sqrt(mean((preds_tau.pi_fd - tau_id).^2, 1)); end
    if isfield(preds_tau, 'pi_rec'),  R.id.rmse.pi_rec  = sqrt(mean((preds_tau.pi_rec - tau_id).^2, 1)); end
    if isfield(preds_tau, 'pi_phys'), R.id.rmse.pi_phys = sqrt(mean((preds_tau.pi_phys - tau_id).^2, 1)); end

    if cfg.do_plot
        figure('Name', 'ID_tau_compare');
        plot_compare_6dof(t_all(1:N_plot), horzcat(mats_tau{:}), 'torque', leg_tau);
        sgtitle('逆动力学：实测 vs 多模型对比');
    end

    % --------------------
    % FD: qdd
    % --------------------
    qdd_id = qdd_all(1:N_fd, :);
    preds_qdd = struct();
    R.fd = struct();
    R.fd.rmse = struct();

    if any(models == "cad"), preds_qdd.cad = zeros(N_fd, n); end
    if any(models == "beta"), preds_qdd.beta = zeros(N_fd, n); end
    if any(models == "pi_fd") && ~isempty(pi_fd), preds_qdd.pi_fd = zeros(N_fd, n); end
    if any(models == "pi_rec") && ~isempty(pi_rec), preds_qdd.pi_rec = zeros(N_fd, n); end
    if any(models == "pi_phys") && ~isempty(pi_phys), preds_qdd.pi_phys = zeros(N_fd, n); end

    for k = 1:N_fd
        qk = q_all(k, :);
        qdk = qd_all(k, :);
        tauk = tau_all(k, :);
        if isfield(preds_qdd, 'cad'),  preds_qdd.cad(k,:)  = forwardDynamics(robot_limb, qk, qdk, tauk).'; end
        if isfield(preds_qdd, 'beta'), preds_qdd.beta(k,:) = forward_dynamics_min(qk.', qdk.', tauk(:), X_hat, index_base, limb, para_order).'; end
        if isfield(preds_qdd, 'pi_fd'),   preds_qdd.pi_fd(k,:)   = forward_dynamics_full(qk, qdk, tauk, pi_fd, limb, para_order).'; end
        if isfield(preds_qdd, 'pi_rec'),  preds_qdd.pi_rec(k,:)  = forward_dynamics_full(qk, qdk, tauk, pi_rec, limb, para_order).'; end
        if isfield(preds_qdd, 'pi_phys'), preds_qdd.pi_phys(k,:) = forward_dynamics_full(qk, qdk, tauk, pi_phys, limb, para_order).'; end
    end

    if isfield(preds_qdd, 'cad'),  R.fd.rmse.cad  = sqrt(mean((preds_qdd.cad  - qdd_id).^2, 1)); end
    if isfield(preds_qdd, 'beta'), R.fd.rmse.beta = sqrt(mean((preds_qdd.beta - qdd_id).^2, 1)); end
    if isfield(preds_qdd, 'pi_fd'),   R.fd.rmse.pi_fd   = sqrt(mean((preds_qdd.pi_fd - qdd_id).^2, 1)); end
    if isfield(preds_qdd, 'pi_rec'),  R.fd.rmse.pi_rec  = sqrt(mean((preds_qdd.pi_rec - qdd_id).^2, 1)); end
    if isfield(preds_qdd, 'pi_phys'), R.fd.rmse.pi_phys = sqrt(mean((preds_qdd.pi_phys - qdd_id).^2, 1)); end

    if cfg.do_plot
        mats_qdd = {qdd_id};
        leg_qdd = {'qdd_{id轨迹}'};
        if isfield(preds_qdd, 'pi_fd'), mats_qdd{end+1} = preds_qdd.pi_fd; leg_qdd{end+1} = 'FD(\pi_{fd})'; end
        if isfield(preds_qdd, 'cad'),  mats_qdd{end+1} = preds_qdd.cad;  leg_qdd{end+1} = 'FD(CAD)'; end
        if isfield(preds_qdd, 'beta'), mats_qdd{end+1} = preds_qdd.beta; leg_qdd{end+1} = 'FD(\beta)'; end
        if isfield(preds_qdd, 'pi_rec'),  mats_qdd{end+1} = preds_qdd.pi_rec;  leg_qdd{end+1} = 'FD(\pi_{rec})'; end
        if isfield(preds_qdd, 'pi_phys'), mats_qdd{end+1} = preds_qdd.pi_phys; leg_qdd{end+1} = 'FD(\pi_{phys})'; end

        figure('Name', 'FD_qdd_compare');
        plot_compare_6dof(t_all(1:N_fd), horzcat(mats_qdd{:}), 'qdd', leg_qdd);
        sgtitle('正动力学：qdd_{id} vs 多模型对比');
    end
end

function s = set_default(s, name, val)
    if ~isfield(s, name) || isempty(s.(name))
        s.(name) = val;
    end
end

function nm = map_name(field)
    switch field
        case 'pi_fd', nm = 'Y\pi_{fd}';
        case 'cad', nm = 'Y\pi_{cad}';
        case 'beta', nm = 'Y_{min}X_{hat}';
        case 'pi_rec', nm = 'Y\pi_{rec}';
        case 'pi_phys', nm = 'Y\pi_{phys}';
        otherwise, nm = field;
    end
end
