function S = run_identify_pi_fd_only(cfg)
% run_identify_pi_fd_only 统一“辨识/优化”入口（函数）
%
% 目标：把所有参数辨识/优化集中在一个地方，你通过 cfg.models 选择要跑的模型。
%
% - 必备输入（两选一）：
%   A) cfg.csv_file + cfg.min_opts（会内部调用 run_min_param_id_from_csv 重新生成 min_param_id_result.mat）
%   B) 已存在 applications/Body_GravityPara_Iden/min_param_id_result.mat（直接加载）
%
% - 可选输出（按 cfg.models 控制）：
%   - 'pi_rec' : recover_full_params_from_beta
%   - 'pi_fd'  : identify_full_params_for_fd
%   - 'pi_phys': solve_full_params_physical（如果你暂时不想跑，别在 models 里加它）
%
% 保存：
%   - build/id_param_sets.mat（统一给“画图/验证”用）
%   - build/pi_fd_result.mat（兼容单独读取 π_fd）
%
% 用法（默认配置）：
%   run_identify_pi_fd_only();
% 用法（自定义）：
%   cfg = struct(); cfg.models = {'pi_fd'}; cfg.opts_fd.MaxFunctionEvaluations = 3000;
%   run_identify_pi_fd_only(cfg);

    if nargin < 1, cfg = struct(); end

    clc;
    app_root = fullfile(fileparts(mfilename('fullpath')), '..');
    addpath(fullfile(app_root, 'principle'));
    addpath(app_root);
    ensure_body_gravity_para_iden_path();

    % --------------------
    % 默认 cfg
    % --------------------
    cfg = set_default(cfg, 'limb', 'left_leg');
    cfg = set_default(cfg, 'para_order', 1);
    cfg = set_default(cfg, 'models', {'pi_fd'}); % 可选：{'pi_rec','pi_fd','pi_phys'}

    cfg = set_default(cfg, 'save_dir', fullfile(app_root, 'build'));
    cfg = set_default(cfg, 'save_id_param_sets', true);
    cfg = set_default(cfg, 'save_pi_fd_result', true);

    cfg = set_default(cfg, 'csv_file', []);
    cfg = set_default(cfg, 'data_cfg', struct()); % 传给 make_avg_data_from_csv（预处理/取窗/周期平均）
    cfg = set_default(cfg, 'force_reidentify_min', false);
    cfg = set_default(cfg, 'save_min_result_mat', true); % 是否写 min_param_id_result.mat（缓存）

    % π_rec（β->π）选项
    cfg = set_default(cfg, 'lambda_rec', 1e-2);

    % π_fd（FD 导向优化）选项
    cfg = set_default(cfg, 'opts_fd', default_opts_fd_quick());
    cfg = set_default(cfg, 'n_qdd', 8);
    cfg = set_default(cfg, 'n_reg', 4);
    cfg = set_default(cfg, 'use_prev_pi_fd_as_init', false);

    % π_phys（物理约束）选项
    cfg = set_default(cfg, 'lambda_phys', 10);
    cfg = set_default(cfg, 'opts_phys', struct());

    limb = cfg.limb;
    para_order = cfg.para_order;
    models = string(cfg.models);

    if ~isfolder(cfg.save_dir), mkdir(cfg.save_dir); end

    % --------------------
    % 0) 数据预处理 + 最小参数辨识：生成/加载 min_param_id_result.mat
    % --------------------
    min_mat = fullfile(app_root, 'min_param_id_result.mat');
    if cfg.force_reidentify_min && isfile(min_mat)
        delete(min_mat);
        fprintf('已删除旧的 min_param_id_result.mat，将重新生成。\n');
    end
    meta_data = struct();
    metrics_min = struct();
    if ~isfile(min_mat) && isempty(cfg.csv_file)
        error('缺少 min_param_id_result.mat，且 cfg.csv_file 为空。请提供 cfg.csv_file 或先生成该 mat。');
    end

    if isfile(min_mat) && isempty(cfg.csv_file)
        % 仅复用缓存
        ld = load(min_mat);
        X_hat = ld.X_hat(:);
        index_base = ld.index_base(:);
        avg_data = ld.avg_data;
        if isfield(ld, 'metrics'), metrics_min = ld.metrics; end
    else
        % 从 CSV 走“预处理->取窗/周期平均->identify_min”
        fprintf('将从 CSV 生成 avg_data 并辨识最小参数（identify_min）。\n');
        data_cfg = cfg.data_cfg;
        data_cfg = set_default(data_cfg, 'limb', cfg.limb);
        [avg_data, meta_data] = make_avg_data_from_csv(cfg.csv_file, data_cfg);

        id_opts = struct();
        id_opts.para_order = para_order;
        % 兼容：若 data_cfg 里传了摩擦/转子惯量，也允许 identify_min 再补偿一次（默认不传）
        if isfield(data_cfg, 'friction_params'), id_opts.friction_params = data_cfg.friction_params; end
        if isfield(data_cfg, 'I_a'), id_opts.I_a = data_cfg.I_a; end
        [X_hat, index_base, metrics_min] = identify_min(avg_data, limb, id_opts);

        if cfg.save_min_result_mat
            Sm = struct();
            Sm.X_hat = X_hat;
            Sm.index_base = index_base;
            Sm.avg_data = avg_data;
            Sm.metrics = metrics_min;
            Sm.csv_file = cfg.csv_file;
            Sm.data_cfg = data_cfg;
            Sm.saved_at = datestr(now);
            save(min_mat, '-struct', 'Sm');
            fprintf('已写入: %s\n', min_mat);
        end
    end

    q_bar   = avg_data.q_bar;
    qd_bar  = avg_data.qd_bar;
    qdd_bar = avg_data.qdd_bar;
    tau_bar = avg_data.tau_bar;
    M = size(q_bar, 1);

    % --------------------
    % 1) CAD 全参/回归矩阵/K
    % --------------------
    [robot_limb, n] = get_e1_limb_robot(limb);
    pi_cad = get_limb_theta_from_URDF(robot_limb, para_order);
    pi_cad = pi_cad(:);

    Y_full = ReMatrix_E1_limb_URDF(limb, q_bar, qd_bar, qdd_bar, 1, para_order);
    Y_min = Y_full(:, index_base);
    K = pinv(Y_min) * Y_full;
    beta_cad = K * pi_cad;

    % --------------------
    % 2) 按需辨识/恢复各模型
    % --------------------
    pi_rec = [];
    pi_fd  = [];
    pi_phys = [];
    info_fd = struct();
    info_phys = struct();

    if any(models == "pi_rec")
        fprintf('\n===== β→π 恢复：pi_rec（recover_full_params_from_beta） =====\n');
        pi_rec = recover_full_params_from_beta(K, X_hat, pi_cad, cfg.lambda_rec);
    end

    if any(models == "pi_fd")
        fprintf('\n===== FD 导向 full 参数辨识：pi_fd（identify_full_params_for_fd） =====\n');
        opts_fd = cfg.opts_fd;
        n_qdd = min(cfg.n_qdd, M);
        n_reg = min(cfg.n_reg, M);
        opts_fd.idx_qdd = round(linspace(1, M, n_qdd)).';
        opts_fd.idx_reg = round(linspace(1, M, n_reg)).';

        if cfg.use_prev_pi_fd_as_init
            prev_path = fullfile(cfg.save_dir, 'pi_fd_result.mat');
            if isfile(prev_path)
                Sp = load(prev_path);
                if isfield(Sp, 'pi_fd') && ~isempty(Sp.pi_fd)
                    opts_fd.pi0 = Sp.pi_fd(:);
                    fprintf('使用上次 pi_fd 作为初值: %s\n', prev_path);
                end
            end
        end

        [pi_fd, info_fd] = identify_full_params_for_fd( ...
            q_bar, qd_bar, qdd_bar, tau_bar, ...
            pi_cad, limb, para_order, opts_fd);
        fprintf('  [fd] exitflag = %d, fval = %.3e\n', info_fd.exitflag, info_fd.fval);
        fprintf('  [fd] rmse_tau = %.4e, rmse_qdd = %.4e\n', info_fd.rmse_tau, info_fd.rmse_qdd);
    end

    if any(models == "pi_phys")
        fprintf('\n===== 物理约束 full 参数：pi_phys（solve_full_params_physical） =====\n');
        opts_phys = cfg.opts_phys;
        [pi_phys, info_phys] = solve_full_params_physical(Y_full, Y_min, X_hat, pi_cad, cfg.lambda_phys, opts_phys);
        fprintf('  [phys] exitflag = %d\n', info_phys.exitflag);
        fprintf('  [phys] fit_res_norm = %.4e, reg_res_norm = %.4e\n', info_phys.fit_res_norm, info_phys.reg_res_norm);
    end

    % --------------------
    % 3) 统一保存（给后续绘图/验证用）
    % --------------------
    S = struct();
    S.saved_at = datestr(now);
    S.app_root = app_root;
    S.limb = limb;
    S.para_order = para_order;
    S.models = cellstr(models);

    S.min_mat = min_mat;
    S.X_hat = X_hat;
    S.index_base = index_base;
    S.avg_data = avg_data;
    S.metrics_min = metrics_min;
    S.meta_data = meta_data;

    S.pi_cad = pi_cad;
    S.K = K;
    S.beta_cad = beta_cad;

    S.pi_rec = pi_rec;
    S.pi_fd = pi_fd;
    S.pi_phys = pi_phys;
    S.info_fd = info_fd;
    S.info_phys = info_phys;
    S.cfg = cfg;

    if cfg.save_id_param_sets
        out1 = fullfile(cfg.save_dir, 'id_param_sets.mat');
        save(out1, '-struct', 'S');
        fprintf('已写入: %s\n', out1);
    end

    if cfg.save_pi_fd_result && ~isempty(pi_fd)
        out2 = fullfile(cfg.save_dir, 'pi_fd_result.mat');
        Sd = struct();
        Sd.pi_fd = pi_fd;
        Sd.info_fd = info_fd;
        Sd.opts_fd = cfg.opts_fd;
        Sd.limb = limb;
        Sd.para_order = para_order;
        Sd.pi_cad = pi_cad;
        Sd.index_base = index_base;
        Sd.X_hat = X_hat;
        Sd.source_mat = min_mat;
        Sd.saved_at = datestr(now);
        save(out2, '-struct', 'Sd');
        fprintf('已写入: %s\n', out2);
    end
end

% =========================
% helpers (local)
% =========================
function s = set_default(s, name, val)
    if ~isfield(s, name) || isempty(s.(name))
        s.(name) = val;
    end
end

function opts = default_opts_fd_quick()
    opts = struct();
    opts.traj_H = 1; % J_traj 短窗步数（w_traj>0 时生效；默认与单步速度/轨迹一致）
    opts.w_tau = 1;
    opts.w_qdd = 10;
    opts.w_cad = 1;
    opts.w_M   = 0;
    opts.m_min_frac = 0.7;
    opts.m_max_frac = 1.3;
    opts.delta_c    = 0.02;
    opts.eps_I      = 1e-4;
    opts.eps_M      = 1e-6;
    opts.algorithm  = 'sqp';
    opts.display    = 'iter';
    opts.max_iter   = 80;
    opts.MaxFunctionEvaluations = 1500;
    opts.FiniteDifferenceType = 'forward';
end
