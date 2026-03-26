function out = validate_mixed_params_schemeA(dataset, limb, para_order, pi_cad, pi_id_best, cfg)
% validate_mixed_params_schemeA  混合参数方案验证（方案 A）
%
% 生成 4 组参数并评估：
%   组 1：全 CAD                 pi_all_cad = pi_cad
%   组 2：全辨识               pi_all_id  = pi_id_best
%   组 3：1~4 辨识，5~6 CAD     pi_mix_14id_56cad
%   组 4：1~4 CAD，5~6 辨识     pi_mix_14cad_56id
%
% 指标（每个字段都是 1x6，按关节分别 RMSE）：
%   1) ID RMSE（vs tau_meas）
%   2) FD RMSE（vs qdd_ref）
%   3) H=cfg.H_steps 串联预测 qd RMSE
%   4) 全程积分 qd RMSE + 全程积分 q RMSE
%
% 输入：
%   dataset   - 结构体，至少包含：
%       dataset.q   (M x  n)
%       dataset.qd  (M x  n)
%       dataset.qdd (M x  n)
%       dataset.tau (M x  n)
%       dataset.t   (M x  1)
%   limb       - 'left_leg' 等
%   para_order - 1（或 2）
%   pi_cad     - CAD 全参向量（长度 = 10*n_joint）
%   pi_id_best - 最优辨识全参向量（同长度）
%   cfg        - 可选配置（建议只改下面少量字段）：
%       .block_dim   默认 10（每关节 10 维参数块）
%       .n_joints    默认 6
%       .H_steps     默认 5
%       .M_fd_max    默认 inf（FD 只取前 M_fd 个样本）
%       .compute_id  默认 true
%       .compute_fd  默认 true
%       .forward_opts 传给 forward_dynamics_full 的 opts（可为空 struct）
%       .verbose     默认 true
%
% 输出 out：
%   out.id_rmse
%   out.fd_qdd_rmse
%   out.fd_qd_rmse_H
%   out.fd_qd_rmse_integral
%   out.fd_q_rmse_integral
%   out.pi_groups  （每组对应的 pi 向量）

    if nargin < 6 || isempty(cfg), cfg = struct(); end
    cfg = set_default(cfg, 'block_dim', 10);
    cfg = set_default(cfg, 'n_joints', 6);
    cfg = set_default(cfg, 'H_steps', 5);
    cfg = set_default(cfg, 'M_fd_max', inf);
    cfg = set_default(cfg, 'compute_id', true);
    cfg = set_default(cfg, 'compute_fd', true);
    cfg = set_default(cfg, 'forward_opts', struct());
    cfg = set_default(cfg, 'verbose', true);

    % -------- 基本一致性检查 --------
    assert(isfield(dataset, 'q') && isfield(dataset, 'qd') && isfield(dataset, 'qdd') && isfield(dataset, 'tau') && isfield(dataset, 't'), ...
        'validate_mixed_params_schemeA: dataset 缺少必要字段（q/qd/qdd/tau/t）。');
    [M, n] = size(dataset.q);
    assert(size(dataset.qd, 1) == M && size(dataset.qdd, 1) == M && size(dataset.tau, 1) == M, ...
        'validate_mixed_params_schemeA: dataset 的前维（样本数 M）不一致。');
    assert(size(dataset.qd, 2) == n && size(dataset.qdd, 2) == n && size(dataset.tau, 2) == n, ...
        'validate_mixed_params_schemeA: dataset 的关节维度 n 不一致。');
    if n ~= cfg.n_joints
        error('validate_mixed_params_schemeA: dataset 关节维度 n=%d 与 cfg.n_joints=%d 不一致。', n, cfg.n_joints);
    end

    pi_cad = pi_cad(:);
    pi_id_best = pi_id_best(:);
    exp_len = cfg.block_dim * cfg.n_joints;
    assert(numel(pi_cad) == exp_len, 'validate_mixed_params_schemeA: pi_cad 长度=%d，但期望 %d（block_dim*n_joints）。', numel(pi_cad), exp_len);
    assert(numel(pi_id_best) == exp_len, 'validate_mixed_params_schemeA: pi_id_best 长度=%d，但期望 %d（block_dim*n_joints）。', numel(pi_id_best), exp_len);

    t = dataset.t(:);
    if numel(t) ~= M
        error('validate_mixed_params_schemeA: dataset.t 长度=%d 但 dataset.q 样本数 M=%d 不一致。', numel(t), M);
    end

    M_fd = min(M, cfg.M_fd_max);
    t_fd = t(1:M_fd);
    qd_ref = dataset.qd(1:M_fd, :);
    qdd_ref = dataset.qdd(1:M_fd, :);
    q_ref_fd = dataset.q(1:M_fd, :);
    tau_meas = dataset.tau; % 全程用于 ID

    % -------- 构造 4 组参数 --------
    [pi_all_cad, pi_all_id, pi_mix_14id_56cad, pi_mix_14cad_56id] = ...
        build_pi_mixed_schemeA(pi_cad, pi_id_best, cfg.block_dim);

    pi_groups = struct();
    pi_groups.all_cad = pi_all_cad;
    pi_groups.all_id = pi_all_id;
    pi_groups.mix_14id_56cad = pi_mix_14id_56cad;
    pi_groups.mix_14cad_56id = pi_mix_14cad_56id;

    group_names = fieldnames(pi_groups);

    % -------- 逐组评估 --------
    out = struct();
    out.pi_groups = pi_groups;
    out.id_rmse = struct();
    out.fd_qdd_rmse = struct();
    out.fd_qd_rmse_H = struct();
    out.fd_qd_rmse_integral = struct();
    out.fd_q_rmse_integral = struct();

    for gi = 1:numel(group_names)
        gname = group_names{gi};
        pi_vec = pi_groups.(gname);
        if cfg.verbose
            fprintf('\n[validate_mixed_params_schemeA] 评估组 %s ...\n', gname);
        end

        res = eval_one_pi_group(dataset, tau_meas, t_fd, qd_ref, qdd_ref, q_ref_fd, limb, para_order, pi_vec, cfg);
        out.id_rmse.(gname) = res.id_rmse;
        out.fd_qdd_rmse.(gname) = res.fd_qdd_rmse;
        out.fd_qd_rmse_H.(gname) = res.fd_qd_rmse_H;
        out.fd_qd_rmse_integral.(gname) = res.fd_qd_rmse_integral;
        out.fd_q_rmse_integral.(gname) = res.fd_q_rmse_integral;

        if cfg.verbose
            fprintf('  ID  RMSE :'); fprintf(' %.3f', out.id_rmse.(gname)); fprintf('\n');
            fprintf('  FD  qdd  :'); fprintf(' %.3f', out.fd_qdd_rmse.(gname)); fprintf('\n');
            fprintf('  H   qd   :'); fprintf(' %.3f', out.fd_qd_rmse_H.(gname)); fprintf('\n');
            fprintf('  Int qd   :'); fprintf(' %.3f', out.fd_qd_rmse_integral.(gname)); fprintf('\n');
            fprintf('  Int q    :'); fprintf(' %.3f', out.fd_q_rmse_integral.(gname)); fprintf('\n');
        end
    end
end

function res = eval_one_pi_group(dataset, tau_meas, t_fd, qd_ref, qdd_ref, q_ref_fd, limb, para_order, pi_vec, cfg)
    [M, n] = size(dataset.q);
    M_fd = numel(t_fd);
    forward_opts = cfg.forward_opts;

    % 使用 full 参数模型（决定了“混合参数方案”的本质口径）
    model_full = struct('type', 'full', 'limb', limb, 'para_order', para_order, 'pi_vec', pi_vec);

    % -------- 1) ID RMSE（vs tau_meas） --------
    if cfg.compute_id
        tau_pred = zeros(M, n);
        for k = 1:M
            tau_pred(k, :) = inverse_dynamics_dispatch(dataset.q(k, :), dataset.qd(k, :), dataset.qdd(k, :), model_full).';
        end
        res.id_rmse = sqrt(mean((tau_pred - tau_meas).^2, 1));
    else
        res.id_rmse = nan(1, n);
    end

    % -------- 2) FD：先得到 qdd_pred --------
    if cfg.compute_fd
        qdd_pred = zeros(M_fd, n);
        for k = 1:M_fd
            % 正动力学输入：q, qd, tau
            qdd_pred(k, :) = forward_dynamics_dispatch(dataset.q(k, :), dataset.qd(k, :), dataset.tau(k, :), model_full, forward_opts).';
        end

        % FD RMSE（vs qdd_ref）
        res.fd_qdd_rmse = sqrt(mean((qdd_pred - qdd_ref).^2, 1));

        % H=cfg.H_steps 串联预测 qd RMSE
        H = cfg.H_steps;
        qd_chain = build_qd_chain_from_measured_qd(t_fd, qd_ref, qdd_pred, H);
        res.fd_qd_rmse_H = sqrt(mean((qd_chain - qd_ref).^2, 1));

        % 全程积分得到 qd/q，并计算积分轨迹 RMSE
        qd_int = integrate_qd_from_qdd_local(t_fd, qd_ref(1, :), qdd_pred);
        res.fd_qd_rmse_integral = sqrt(mean((qd_int - qd_ref).^2, 1));

        q_int = integrate_q_from_qd_local(t_fd, q_ref_fd(1, :), qd_int, q_ref_fd);
        res.fd_q_rmse_integral = sqrt(mean((q_int - q_ref_fd).^2, 1));
    else
        res.fd_qdd_rmse = nan(1, n);
        res.fd_qd_rmse_H = nan(1, n);
        res.fd_qd_rmse_integral = nan(1, n);
        res.fd_q_rmse_integral = nan(1, n);
    end
end

function [pi_all_cad, pi_all_id, pi_mix_14id_56cad, pi_mix_14cad_56id] = build_pi_mixed_schemeA(pi_cad, pi_id_best, block_dim)
    % joint1 -> 1:10, joint2 -> 11:20, ... joint6 -> 51:60
    % 组 3：1~4 辨识，5~6 CAD
    % 组 4：1~4 CAD，5~6 辨识
    cut_14 = 4 * block_dim;
    pi_all_cad = pi_cad;
    pi_all_id = pi_id_best;

    pi_mix_14id_56cad = pi_cad;
    pi_mix_14id_56cad(1:cut_14) = pi_id_best(1:cut_14);

    pi_mix_14cad_56id = pi_id_best;
    pi_mix_14cad_56id(1:cut_14) = pi_cad(1:cut_14);
end

function qd_out = build_qd_chain_from_measured_qd(t_vec, qd_meas, qdd_model, H)
% H 步串联预测 qd：前 H 行与测量一致
% qd_pred(k) = qd_meas(k-H) + sum_{j=0..H-1} dt(s+j)*qdd_model(s+j), s=k-H
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

function qd_out = integrate_qd_from_qdd_local(t_vec, qd0, qdd_in)
% qd_out(k)=qd_out(k-1)+dt(k-1)*qdd_in(k-1)
    Mloc = size(qdd_in, 1);
    nloc = size(qdd_in, 2);
    qd_out = zeros(Mloc, nloc);
    if Mloc <= 0, return; end
    qd_out(1, :) = qd0;
    if Mloc == 1, return; end
    dt_med = median(diff(t_vec));
    if dt_med <= 0 || isnan(dt_med), dt_med = 0.002; end
    for kk = 2:Mloc
        dt_k = t_vec(kk) - t_vec(kk-1);
        if dt_k <= 0 || isnan(dt_k), dt_k = dt_med; end
        qd_out(kk, :) = qd_out(kk-1, :) + dt_k * qdd_in(kk-1, :);
    end
end

function q_out = integrate_q_from_qd_local(t_vec, q0, qd_in, q_anchor)
% q_out(k)=q_anchor(k-1)+dt(k-1)*qd_in(k-1)
    Mloc = size(qd_in, 1);
    nloc = size(qd_in, 2);
    q_out = zeros(Mloc, nloc);
    if Mloc <= 0, return; end
    q_out(1, :) = q0;
    if Mloc == 1, return; end
    dt_med = median(diff(t_vec));
    if dt_med <= 0 || isnan(dt_med), dt_med = 0.002; end
    for kk = 2:Mloc
        dt_k = t_vec(kk) - t_vec(kk-1);
        if dt_k <= 0 || isnan(dt_k), dt_k = dt_med; end
        if nargin >= 4 && ~isempty(q_anchor)
            q_out(kk, :) = q_anchor(kk-1, :) + dt_k * qd_in(kk-1, :);
        else
            q_out(kk, :) = q_out(kk-1, :) + dt_k * qd_in(kk-1, :);
        end
    end
end

function s = set_default(s, field, default_v)
    if ~isfield(s, field) || isempty(s.(field))
        s.(field) = default_v;
    end
end

