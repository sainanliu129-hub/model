function [t, q_ref, qd_ref, qdd_ref, report] = generate_multisine_joint_traj(opts)
% generate_multisine_joint_traj  生成 6 关节 multisine 参考轨迹并校验限位
%
% 目标形式：
%   q_j(t) = q0_j + sum_{k=1..K} A_j,k * sin(2*pi*f_k*t + phi_j,k)
% 导数：
%   qd_j(t)  = sum A_j,k * (2*pi*f_k) * cos(2*pi*f_k*t + phi_j,k)
%   qdd_j(t) = -sum A_j,k * (2*pi*f_k)^2 * sin(2*pi*f_k*t + phi_j,k)
%
% 默认配置（对齐 TLBO_E1 / validate_trajectory_E1 的“校验逻辑与限位参数”）：
%   freqs    = [0.15, 0.35, 0.55]  (Hz)
%   amp_scale= [0.20, 0.18, 0.18, 0.15, 0.10, 0.10] (rad) 作为 A_j,k
%   phase    = rand(6, K) * 2*pi
%   duration = 25s, sample_frequency = 500Hz (dt=0.002s，和项目内 dt 兜底一致)
%
% 输出：
%   t       : N×1
%   q_ref   : N×6
%   qd_ref  : N×6
%   qdd_ref : N×6
%   report  : 超限检查结果与峰值统计
%
% 用法示例：
%   [t,q,qd,qdd,report] = generate_multisine_joint_traj();
%   opts.duration = 30;
%   opts.sample_frequency = 200;
%   opts.rng_seed = 0;  % 固定随机相位
%   opts.export_csv_path = 'multisine_left_leg.csv'; % 导出为激励轨迹同款 CSV（time+12关节）
%   % 默认另一腿安全位与激励轨迹流程一致：
%   % left_leg:  [0; -0.436; 0; 0; 0; 0]  (right roll lower)
%   % right_leg: [0;  0.436; 0; 0; 0; 0]  (left  roll upper)
%   generate_multisine_joint_traj(opts);
%
% 兼容说明：本文件为纯数学生成器；不依赖机器人工具箱。

if nargin < 1 || isempty(opts)
    opts = struct();
end

% ---------------- Defaults ----------------
J = 6; %#ok<NASGU> % 关节数（默认 6）
if ~isfield(opts, 'duration') || isempty(opts.duration), opts.duration = 25; end
if ~isfield(opts, 'sample_frequency') || isempty(opts.sample_frequency), opts.sample_frequency = 500; end
if ~isfield(opts, 'freqs') || isempty(opts.freqs), opts.freqs = [0.15, 0.35, 0.55]; end
K = numel(opts.freqs);
if ~isfield(opts, 'amp_scale') || isempty(opts.amp_scale)
    opts.amp_scale = [0.20, 0.18, 0.18, 0.15, 0.10, 0.10]';
end
if ~isfield(opts, 'rng_seed') || isempty(opts.rng_seed)
    opts.rng_seed = 1; % 默认固定，方便复现与比对
end
if ~isfield(opts, 'phase') || isempty(opts.phase)
    rng(opts.rng_seed);
    opts.phase = rand(J, K) * 2*pi;
end
if ~isfield(opts, 'q_min') || isempty(opts.q_min)
    % TLBO_E1（DOF=6, LARGE_EXCITATION=false）名义位置紧限位（rad）
    opts.q_min = [-0.61; -0.261; -2.09; 0; -0.872; -0.262];
end
if ~isfield(opts, 'q_max') || isempty(opts.q_max)
    opts.q_max = [0.61; 0.436; 0.785; 2.44; 0.523; 0.262];
end
if ~isfield(opts, 'qd_max') || isempty(opts.qd_max)
    % TLBO_E1：dq_max = [16.75; 20.1; 20.1; 13.18; 12.46; 12.46].*0.9
    opts.qd_max = ([16.75; 20.1; 20.1; 13.18; 12.46; 12.46] * 0.9); % rad/s
end
if ~isfield(opts, 'qdd_max') || isempty(opts.qdd_max)
    % TLBO_E1：ddq_max = [200;200;400;400;200;200] （rad/s^2）
    opts.qdd_max = [200; 200; 400; 400; 200; 200]; % rad/s^2
end
if ~isfield(opts, 'do_plot') || isempty(opts.do_plot), opts.do_plot = true; end
if ~isfield(opts, 'do_animate') || isempty(opts.do_animate), opts.do_animate = false; end
if ~isfield(opts, 'animate_stride') || isempty(opts.animate_stride), opts.animate_stride = 200; end
if ~isfield(opts, 'do_plot_xyz') || isempty(opts.do_plot_xyz), opts.do_plot_xyz = true; end
if ~isfield(opts, 'tol_pos') || isempty(opts.tol_pos), opts.tol_pos = 1e-6; end
if ~isfield(opts, 'tol_vel') || isempty(opts.tol_vel), opts.tol_vel = 1e-6; end
if ~isfield(opts, 'tol_acc') || isempty(opts.tol_acc), opts.tol_acc = 1e-6; end
if ~isfield(opts, 'limb') || isempty(opts.limb), opts.limb = 'left_leg'; end
if ~isfield(opts, 'enable_collision_check') || isempty(opts.enable_collision_check)
    opts.enable_collision_check = true;
end
if ~isfield(opts, 'enable_torque_check') || isempty(opts.enable_torque_check)
    opts.enable_torque_check = true;
end
if ~isfield(opts, 'relax_vel') || isempty(opts.relax_vel), opts.relax_vel = 1.0; end
if ~isfield(opts, 'relax_acc') || isempty(opts.relax_acc), opts.relax_acc = 1.0; end
if ~isfield(opts, 'torque_validate_ratio') || isempty(opts.torque_validate_ratio)
    opts.torque_validate_ratio = 0.9;
end
if ~isfield(opts, 'max_effort') || isempty(opts.max_effort)
    opts.max_effort = [];
end
if ~isfield(opts, 'validate_max_points') || isempty(opts.validate_max_points)
    opts.validate_max_points = 400;
end
if ~isfield(opts, 'auto_scale_position') || isempty(opts.auto_scale_position)
    opts.auto_scale_position = true; % 自动缩放到位置限位内（保持关节幅值比例）
end
if ~isfield(opts, 'auto_scale_margin') || isempty(opts.auto_scale_margin)
    opts.auto_scale_margin = 1e-9; % 缩放后再留一点数值裕度，避免贴边因数值误差超限
end
if ~isfield(opts, 'export_csv_path') || isempty(opts.export_csv_path)
    opts.export_csv_path = '';
end
if ~isfield(opts, 'export_limb') || isempty(opts.export_limb)
    opts.export_limb = opts.limb; % 与校验 limb 保持一致
end
if ~isfield(opts, 'export_other_leg_safe_6') || isempty(opts.export_other_leg_safe_6)
    % 与激励轨迹流程保持一致：
    % 左腿激励 => 右腿 roll 取 lower；右腿激励 => 左腿 roll 取 upper
    opts.export_other_leg_safe_6 = local_default_other_leg_safe_6(opts.export_limb);
end

% ---------------- Validate sizes ----------------
freqs = opts.freqs(:).';
K = numel(freqs);
amp_scale = opts.amp_scale(:);
phase = opts.phase;

q_min = opts.q_min(:);
q_max = opts.q_max(:);
qd_max = opts.qd_max(:);
qdd_max = opts.qdd_max(:);

if numel(amp_scale) ~= J
    error('amp_scale 长度须为 %d（当前为 %d）', J, numel(amp_scale));
end
if ~isequal(size(phase), [J, K])
    error('phase 尺寸须为 %d×%d（当前为 %dx%d）', J, K, size(phase,1), size(phase,2));
end
if ~isequal(numel(q_min), J) || ~isequal(numel(q_max), J)
    error('q_min/q_max 长度须为 %d', J);
end
if ~isequal(numel(qd_max), J) || ~isequal(numel(qdd_max), J)
    error('qd_max/qdd_max 长度须为 %d', J);
end

% q0 取限位中点（用户若想自定义，可在 opts.q0 覆盖）
if isfield(opts, 'q0') && ~isempty(opts.q0)
    q0 = opts.q0(:);
    if numel(q0) ~= J, error('q0 长度须为 %d', J); end
else
    q0 = (q_min + q_max) / 2;
end

% ---------------- Time grid ----------------
fs = opts.sample_frequency;
if fs <= 0 || ~isfinite(fs)
    error('sample_frequency 必须是正数');
end
T = opts.duration;
if T <= 0 || ~isfinite(T)
    error('duration 必须是正数');
end

dt = 1 / fs;
N = floor(T / dt) + 1;
t = (0:N-1)' * dt; % N×1, t(end) ≈ T

% ---------------- Multisine synthesis ----------------
q_ref = zeros(N, J);
qd_ref = zeros(N, J);
qdd_ref = zeros(N, J);

for j = 1:J
    A_j = amp_scale(j);
    q_ref(:, j) = q0(j);
    for k = 1:K
        w = 2 * pi * freqs(k);
        phi = phase(j, k);
        ang = w * t + phi; % N×1
        s = sin(ang);
        c = cos(ang);
        q_ref(:, j) = q_ref(:, j) + A_j * s;
        qd_ref(:, j) = qd_ref(:, j) + A_j * w * c;
        qdd_ref(:, j) = qdd_ref(:, j) - A_j * (w^2) * s;
    end
end

% ---------------- Optional: auto scale position ----------------
if opts.auto_scale_position
    % q_ref = q0 + delta_q，其中 delta_q 与 amp_scale 成正比；
    % 因此通过缩放 amp_scale 可线性保证“validate_trajectory_E1 的位置校核区间”不超限。
    center_pos = (q_max.' + q_min.') / 2;
    full_range_pos = (q_max.' - q_min.');
    pos_valid_upper = center_pos + 0.45 * full_range_pos;
    pos_valid_lower = center_pos - 0.45 * full_range_pos;

    delta_q = q_ref - q0.'; % N×J

    % 每个关节分别计算最大允许缩放比例，让该关节的 q_ref 都尽量贴到 pos_valid 边界
    % 这样不会被“某个最小限位关节”拖累其它关节的幅值。
    scale_vec = ones(J, 1);
    for j = 1:J
        allowed_pos = pos_valid_upper(j) - q0(j);
        allowed_neg = q0(j) - pos_valid_lower(j);
        max_pos = max(delta_q(:, j));
        max_neg = min(delta_q(:, j));

        s_pos = inf; s_neg = inf;
        if max_pos > 0
            s_pos = allowed_pos / max_pos;
        end
        if max_neg < 0
            s_neg = allowed_neg / (-max_neg);
        end

        s_j = min(s_pos, s_neg);
        if isfinite(s_j) && s_j > 0
            scale_vec(j) = s_j * (1 - opts.auto_scale_margin);
        end
    end

    if any(abs(scale_vec - 1) > 1e-12)
        fprintf('[Multisine Generator] auto_scale_position: per-joint scale = [');
        fprintf(' %.6g', scale_vec);
        fprintf(' ]\n');
        amp_scale = amp_scale .* scale_vec;

        % 重新合成（qd/qdd 会随 amp_scale 同比例放大/缩小）
        q_ref(:) = 0; qd_ref(:) = 0; qdd_ref(:) = 0;
        for j = 1:J
            A_j = amp_scale(j);
            q_ref(:, j) = q0(j);
            for k = 1:K
                w = 2 * pi * freqs(k);
                phi = phase(j, k);
                ang = w * t + phi;
                s = sin(ang);
                c = cos(ang);
                q_ref(:, j) = q_ref(:, j) + A_j * s;
                qd_ref(:, j) = qd_ref(:, j) + A_j * w * c;
                qdd_ref(:, j) = qdd_ref(:, j) - A_j * (w^2) * s;
            end
        end
    end
end

% ---------------- Limit checks ----------------
report = struct();
report.ok = true;
report.violations = struct('type', {}, 't', {}, 'sample_idx', {}, 'joint_idx', {}, ...
    'value', {}, 'limit', {}, 'excess', {});

max_q = max(q_ref, [], 1);
min_q = min(q_ref, [], 1);
max_abs_qd = max(abs(qd_ref), [], 1);
max_abs_qdd = max(abs(qdd_ref), [], 1);

report.max_q = max_q;
report.min_q = min_q;
report.max_abs_qd = max_abs_qd;
report.max_abs_qdd = max_abs_qdd;
center_pos = (q_max.' + q_min.') / 2;
full_range_pos = (q_max.' - q_min.');
pos_valid_upper = center_pos + 0.45 * full_range_pos;
pos_valid_lower = center_pos - 0.45 * full_range_pos;

report.pos_margin_upper = (pos_valid_upper - max_q);   % 正数表示有裕度
report.pos_margin_lower = (min_q - pos_valid_lower);   % 正数表示有裕度
report.vel_margin = (qd_max.' - max_abs_qd);   % 正数表示有裕度
report.acc_margin = (qdd_max.' - max_abs_qdd); % 正数表示有裕度

addViolation = @(type, sample_idx, joint_idx, value, limit) local_add_violation(...
    type, sample_idx, joint_idx, value, limit, t, report.violations);

tol_pos = opts.tol_pos;
tol_vel = opts.tol_vel;
tol_acc = opts.tol_acc;

% 复用 validate_trajectory_E1 的校验逻辑：最多检查 validate_max_points 个离散点
tol_pos_validate = 1e-4;   % 与 validate_trajectory_E1.m 一致
idx_validate = round(linspace(1, N, min(opts.validate_max_points, N)));

% 可选：准备碰撞与扭矩所需的 robot_limb / max_effort
robot_limb = [];
max_effort = opts.max_effort;

% 尝试加载 Body_GravityPara_Iden 路径，以便复用 get_e1_limb_robot / inverseDynamics / checkCollision 等
if (opts.enable_collision_check || opts.enable_torque_check)
    try
        this_file = mfilename('fullpath');
        this_dir = fileparts(this_file);
        principle_dir = fullfile(this_dir, '..', 'applications', 'Body_GravityPara_Iden', 'principle');
        if exist('ensure_body_gravity_para_iden_path', 'file') ~= 2
            addpath(principle_dir);
        end
        if exist('ensure_body_gravity_para_iden_path', 'file') == 2
            ensure_body_gravity_para_iden_path();
        end
    catch
    end
end

if opts.enable_collision_check
    if exist('get_e1_limb_robot', 'file') == 2 && exist('checkCollision', 'file') == 2
        try
            [robot_limb, dof_robot] = get_e1_limb_robot(opts.limb); %#ok<ASGLU>
        catch
            robot_limb = [];
        end
    end
end

if opts.enable_torque_check
    if isempty(max_effort)
        % 复用 TLBO_E1：从 URDF 读 max_effort；失败则用 E1 默认值兜底
        E1_LEG_EFFORT_DEFAULT = [110; 170; 170; 330; 70; 70];
        if exist('get_limb_effort_from_urdf', 'file') == 2
            try
                this_file = mfilename('fullpath');
                this_dir = fileparts(this_file);
                repo_root = fullfile(this_dir, '..');
                urdf_path = fullfile(repo_root, 'noetix_description', 'urdf', 'E1.urdf');
                if ~exist(urdf_path, 'file')
                    urdf_path = fullfile(repo_root, 'urdf', 'E1.urdf');
                end
                if exist('get_e1_limb_robot', 'file') == 2
                    % 确保 robot_limb 也可用于 inverseDynamics（如 enable_torque_check）
                    [robot_limb, dof_robot] = get_e1_limb_robot(opts.limb); %#ok<ASGLU>
                end
                if exist('get_limb_effort_from_urdf', 'file') == 2
                    max_effort = get_limb_effort_from_urdf(urdf_path, opts.limb);
                end
            catch
                max_effort = [];
            end
        end
        if isempty(max_effort)
            max_effort = E1_LEG_EFFORT_DEFAULT;
        end
    end
    % 也确保 robot_limb 存在（inverseDynamics 需要）
    if isempty(robot_limb) && exist('get_e1_limb_robot', 'file') == 2 && exist('inverseDynamics', 'file') == 2
        try
            [robot_limb, dof_robot] = get_e1_limb_robot(opts.limb); %#ok<ASGLU>
        catch
            robot_limb = [];
        end
    end
end

% 执行逐点校核（任一点超限即 ok=false；与 validate_trajectory_E1 一致）
for ii = 1:numel(idx_validate)
    i = idx_validate(ii);
    for j = 1:J
        qj = q_ref(i, j);
        if qj > pos_valid_upper(j) + tol_pos_validate
            report.ok = false;
            v = local_pack_violation('pos_upper', i, j, qj, pos_valid_upper(j), qj - pos_valid_upper(j), t(i));
            report.violations = v;
            report.violations = report.violations; %#ok<NASGU>
            break;
        end
        if qj < pos_valid_lower(j) - tol_pos_validate
            report.ok = false;
            v = local_pack_violation('pos_lower', i, j, qj, pos_valid_lower(j), pos_valid_lower(j) - qj, t(i));
            report.violations = v;
            report.violations = report.violations; %#ok<NASGU>
            break;
        end
    end
    if ~report.ok, break; end

    for j = 1:J
        qdj = qd_ref(i, j);
        if abs(qdj) > qd_max(j) * opts.relax_vel
            report.ok = false;
            limit = qd_max(j) * opts.relax_vel;
            ex = abs(qdj) - limit;
            v = local_pack_violation('vel', i, j, qdj, limit, ex, t(i));
            report.violations = v;
            report.violations = report.violations; %#ok<NASGU>
            break;
        end
    end
    if ~report.ok, break; end

    for j = 1:J
        qddj = qdd_ref(i, j);
        if abs(qddj) > qdd_max(j) * opts.relax_acc
            report.ok = false;
            limit = qdd_max(j) * opts.relax_acc;
            ex = abs(qddj) - limit;
            v = local_pack_violation('acc', i, j, qddj, limit, ex, t(i));
            report.violations = v;
            report.violations = report.violations; %#ok<NASGU>
            break;
        end
    end
    if ~report.ok, break; end

    % 自碰撞
    if opts.enable_collision_check && ~isempty(robot_limb) && exist('checkCollision', 'file') == 2
        try
            cfg = q_ref(i, :); % 行向量
            if checkCollision(robot_limb, cfg)
                report.ok = false;
                v = local_pack_violation('collision', i, 0, NaN, NaN, 1, t(i));
                report.violations = v;
                report.violations = report.violations; %#ok<NASGU>
                break;
            end
        catch
        end
    end
    if ~report.ok, break; end
end

% 扭矩校核（独立于位置/速度/加速度的提前返回，保持 validate_trajectory_E1 的“若已超限则不再算”的简单做法）
if report.ok && opts.enable_torque_check && ~isempty(robot_limb) && ~isempty(max_effort) && exist('inverseDynamics', 'file') == 2
    TORQUE_VALIDATE_RATIO = opts.torque_validate_ratio;
    max_effort = max_effort(:);
    if numel(max_effort) >= J
        max_effort = max_effort(1:J);
        effort_limit = TORQUE_VALIDATE_RATIO * max_effort;
        for ii = 1:numel(idx_validate)
            i = idx_validate(ii);
            qrow = q_ref(i, :);
            qdrow = qd_ref(i, :);
            qddrow = qdd_ref(i, :);
            try
                tau = inverseDynamics(robot_limb, qrow, qdrow, qddrow);
                tau = tau(:);
                for j = 1:J
                    if abs(tau(j)) > effort_limit(j)
                        report.ok = false;
                        ex = abs(tau(j)) - effort_limit(j);
                        v = local_pack_violation('torque', i, j, tau(j), effort_limit(j), ex, t(i));
                        report.violations = v;
                        report.violations = report.violations; %#ok<NASGU>
                        break;
                    end
                end
            catch
            end
            if ~report.ok, break; end
        end
    end
end

% 若 report.violations 只保存了单条结构体，统一成数组形式（保持下方打印代码兼容）
if ~report.ok && isstruct(report.violations) && ~isempty(fieldnames(report.violations))
    report.violations = report.violations(:).';
end

% ---------------- Print summary ----------------
fprintf('\n[Multisine Generator] duration=%.3fs, fs=%.1fHz (dt=%.6fs)\n', t(end), fs, dt);
fprintf('Freqs:'); fprintf(' %.3f', freqs); fprintf(' (Hz)\n');
fprintf('amp_scale (rad):'); fprintf(' %.3f', amp_scale); fprintf('\n');
fprintf('Phase (rad):\n');
for j = 1:J
    fprintf('  j=%d:', j); fprintf(' %.3f', phase(j, :)); fprintf('\n');
end

fprintf('\n[Limit Check] ok=%d (validate_tol_pos=%.1e, relax_vel=%.3f, relax_acc=%.3f)\n', ...
    report.ok, 1e-4, opts.relax_vel, opts.relax_acc);
for j = 1:J
    fprintf('  joint %d: q=[%.4f, %.4f] (margin up=%.4f, low=%.4f), |qd|max=%.4f (lim=%.4f, margin=%.4f), |qdd|max=%.4f (lim=%.4f, margin=%.4f)\n', ...
        j, min_q(j), max_q(j), report.pos_margin_upper(j), report.pos_margin_lower(j), ...
        max_abs_qd(j), qd_max(j), report.vel_margin(j), ...
        max_abs_qdd(j), qdd_max(j), report.acc_margin(j));
end

if ~report.ok && ~isempty(report.violations)
    fprintf('\nFirst violations (up to %d):\n', min(numel(report.violations), 8));
    nshow = min(numel(report.violations), 8);
    for ii = 1:nshow
        v = report.violations(ii);
        fprintf('  %s @ t=%.6f s, joint %d: value=%.6f, limit=%.6f, excess=%.6f\n', ...
            v.type, v.t, v.joint_idx, v.value, v.limit, v.excess);
    end
end

% ---------------- Optional CSV export ----------------
if ~isempty(opts.export_csv_path)
    try
        Q12 = local_pack_single_limb_to_q12(q_ref, opts.export_limb, opts.export_other_leg_safe_6);
        if exist('write_leg_trajectory_csv_plan_format', 'file') == 2
            write_leg_trajectory_csv_plan_format(opts.export_csv_path, t, Q12);
        else
            % 兜底：按激励轨迹一致格式自行写 CSV，避免路径缺失时导出失败
            local_write_plan_format_csv(opts.export_csv_path, t, Q12);
        end
        report.export_csv_path = opts.export_csv_path;
        report.export_csv_ok = true;
        fprintf('[Multisine Generator] CSV exported: %s\n', opts.export_csv_path);
    catch ME
        report.export_csv_path = opts.export_csv_path;
        report.export_csv_ok = false;
        report.export_csv_error = ME.message;
        warning('CSV export failed: %s', ME.message);
    end
end

% ---------------- Plot ----------------
if opts.do_plot
    joint_legend = arrayfun(@(j) sprintf('q%d', j), 1:J, 'UniformOutput', false);
    colors = lines(J);

    % q_ref
    figure('Name', 'Multisine Generator - q / qd / qdd', 'Position', [80, 80, 1200, 800]);
    ax1 = subplot(3,1,1);
    hold(ax1, 'on'); grid(ax1, 'on');

    % 限位（与 validate_trajectory_E1 一致：位置校核半宽=0.45×完整范围）
    center_pos = (q_max + q_min) / 2;
    full_range_pos = (q_max - q_min);
    pos_valid_upper = center_pos + 0.45 * full_range_pos;
    pos_valid_lower = center_pos - 0.45 * full_range_pos;
    nom_color = [0.6, 0.6, 0.6];

    for j = 1:J
        plot(ax1, t, q_ref(:, j), 'LineWidth', 1.2, 'Color', colors(j, :));
        % validate 限位：用同色虚线贴近曲线
        plot(ax1, [t(1), t(end)], [pos_valid_upper(j), pos_valid_upper(j)], '--', ...
            'Color', colors(j, :), 'LineWidth', 1.0, 'HandleVisibility', 'off');
        plot(ax1, [t(1), t(end)], [pos_valid_lower(j), pos_valid_lower(j)], '--', ...
            'Color', colors(j, :), 'LineWidth', 1.0, 'HandleVisibility', 'off');
        % 名义限位：灰色点线（更宽，供对照）
        plot(ax1, [t(1), t(end)], [q_max(j), q_max(j)], ':', ...
            'Color', nom_color, 'LineWidth', 0.9, 'HandleVisibility', 'off');
        plot(ax1, [t(1), t(end)], [q_min(j), q_min(j)], ':', ...
            'Color', nom_color, 'LineWidth', 0.9, 'HandleVisibility', 'off');
    end
    ylabel(ax1, 'q (rad)');
    title(ax1, 'q\_ref (limits: nom q_min/q_max :, validate 0.45 --)');

    ax2 = subplot(3,1,2);
    hold(ax2, 'on'); grid(ax2, 'on');
    for j = 1:J
        plot(ax2, t, qd_ref(:, j), 'LineWidth', 1.2, 'Color', colors(j, :));
    end
    ylabel(ax2, 'qd (rad/s)');
    title(ax2, 'qd\_ref');

    ax3 = subplot(3,1,3);
    hold(ax3, 'on'); grid(ax3, 'on');
    for j = 1:J
        plot(ax3, t, qdd_ref(:, j), 'LineWidth', 1.2, 'Color', colors(j, :));
    end
    ylabel(ax3, 'qdd (rad/s^2)');
    title(ax3, 'qdd\_ref');
    xlabel(ax3, 't (s)');

    % 只在最上面的图放 legend，避免遮挡
    try
        legend(ax1, joint_legend, 'Location', 'eastoutside');
    catch
        % 部分旧版本 MATLAB 对 legend 不同对象支持不一致
        legend(joint_legend);
    end

    if opts.do_animate
        animate_cursor(ax1, ax2, ax3, t, q_ref, qd_ref, qdd_ref, opts.animate_stride);
    end

    % -------- 3D end-effector trajectory (xyz) --------
    if opts.do_plot_xyz && exist('getTransform', 'file') == 2
        try
            % 如果前面没启用碰撞/扭矩，robot_limb 可能为空；这里尝试加载
            robot_limb_xyz = robot_limb;
            if isempty(robot_limb_xyz) && exist('get_e1_limb_robot', 'file') == 2
                try
                    [robot_limb_xyz, ~] = get_e1_limb_robot(opts.limb);
                catch
                    robot_limb_xyz = [];
                end
            end

            if ~isempty(robot_limb_xyz)
                baseName  = robot_limb_xyz.BaseName;
                end_body  = robot_limb_xyz.Bodies{robot_limb_xyz.NumBodies}.Name;
                Nxyz = size(q_ref, 1);
                xyz = zeros(Nxyz, 3);
                for ii = 1:Nxyz
                    cfg = q_ref(ii, :);
                    if iscolumn(cfg), cfg = cfg'; end
                    T = getTransform(robot_limb_xyz, cfg, end_body, baseName);
                    xyz(ii, :) = T(1:3, 4)';
                end

                figure('Name', 'Multisine - end-effector 3D xyz', 'Position', [150, 150, 900, 650]);
                plot3(xyz(:,1), xyz(:,2), xyz(:,3), 'b-', 'LineWidth', 1.8);
                hold on; grid on;
                plot3(xyz(1,1), xyz(1,2), xyz(1,3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
                plot3(xyz(end,1), xyz(end,2), xyz(end,3), 'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
                xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
                title('Multisine end-effector trajectory (3D xyz)');
                view(45, 25);
                legend({'trajectory','start','end'}, 'Location','best');
                hold off;
            end
        catch ME
            warning('3D xyz plot failed: %s', ME.message);
        end
    end
end

end % main

% ---------------- Helpers ----------------
function v = local_pack_violation(type, sample_idx, joint_idx, value, limit, excess, t_cur)
v = struct();
v.type = type;
v.t = t_cur;
v.sample_idx = sample_idx;
v.joint_idx = joint_idx;
v.value = value;
v.limit = limit;
v.excess = excess;
end

function animate_cursor(ax1, ax2, ax3, t, q_ref, qd_ref, qdd_ref, stride)
% 在 3 个子图中移动竖线游标，便于肉眼检查峰值出现时刻
J = size(q_ref, 2);
if stride < 1, stride = 1; end

% 先固定轴范围（否则 drawnow 会改变）
yl1 = [min(q_ref(:)), max(q_ref(:))];
yl2 = [min(qd_ref(:)), max(qd_ref(:))];
yl3 = [min(qdd_ref(:)), max(qdd_ref(:))];

cursor1 = plot(ax1, [t(1), t(1)], yl1, 'k--', 'LineWidth', 1.0);
cursor2 = plot(ax2, [t(1), t(1)], yl2, 'k--', 'LineWidth', 1.0);
cursor3 = plot(ax3, [t(1), t(1)], yl3, 'k--', 'LineWidth', 1.0);

txt = text(ax1, 0.01, 0.95, '', 'Units', 'normalized', 'FontSize', 10, 'BackgroundColor', 'w');
for i = 1:stride:numel(t)
    tt = t(i);
    set(cursor1, 'XData', [tt, tt]);
    set(cursor2, 'XData', [tt, tt]);
    set(cursor3, 'XData', [tt, tt]);
    set(txt, 'String', sprintf('t=%.3fs', tt));
    drawnow;
end
% 最后停留在末尾
tt = t(end);
set(cursor1, 'XData', [tt, tt]);
set(cursor2, 'XData', [tt, tt]);
set(cursor3, 'XData', [tt, tt]);
set(txt, 'String', sprintf('t=%.3fs (end)', tt));
drawnow;
end

function violations = local_add_violation(type, sample_idx, joint_idx, value, limit, t, violations) %#ok<DEFNU>
% 目前未使用：保留给后续扩展（可将本文件改成“记录所有超限点”）
excess = NaN;
if any(strcmp(type, {'pos_upper', 'pos_lower', 'vel', 'acc'}))
    excess = abs(value) - abs(limit);
end
v = struct('type', type, 't', t(sample_idx), 'sample_idx', sample_idx, 'joint_idx', joint_idx, ...
    'value', value, 'limit', limit, 'excess', excess);
violations(end+1) = v; %#ok<AGROW>
end

function Q12 = local_pack_single_limb_to_q12(q_ref, limb, other_leg_safe_6)
% 将单腿 6 轴轨迹打包成 plan/excitation 同款 12 轴列顺序：
% [leg_l1..leg_l6, leg_r1..leg_r6]
if size(q_ref, 2) ~= 6
    error('q_ref 需为 N×6');
end
safe6 = other_leg_safe_6(:).';
if numel(safe6) ~= 6
    error('export_other_leg_safe_6 长度需为 6');
end
N = size(q_ref, 1);
Q12 = zeros(N, 12);
limb_l = lower(strtrim(limb));
if contains(limb_l, 'left')
    Q12(:, 1:6) = q_ref;
    Q12(:, 7:12) = repmat(safe6, N, 1);
elseif contains(limb_l, 'right')
    Q12(:, 1:6) = repmat(safe6, N, 1);
    Q12(:, 7:12) = q_ref;
else
    error('export_limb 需包含 "left" 或 "right"');
end
end

function local_write_plan_format_csv(csv_path, t, Q12)
% 与 write_leg_trajectory_csv_plan_format 保持一致：time + 12 joints
t = t(:);
N = numel(t);
if size(Q12, 1) ~= N || size(Q12, 2) ~= 12
    error('Q12 须为 %d×12（与 t 长度一致）', N);
end

function safe6 = local_default_other_leg_safe_6(export_limb)
% 与 applications/Excitation_Trajectory_E1/excitation_trajectory 一致
% E1_ROLL_SAFE_RIGHT_LOWER = -0.436; E1_ROLL_SAFE_LEFT_UPPER = 0.436
limb_l = lower(strtrim(export_limb));
if contains(limb_l, 'left')
    safe6 = [0; -0.436; 0; 0; 0; 0];
elseif contains(limb_l, 'right')
    safe6 = [0; 0.436; 0; 0; 0; 0];
else
    error('export_limb 需包含 "left" 或 "right"');
end
end
header_names = {'time', 'leg_l1_joint', 'leg_l2_joint', 'leg_l3_joint', 'leg_l4_joint', 'leg_l5_joint', 'leg_l6_joint', ...
    'leg_r1_joint', 'leg_r2_joint', 'leg_r3_joint', 'leg_r4_joint', 'leg_r5_joint', 'leg_r6_joint'};
fid = fopen(csv_path, 'w');
if fid == -1
    error('无法创建文件: %s', csv_path);
end
cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s', header_names{1});
for c = 2:numel(header_names)
    fprintf(fid, ',%s', header_names{c});
end
fprintf(fid, '\n');
for i = 1:N
    fprintf(fid, '%.5f', t(i));
    for j = 1:12
        fprintf(fid, ',%.6f', Q12(i, j));
    end
    fprintf(fid, '\n');
end
end

