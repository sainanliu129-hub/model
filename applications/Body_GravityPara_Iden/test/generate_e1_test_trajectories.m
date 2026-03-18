function traj = generate_e1_test_trajectories(varargin)
% generate_e1_test_trajectories  基于 noetix_description/urdf 下的 URDF 生成用于动力学/重力验证的测试轨迹
%
% 功能：
%   1) 静止 50 个姿态（单关节变化），其余关节保持在安全位
%   2) 单关节匀速往返轨迹：速度 = 0.15/0.5/0.9 × v_max_global，
%      v_max_global = min(12 个关节的 URDF 速度限位)，保证所有关节用同一速度基准；
%      便于后续按单关节往返进行对齐分析（例如 compare_single_joint_torque_aligned）
%
% 用法示例：
%   traj = generate_e1_test_trajectories();
%   traj = generate_e1_test_trajectories('dt', 0.002, 'n_static', 50);
%   traj = generate_e1_test_trajectories('save_mat', 'e1_test_traj.mat');
%   traj = generate_e1_test_trajectories('robot_name', 'E1_update');  % 仅需指定机器人名称
%
% 输出 traj 结构体主要字段：
%   traj.joint_names       - 1x12 关节名（leg_l1_joint..leg_l6_joint, leg_r1_joint..leg_r6_joint）
%   traj.pos_limits        - 12x2 关节位置限位 [lower, upper] (rad)
%   traj.vel_limits        - 12x1 最大关节速度 (rad/s)，来自 URDF limit.velocity
%   traj.effort_limits     - 12x1 关节峰值扭矩 (N·m)，来自 URDF limit.effort
%   traj.acc_limits        - 12x1 轨迹规划用关节最大加速度近似 (rad/s^2)
%   traj.q_safe            - 1x12 安全位姿（中间值或 0 在限位内）
%
%   traj.static_poses{j}   - n_static x 12，第 j 个关节作为主动关节的静止姿态集合
%
%   traj.const_vel(j).v_list           - 1xNv 速度列表（默认 [0.15 0.5 0.9]）
%   traj.const_vel(j).segments(k).v    - 该段目标恒定速度 (rad/s)
%   traj.const_vel(j).segments(k).time - 列向量时间轴 (s)
%   traj.const_vel(j).segments(k).q    - n x 12 位置轨迹（单关节往返，其余关节：左腿动时右腿 roll=lower，右腿动时左腿 roll=upper）
%
% 说明：
%   - 本函数只生成「名义关节空间轨迹」，不负责下发到真实机器人或仿真器。
%   - 若需用于对齐分析，可在仿真/控制侧按本名义轨迹生成实际采样数据，并在 CSV 旁边
%     额外保存 meta（例如只保存主动关节 q_act），再由 compare_single_joint_torque_aligned 读取。

p = inputParser;
addParameter(p, 'urdf_path', '', @ischar);              % 如果给定则直接使用该 URDF 路径
addParameter(p, 'robot_name', 'E1_update', @ischar);    % 默认使用 noetix_description/urdf/E1_update.urdf
addParameter(p, 'dt', 0.002, @isnumeric);               % 采样周期
addParameter(p, 'n_static', 50, @isnumeric);            % 每个关节的静止姿态个数
addParameter(p, 'vel_list', [0.15 0.5 0.9], @isnumeric); % 单关节匀速段速度比例（相对 v_max_global）
addParameter(p, 'margin_ratio', 0.95, @isnumeric);      % 使用 95%% 工作空间，避免贴边
addParameter(p, 'static_hold_time', 1.0, @isnumeric);   % 每个静止姿态保持时间 (s)
addParameter(p, 'max_acc', 200, @isnumeric);           % 全局最大加速度上限 (rad/s^2)，用于尽量拉长匀速段
addParameter(p, 'save_mat', '', @ischar);               % 非空则保存 MAT 文件
addParameter(p, 'export_csv', false, @islogical);       % 是否导出 CSV 轨迹文件
addParameter(p, 'export_csv_039', true, @islogical);     % 是否额外导出 0.15×、0.5×、0.9× 各一条轨迹 CSV（每条正序+逆序）
addParameter(p, 'csv_dir', '', @ischar);                % CSV 输出目录（默认当前工作目录）
addParameter(p, 'plot_traj', true, @islogical);         % 是否绘制 q/qd/qdd 曲线
parse(p, varargin{:});
opts = p.Results;

%% 1. 从 URDF 读取腿部关节限位和最大速度
base_dir = fileparts(mfilename('fullpath'));
repo_root = fullfile(base_dir, '..', '..', '..');  % test -> Body_GravityPara_Iden -> applications -> 仓库根

if ~isempty(opts.urdf_path)
    urdf_path = opts.urdf_path;
else
    urdf_path = fullfile(repo_root, 'noetix_description', 'urdf', [opts.robot_name, '.urdf']);
end

if ~exist(urdf_path, 'file')
    error('未找到 URDF 文件: %s', urdf_path);
end

fprintf('===== 生成 E1 测试轨迹（基于 %s）=====\n', urdf_path);

joint_suffix = {'leg_l1_joint','leg_l2_joint','leg_l3_joint','leg_l4_joint','leg_l5_joint','leg_l6_joint', ...
    'leg_r1_joint','leg_r2_joint','leg_r3_joint','leg_r4_joint','leg_r5_joint','leg_r6_joint'};

[pos_limits, vel_limits, effort_limits, leg_mass] = read_leg_joint_limits_from_urdf(urdf_path, joint_suffix);

fprintf('关节位置限位、最大速度与峰值扭矩（来自 URDF）：\n');
for j = 1:numel(joint_suffix)
    fprintf('  %-15s  q:[%7.3f, %7.3f] rad,  v_max: %6.3f rad/s, tau_max: %.3f N·m\n', ...
        joint_suffix{j}, pos_limits(j,1), pos_limits(j,2), vel_limits(j), effort_limits(j));
end

% 关节最大加速度：统一使用全局上限 max_acc（默认 200 rad/s^2）
acc_limits = opts.max_acc * ones(numel(joint_suffix), 1);

% 轨迹速度基准：取 12 个关节中最小最大速度，再按 0.15/0.5/0.9 倍使用
vel_limits_valid = vel_limits(isfinite(vel_limits) & vel_limits > 0);
if isempty(vel_limits_valid)
    v_max_global = 1.0;
    warning('URDF 中无有效速度限位，使用 v_max_global = 1 rad/s。');
else
    v_max_global = min(vel_limits_valid);
end
fprintf('\n关节加速度上限统一设置为 a_max = %.3f rad/s^2。\n', opts.max_acc);
fprintf('轨迹速度基准 v_max_global = min(vel_limits) = %.4f rad/s（0.15/0.5/0.9 倍此值）。\n', v_max_global);

%% 2. 定义安全位姿（与之前测试轨迹一致：左腿动时右腿最外侧，右腿动时左腿最外侧）
% 关节顺序：1~6 左腿(leg_l1..leg_l6)，7~12 右腿(leg_r1..leg_r6)；roll 为第 2/8 关节
q_safe_left  = zeros(1, numel(joint_suffix));  % 左腿运动时：右腿 roll 置为 lower（最外侧）
q_safe_left(8)  = pos_limits(8, 1);
q_safe_right = zeros(1, numel(joint_suffix));  % 右腿运动时：左腿 roll 置为 upper（最外侧）
q_safe_right(2) = pos_limits(2, 2);
% 兼容：汇总用默认（仅用于 traj.q_safe 等）
q_safe = zeros(1, numel(joint_suffix));

fprintf('\n安全位姿（非主动关节，与 plan/generate_multi_speed_trajectory 一致）：\n');
fprintf('  左腿运动时：右腿 roll(leg_r2)=lower=%7.3f rad，其余 0\n', pos_limits(8,1));
fprintf('  右腿运动时：左腿 roll(leg_l2)=upper=%7.3f rad，其余 0\n', pos_limits(2,2));

%% 3. 生成静止姿态集合（单关节，其他关节在安全位）
fprintf('\n--- 生成静止姿态（每个关节 %d 个）---\n', opts.n_static);

static_poses = cell(numel(joint_suffix), 1);

for j = 1:numel(joint_suffix)
    q_low = pos_limits(j,1);
    q_up  = pos_limits(j,2);
    q_range = q_up - q_low;
    q_min_eff = q_low  + (1 - opts.margin_ratio) * 0.5 * q_range;
    q_max_eff = q_up   - (1 - opts.margin_ratio) * 0.5 * q_range;
    if q_min_eff >= q_max_eff
        q_min_eff = q_low;
        q_max_eff = q_up;
    end
    q_list = linspace(q_min_eff, q_max_eff, opts.n_static).';

    if j <= 6
        q_safe_j = q_safe_left;
    else
        q_safe_j = q_safe_right;
    end
    poses_j = repmat(q_safe_j, opts.n_static, 1);
    poses_j(:, j) = q_list;
    static_poses{j} = poses_j;

    fprintf('  %-15s: q in [%7.3f, %7.3f] (linspace, %d 点)\n', ...
        joint_suffix{j}, q_min_eff, q_max_eff, opts.n_static);
end

%% 4. 生成单关节匀速往返轨迹：0→最小→最大→最小→0
%  全程用名义速度 v_nominal_ratio × v_max_global（默认 0.5 倍）
fprintf('\n--- 生成单关节匀速往返轨迹（0→min→max→min→0）---\n');
fprintf('  速度 = 名义比例 × v_max_global（vel_list(2)=%.3f）\n', opts.vel_list(min(2,numel(opts.vel_list))));

dt = opts.dt;
vel_list = opts.vel_list(:).';
v_ratios = [vel_list(1), vel_list(min(2,numel(vel_list))), vel_list(min(3,numel(vel_list)))];
v_nominal_ratio = vel_list(min(2, numel(vel_list)));  % 0→min、min→0 用名义速度

addpath(fullfile(repo_root, 'utility_function'));

const_vel = struct([]);

for j = 1:numel(joint_suffix)
    q_low = pos_limits(j,1);
    q_up  = pos_limits(j,2);
    L = q_up - q_low;
    if L <= 0
        warning('关节 %s 的位置限位不正确 (upper <= lower)，跳过该关节的匀速轨迹。', joint_suffix{j});
        continue;
    end

    a_max = acc_limits(j);
    if ~isfinite(a_max) || a_max <= 0
        a_max = opts.max_acc;
        warning('关节 %s 缺少有效加速度上限，使用全局 max_acc=%.3f rad/s^2。', joint_suffix{j}, a_max);
    end

    % 速度取 v_max_global 的倍数（v_max_global = min(12 关节 vel_limits)）
    v1 = v_nominal_ratio * v_max_global;
    if v1 > 0.99 * v_max_global, v1 = 0.99 * v_max_global; end

    const_vel(j).joint_name = joint_suffix{j};
    const_vel(j).v_list = vel_list;

    % 0→最小→最大→最小→0（四段）
    [t1, q1, qd1, qdd1] = generate_trapezoid_segment(0, q_low, abs(v1), a_max, dt);
    T = t1(end);
    t_act = t1; q_act = q1; qd_act = qd1; qdd_act = qdd1;
    [t2, q2, qd2, qdd2] = generate_trapezoid_segment(q_low, q_up, abs(v1), a_max, dt);
    t_act = [t_act; T + t2(2:end)]; q_act = [q_act; q2(2:end)]; qd_act = [qd_act; qd2(2:end)]; qdd_act = [qdd_act; qdd2(2:end)];
    T = T + t2(end);
    [t3, q3, qd3, qdd3] = generate_trapezoid_segment(q_up, q_low, abs(v1), a_max, dt);
    t_act = [t_act; T + t3(2:end)]; q_act = [q_act; q3(2:end)]; qd_act = [qd_act; qd3(2:end)]; qdd_act = [qdd_act; qdd3(2:end)];
    T = T + t3(end);
    [t4, q4, qd4, qdd4] = generate_trapezoid_segment(q_low, 0, abs(v1), a_max, dt);
    t_act = [t_act; T + t4(2:end)]; q_act = [q_act; q4(2:end)]; qd_act = [qd_act; qd4(2:end)]; qdd_act = [qdd_act; qdd4(2:end)];

    N_full = numel(q_act);
    if j <= 6
        q_safe_j = q_safe_left;
    else
        q_safe_j = q_safe_right;
    end
    q_full = repmat(q_safe_j, N_full, 1);
    q_full(:, j) = q_act;
    qd_full  = zeros(N_full, 12);
    qdd_full = zeros(N_full, 12);
    qd_full(:, j)  = qd_act;
    qdd_full(:, j) = qdd_act;

    seg.time = t_act;
    seg.q    = q_full;
    seg.qd   = qd_full;
    seg.qdd  = qdd_full;
    seg.v_cmd = v1;
    seg.q_start = 0;
    seg.q_end   = 0;
    seg.description = sprintf('joint=%s, 0->min->max->min->0', joint_suffix{j});

    const_vel(j).segments = seg;

    fprintf('  %-15s  0->min->max->min->0, v=%.4f rad/s, 总点数 %d\n', joint_suffix{j}, v1, N_full);
end

% 生成三条独立轨迹：0.15×、0.5×、0.9× v_max_global 各一条（全程单一速度），用于单独导出 CSV
if opts.export_csv && opts.export_csv_039
    v_ratios_single = [0.15 0.5 0.9];
    suffixes = {'_v015', '_v05', '_v09'};
    const_vel_single = cell(1, 3);
    for iv = 1:3
        vr = v_ratios_single(iv);
        v_one = vr * v_max_global;
        if v_one > 0.99 * v_max_global, v_one = 0.99 * v_max_global; end
        const_vel_single{iv} = struct([]);
        fprintf('\n--- 生成 %.1f×v_max_global 轨迹（%s）---\n', vr, suffixes{iv});
        for j = 1:numel(joint_suffix)
            q_low = pos_limits(j,1);
            q_up  = pos_limits(j,2);
            L = q_up - q_low;
            if L <= 0, continue; end
            a_max = acc_limits(j);
            if ~isfinite(a_max) || a_max <= 0, a_max = opts.max_acc; end
            const_vel_single{iv}(j).joint_name = joint_suffix{j};
            const_vel_single{iv}(j).v_list = vr;
            % 0→最小→最大→最小→0（四段）
            [t1, q1, qd1, qdd1] = generate_trapezoid_segment(0, q_low, abs(v_one), a_max, dt);
            T = t1(end);
            t_act = t1; q_act = q1; qd_act = qd1; qdd_act = qdd1;
            [t2, q2, qd2, qdd2] = generate_trapezoid_segment(q_low, q_up, abs(v_one), a_max, dt);
            t_act = [t_act; T + t2(2:end)]; q_act = [q_act; q2(2:end)]; qd_act = [qd_act; qd2(2:end)]; qdd_act = [qdd_act; qdd2(2:end)];
            T = T + t2(end);
            [t3, q3, qd3, qdd3] = generate_trapezoid_segment(q_up, q_low, abs(v_one), a_max, dt);
            t_act = [t_act; T + t3(2:end)]; q_act = [q_act; q3(2:end)]; qd_act = [qd_act; qd3(2:end)]; qdd_act = [qdd_act; qdd3(2:end)];
            T = T + t3(end);
            [t4, q4, qd4, qdd4] = generate_trapezoid_segment(q_low, 0, abs(v_one), a_max, dt);
            t_act = [t_act; T + t4(2:end)]; q_act = [q_act; q4(2:end)]; qd_act = [qd_act; qd4(2:end)]; qdd_act = [qdd_act; qdd4(2:end)];
            N_full = numel(q_act);
            if j <= 6, q_safe_j = q_safe_left; else, q_safe_j = q_safe_right; end
            q_full = repmat(q_safe_j, N_full, 1);
            q_full(:, j) = q_act;
            qd_full = zeros(N_full, 12); qdd_full = zeros(N_full, 12);
            qd_full(:, j) = qd_act; qdd_full(:, j) = qdd_act;
            seg.time = t_act; seg.q = q_full; seg.qd = qd_full; seg.qdd = qdd_full;
            seg.v_cmd = v_one; seg.q_start = 0; seg.q_end = 0;
            seg.description = sprintf('joint=%s, 0->min->max->min->0 (v=%.1f×)', joint_suffix{j}, vr);
            const_vel_single{iv}(j).segments = seg;
        end
    end
end

%% 5. 汇总输出
traj = struct();
traj.joint_names   = joint_suffix;
traj.pos_limits    = pos_limits;
traj.vel_limits    = vel_limits;
traj.v_max_global  = v_max_global;   % min(vel_limits)，轨迹速度 = vel_list 比例 × 此值
traj.effort_limits = effort_limits;
traj.acc_limits    = acc_limits;
traj.q_safe        = q_safe;
traj.static_poses  = static_poses;
traj.const_vel     = const_vel;
traj.dt            = dt;
traj.vel_list      = vel_list;
traj.n_static      = opts.n_static;
traj.static_hold_time = opts.static_hold_time;
traj.margin_ratio  = opts.margin_ratio;
traj.urdf_path     = urdf_path;

if ~isempty(opts.save_mat)
    save(opts.save_mat, 'traj');
    fprintf('\n已将轨迹保存到 MAT 文件: %s\n', opts.save_mat);
end

%% 6. 可选：导出单个 CSV 文件（time + 12 个关节角），并在一张图上绘制所有关节 q/qd/qdd
if opts.export_csv || opts.plot_traj
    if isempty(opts.csv_dir)
        csv_dir = pwd;
    else
        csv_dir = opts.csv_dir;
    end
    if ~exist(csv_dir, 'dir')
        mkdir(csv_dir);
    end

    % 汇总所有关节、所有速度段的轨迹到一条时间轴
    t_all   = [];
    q_all   = zeros(0, 12);
    qd_all  = zeros(0, 12);
    qdd_all = zeros(0, 12);
    t_current = 0;

    for j = 1:numel(joint_suffix)
        segs = const_vel(j).segments;
        for s = 1:numel(segs)
            seg = segs(s);
            t_seg_rel = seg.time(:);
            if isempty(t_seg_rel)
                continue;
            end
            % 以段内起点为 0，接到全局时间轴后面
            t_seg_rel = t_seg_rel - t_seg_rel(1);
            t_seg = t_current + t_seg_rel;
            t_current = t_seg(end);

            q_seg  = seg.q;
            % 使用规划时的 qd、qdd（若不存在则数值微分一次并存入）
            if isfield(seg, 'qd') && isfield(seg, 'qdd') && ~isempty(seg.qd) && ~isempty(seg.qdd)
                qd_seg  = seg.qd;
                qdd_seg = seg.qdd;
            else
                dt_local = median(diff(t_seg));
                if dt_local <= 0 || isnan(dt_local)
                    dt_local = dt;
                end
                qd_seg  = zeros(size(q_seg));
                qdd_seg = zeros(size(q_seg));
                for col = 1:size(q_seg,2)
                    qd_seg(2:end,col) = diff(q_seg(:,col)) / dt_local;
                    qd_seg(1,col) = qd_seg(2,col);
                    qdd_seg(2:end,col) = diff(qd_seg(:,col)) / dt_local;
                    qdd_seg(1,col) = qdd_seg(2,col);
                end
            end

            t_all   = [t_all;   t_seg];
            q_all   = [q_all;   q_seg];
            qd_all  = [qd_all;  qd_seg];
            qdd_all = [qdd_all; qdd_seg];
        end
    end

    header_cells = [{'time'}, joint_suffix];

    % 导出两个 CSV：正序 + 逆序（time + 12 个关节角）
    if opts.export_csv && ~isempty(t_all)
        base_name = sprintf('traj_%s_all_joints', opts.robot_name);
        % 正序
        fpath1 = fullfile(csv_dir, [base_name, '.csv']);
        fid = fopen(fpath1, 'w');
        if fid == -1
            warning('无法打开文件写入 CSV: %s', fpath1);
        else
            for c = 1:numel(header_cells)
                if c < numel(header_cells), fprintf(fid, '%s,', header_cells{c}); else, fprintf(fid, '%s\n', header_cells{c}); end
            end
            for i = 1:numel(t_all)
                fprintf(fid, '%.5f', t_all(i));
                for col = 1:12, fprintf(fid, ',%.6f', q_all(i,col)); end
                fprintf(fid, '\n');
            end
            fclose(fid);
            fprintf('已导出 CSV: %s (正序)\n', fpath1);
        end
        % 逆序
        fpath2 = fullfile(csv_dir, [base_name, '_reverse.csv']);
        fid = fopen(fpath2, 'w');
        if fid == -1
            warning('无法打开文件写入 CSV: %s', fpath2);
        else
            for c = 1:numel(header_cells)
                if c < numel(header_cells), fprintf(fid, '%s,', header_cells{c}); else, fprintf(fid, '%s\n', header_cells{c}); end
            end
            for i = numel(t_all):-1:1
                fprintf(fid, '%.5f', t_all(i));
                for col = 1:12, fprintf(fid, ',%.6f', q_all(i,col)); end
                fprintf(fid, '\n');
            end
            fclose(fid);
            fprintf('已导出 CSV: %s (逆序)\n', fpath2);
        end
        % 0.15×、0.5×、0.9× 三条轨迹分别导出（每条正序 + 逆序）
        if opts.export_csv_039 && exist('const_vel_single', 'var')
            suffixes = {'_v03', '_v06', '_v09'};
            for iv = 1:3
                cv = const_vel_single{iv};
                t_s = []; q_s = zeros(0, 12); qd_s = zeros(0, 12); qdd_s = zeros(0, 12);
                t_cur = 0;
                for j = 1:numel(joint_suffix)
                    segs = cv(j).segments;
                    for s = 1:numel(segs)
                        seg = segs(s);
                        t_seg_rel = seg.time(:);
                        if isempty(t_seg_rel), continue; end
                        t_seg_rel = t_seg_rel - t_seg_rel(1);
                        t_seg = t_cur + t_seg_rel;
                        t_cur = t_seg(end);
                        t_s = [t_s; t_seg]; q_s = [q_s; seg.q]; qd_s = [qd_s; seg.qd]; qdd_s = [qdd_s; seg.qdd];
                    end
                end
                if isempty(t_s), continue; end
                base_s = sprintf('traj_%s_all_joints%s', opts.robot_name, suffixes{iv});
                for rev = [false, true]
                    if rev
                        fpath_s = fullfile(csv_dir, [base_s, '_reverse.csv']);
                    else
                        fpath_s = fullfile(csv_dir, [base_s, '.csv']);
                    end
                    fid = fopen(fpath_s, 'w');
                    if fid == -1, continue; end
                    for c = 1:numel(header_cells)
                        if c < numel(header_cells), fprintf(fid, '%s,', header_cells{c}); else, fprintf(fid, '%s\n', header_cells{c}); end
                    end
                    if rev, idx = numel(t_s):-1:1; else, idx = 1:numel(t_s); end
                    for i = idx
                        fprintf(fid, '%.5f', t_s(i));
                        for col = 1:12, fprintf(fid, ',%.6f', q_s(i,col)); end
                        fprintf(fid, '\n');
                    end
                    fclose(fid);
                    if rev, ord = '逆序'; else, ord = '正序'; end
                    fprintf('已导出 CSV: %s (%s)\n', fpath_s, ord);
                end
            end
        end
    end

    % 绘图：分图绘制，每张图内正序+逆序画在一起
    if opts.plot_traj && ~isempty(t_all)
        n_joints = numel(joint_suffix);
        % 逆序时间与数据（时间从 0 到 T，便于与正序叠画）
        t_rev = t_all(end) - t_all(end:-1:1);
        q_rev = q_all(end:-1:1, :);
        qd_rev = -qd_all(end:-1:1, :);
        qdd_rev = qdd_all(end:-1:1, :);

        % 图1：主轨迹（名义速度 0.5×）正序+逆序
        figure('Name', 'E1 测试轨迹 - 主轨迹 (正序+逆序)', 'Position', [50, 50, 1600, 900]);
        for j = 1:n_joints
            row = j - 1;
            subplot(n_joints, 3, row*3 + 1);
            plot(t_all, q_all(:, j), 'b-', t_rev, q_rev(:, j), 'b--');
            grid on; ylabel('q (rad)');
            title(sprintf('%s - q', joint_suffix{j}), 'Interpreter', 'none');
            if j == 1, legend('正序', '逆序', 'Location', 'best'); end
            if j == n_joints, xlabel('time (s)'); end

            subplot(n_joints, 3, row*3 + 2);
            plot(t_all, qd_all(:, j), 'r-', t_rev, qd_rev(:, j), 'r--');
            grid on; ylabel('qd (rad/s)');
            title(sprintf('%s - qd', joint_suffix{j}), 'Interpreter', 'none');
            if j == n_joints, xlabel('time (s)'); end

            subplot(n_joints, 3, row*3 + 3);
            plot(t_all, qdd_all(:, j), 'k-', t_rev, qdd_rev(:, j), 'k--');
            grid on; ylabel('qdd (rad/s^2)');
            title(sprintf('%s - qdd', joint_suffix{j}), 'Interpreter', 'none');
            if j == n_joints, xlabel('time (s)'); end
        end

        % 图2～4：0.15×、0.5×、0.9× 各一张，每张正序+逆序
        if opts.export_csv_039 && exist('const_vel_single', 'var')
            suffixes = {'_v015', '_v05', '_v09'};
            titles_plot = {'0.15×v_{max}', '0.5×v_{max}', '0.9×v_{max}'};
            for iv = 1:3
                cv = const_vel_single{iv};
                t_s = []; q_s = zeros(0, 12); qd_s = zeros(0, 12); qdd_s = zeros(0, 12);
                t_cur = 0;
                for j = 1:numel(joint_suffix)
                    segs = cv(j).segments;
                    for s = 1:numel(segs)
                        seg = segs(s);
                        t_seg_rel = seg.time(:);
                        if isempty(t_seg_rel), continue; end
                        t_seg_rel = t_seg_rel - t_seg_rel(1);
                        t_seg = t_cur + t_seg_rel;
                        t_cur = t_seg(end);
                        t_s = [t_s; t_seg]; q_s = [q_s; seg.q]; qd_s = [qd_s; seg.qd]; qdd_s = [qdd_s; seg.qdd];
                    end
                end
                if isempty(t_s), continue; end
                t_s_rev = t_s(end) - t_s(end:-1:1);
                q_s_rev = q_s(end:-1:1, :);
                qd_s_rev = -qd_s(end:-1:1, :);
                qdd_s_rev = qdd_s(end:-1:1, :);

                figure('Name', sprintf('E1 测试轨迹 - %s (正序+逆序)', titles_plot{iv}), 'Position', [50 + iv*30, 50 + iv*30, 1600, 900]);
                for j = 1:n_joints
                    row = j - 1;
                    subplot(n_joints, 3, row*3 + 1);
                    plot(t_s, q_s(:, j), 'b-', t_s_rev, q_s_rev(:, j), 'b--');
                    grid on; ylabel('q (rad)');
                    title(sprintf('%s - q', joint_suffix{j}), 'Interpreter', 'none');
                    if j == 1, legend('正序', '逆序', 'Location', 'best'); end
                    if j == n_joints, xlabel('time (s)'); end

                    subplot(n_joints, 3, row*3 + 2);
                    plot(t_s, qd_s(:, j), 'r-', t_s_rev, qd_s_rev(:, j), 'r--');
                    grid on; ylabel('qd (rad/s)');
                    title(sprintf('%s - qd', joint_suffix{j}), 'Interpreter', 'none');
                    if j == n_joints, xlabel('time (s)'); end

                    subplot(n_joints, 3, row*3 + 3);
                    plot(t_s, qdd_s(:, j), 'k-', t_s_rev, qdd_s_rev(:, j), 'k--');
                    grid on; ylabel('qdd (rad/s^2)');
                    title(sprintf('%s - qdd', joint_suffix{j}), 'Interpreter', 'none');
                    if j == n_joints, xlabel('time (s)'); end
                end
            end
        end
    end
end

fprintf('\n===== 生成测试轨迹完成 =====\n');

end

%% 辅助函数：从 URDF 中读取腿部关节的位姿限位、最大速度与峰值扭矩，并估算腿部总质量
function [pos_limits, vel_limits, effort_limits, leg_mass] = read_leg_joint_limits_from_urdf(urdf_path, joint_names)

pos_limits    = nan(numel(joint_names), 2);
vel_limits    = nan(numel(joint_names), 1);
effort_limits = nan(numel(joint_names), 1);
leg_mass = struct('left', NaN, 'right', NaN);

try
    doc = xmlread(urdf_path);
catch ME
    error('读取 URDF 失败: %s', ME.message);
end

joint_elems = doc.getElementsByTagName('joint');

for idx = 1:numel(joint_names)
    name = joint_names{idx};
    found = false;

    for k = 0:joint_elems.getLength-1
        joint_node = joint_elems.item(k);
        jname = char(joint_node.getAttribute('name'));
        if ~strcmp(jname, name)
            continue;
        end

        % 查找 limit 子节点
        child_nodes = joint_node.getChildNodes();
        for c = 0:child_nodes.getLength-1
            node = child_nodes.item(c);
            if node.getNodeType() ~= node.ELEMENT_NODE
                continue;
            end
            if ~strcmp(char(node.getNodeName()), 'limit')
                continue;
            end

            lower_str   = char(node.getAttribute('lower'));
            upper_str   = char(node.getAttribute('upper'));
            vel_str     = char(node.getAttribute('velocity'));
            effort_str  = char(node.getAttribute('effort'));

            lower_val = str2double(lower_str);
            upper_val = str2double(upper_str);
            vel_val    = str2double(vel_str);
            effort_val = str2double(effort_str);

            if ~isfinite(lower_val) || ~isfinite(upper_val)
                error('URDF 中关节 %s 的 limit.lower/upper 非法或缺失。', name);
            end

            pos_limits(idx,1) = lower_val;
            pos_limits(idx,2) = upper_val;
            if isfinite(vel_val)
                vel_limits(idx) = vel_val;
            else
                vel_limits(idx) = NaN;
            end
            if isfinite(effort_val)
                effort_limits(idx) = effort_val;
            else
                effort_limits(idx) = NaN;
            end

            found = true;
            break;
        end

        if found
            break;
        end
    end

    if ~found
        warning('在 URDF 中未找到关节 %s 的 limit 信息，将使用 [0,0] 和 NaN 速度/扭矩。', name);
        pos_limits(idx,:)  = [0, 0];
        vel_limits(idx)    = NaN;
        effort_limits(idx) = NaN;
    end
end

% 估算左右腿总质量：按 link 名称中包含 leg_l / leg_r 汇总 mass
link_elems = doc.getElementsByTagName('link');
mass_left  = 0;
mass_right = 0;

for k = 0:link_elems.getLength-1
    link_node = link_elems.item(k);
    lname = char(link_node.getAttribute('name'));

    child_nodes = link_node.getChildNodes();
    for c = 0:child_nodes.getLength-1
        node = child_nodes.item(c);
        if node.getNodeType() ~= node.ELEMENT_NODE
            continue;
        end
        if ~strcmp(char(node.getNodeName()), 'inertial')
            continue;
        end

        inertial_children = node.getChildNodes();
        for ic = 0:inertial_children.getLength-1
            mnode = inertial_children.item(ic);
            if mnode.getNodeType() ~= mnode.ELEMENT_NODE
                continue;
            end
            if ~strcmp(char(mnode.getNodeName()), 'mass')
                continue;
            end

            m_str = char(mnode.getAttribute('value'));
            m_val = str2double(m_str);
            if ~isfinite(m_val)
                continue;
            end

            if contains(lname, 'leg_l')
                mass_left = mass_left + m_val;
            elseif contains(lname, 'leg_r')
                mass_right = mass_right + m_val;
            end
        end
    end
end

if mass_left <= 0
    warning('未能从 URDF 中可靠估计左腿质量，将腿质量置为 NaN。');
    mass_left = NaN;
end
if mass_right <= 0
    warning('未能从 URDF 中可靠估计右腿质量，将腿质量置为 NaN。');
    mass_right = NaN;
end

leg_mass.left  = mass_left;
leg_mass.right = mass_right;

end


