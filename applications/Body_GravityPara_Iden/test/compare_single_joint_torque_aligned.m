% compare_single_joint_torque_aligned  单关节往返轨迹对齐查看模型输出力矩和实际力矩误差
%
% 功能：
%   1. 读取包含12个关节往返轨迹的实机/仿真数据
%   2. 将文件分成12段（或按运动区间），每段对应一个关节的往返轨迹
%   3. 对每个关节段进行往返对齐：后半段逆序后与前半段轨迹完全相同、速度完全相反；
%      正向取段内前部（去程 A→B），反向在整段翻转后匹配得返程 B→A，同姿态处力矩平均抵消摩擦
%   4. 计算模型输出力矩（逆动力学）
%   5. 对比实际力矩与模型力矩，计算误差并可视化
%
% 用法：
%   compare_single_joint_torque_aligned()
%   compare_single_joint_torque_aligned('csv_file', 'path/to/file.csv')
%
% 参数：
%   csv_file        - CSV数据文件路径（默认从 data/friction_iden/ 查找）
%   forward_range   - 正向段范围 [start_ratio, end_ratio]（默认 [0.15, 0.45]，即 15%~45%）
%   qdd_smooth      - 加速度数值微分平滑点数（默认 5，0=不平滑）
%   plot_all_joints - 是否绘制所有关节（默认 true）
%   use_motion_interval - true：先检测每个关节连续运动区间再对齐（推荐）；false：按文件等分12段
%                         运动区间规则：只选取一段连续轨迹；若检测到多段则选取最长的那段
%   use_ordered_segments - true：运动按 L1→L2→…→L6→R1→…→R6 顺序，每关节只在其对应时间块内检测（默认 true）
%   overlap_points  - 顺序检测时，当前关节的检测块可从上一关节结束前 overlap_points 点开始
%                     默认为 0（即不往前多取，块起点就是名义段起点）
%   block_end_extend - 顺序检测时，块终点在名义段尾后再延伸点数（保证返程在 35784 等之后也能被包含，默认 0）
%   motion_window   - 细判定窗口长度（±motion_window 点，默认 500），用于排除局部抖动
%   vel_threshold   - 细判定阈值：窗口内速度绝对值平均 > vel_threshold 视为“在动”，默认 0.01 rad/s
%   min_motion_len  - 运动区间最少连续点数，默认 50
%   max_gap_merge   - 两段运动间隔若<=此点数则合并为一段（避免拐点处速度掉下去拆成多段），默认 150
%   max_segment_ratio - 仅用该段前比例（1=整段；<1 时截断），默认 1（用整段）
%
% 输出：
%   生成对比图，显示：
%   - 对齐后的实际力矩（正反向平均）
%   - 模型输出力矩（逆动力学）
%   - 误差（实际-模型）
%   - 误差统计（RMSE, MAE）
%
% 注意：
%   - 如果任何关节段出现正反向段重叠错误，程序会直接退出（使用error）
%   - 默认按每个关节的连续运动区间分段（|qd|>vel_threshold 且连续>=min_motion_len）；可选按文件等分12段
%
% 依赖：
%   - utility_function/read_leg_joint_csv.m
%   - robot_algorithm/Dynamics/E1/compute_e1_limb_dynamics.m（按肢体逆动力学，每条腿 6 维）

function compare_single_joint_torque_aligned(varargin)
p = inputParser;
addParameter(p, 'csv_file', '', @ischar);
addParameter(p, 'forward_range', [0.10, 0.45], @(x) isnumeric(x) && length(x)==2);  % 正向取段内 10%~45%（去程）
addParameter(p, 'qdd_smooth', 5, @isnumeric);
addParameter(p, 'plot_all_joints', true, @islogical);
addParameter(p, 'use_motion_interval', true, @islogical);   % 先检测运动区间再对齐
addParameter(p, 'use_ordered_segments', true, @islogical);  % true=运动按 L1..L6 R1..R6 顺序，每关节只在其时间块内
addParameter(p, 'overlap_points', 0, @isnumeric);          % 默认 0：块起点不向前多取；>0 时才从上一关节结束前 N 点开始
addParameter(p, 'block_end_extend', 0, @isnumeric);    % 块终点在名义段尾后再延伸 N 点（含返程在后的情况）
addParameter(p, 'motion_window', 500, @isnumeric);         % 细判定窗口长度（±motion_window 点）
addParameter(p, 'vel_threshold', 0.01, @isnumeric);        % 细判定阈值：窗口内速度绝对值平均 > 此值视为在动
addParameter(p, 'min_motion_len', 500, @isnumeric);        % 运动区间最少点数
addParameter(p, 'max_gap_merge', 2000, @isnumeric);       % 两段间隔<=此点数则合并为一段
addParameter(p, 'max_segment_ratio', 1, @isnumeric);    % 1=用整段；<1 时仅用该段前比例（多段时已取最长一段）
addParameter(p, 'seed_threshold', 0.2, @isnumeric);        % 运动“强种子”阈值：|qd|>seed_threshold 视为强运动
addParameter(p, 'refine_window', 20, @isnumeric);           % 有 meta 时按 q 精炼的邻域半宽（点数）
parse(p, varargin{:});
opts = p.Results;

%% 1. 定位并读取数据文件（动力学参数辨识数据建议放在 data/dynamics/ 或 data/dynamics/single_pos/）
if isempty(opts.csv_file)
    app_root = fullfile(fileparts(mfilename('fullpath')), '..');
    possible_dirs = {
        fullfile(app_root, 'data', 'dynamics', 'single_pos');
        fullfile(app_root, 'data', 'dynamics');
        fullfile(app_root, 'data', 'friction_iden');
        fullfile(app_root);
    };
    
    csv_files = {};
    for i = 1:length(possible_dirs)
        dir_path = possible_dirs{i};
        if exist(dir_path, 'dir')
            files = dir(fullfile(dir_path, '*.csv'));
            if ~isempty(files)
                for j = 1:length(files)
                    csv_files{end+1} = fullfile(dir_path, files(j).name);
                end
            end
        end
    end
    
    if isempty(csv_files)
        error('未找到CSV文件。请指定 csv_file 参数，或将数据文件放入 data/dynamics/ 或 data/dynamics/single_pos/ 目录下');
    elseif length(csv_files) == 1
        opts.csv_file = csv_files{1};
        fprintf('自动找到CSV文件: %s\n', opts.csv_file);
    else
        fprintf('找到多个CSV文件，请指定 csv_file 参数：\n');
        for i = 1:length(csv_files)
            fprintf('  %d. %s\n', i, csv_files{i});
        end
        error('请使用 csv_file 参数指定文件');
    end
end

if ~exist(opts.csv_file, 'file')
    error('数据文件不存在: %s', opts.csv_file);
end

fprintf('===== 单关节往返轨迹对齐 - 模型力矩 vs 实际力矩 =====\n');
fprintf('数据文件: %s\n', opts.csv_file);

%% 2. 读取数据（保证 principle 在路径）
app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
if exist('ensure_body_gravity_para_iden_path', 'file')
    ensure_body_gravity_para_iden_path();
end
data = read_leg_joint_csv(opts.csv_file);

time     = data.time;
q        = [data.pos_leg_l,   data.pos_leg_r];    % n×12
qd       = [data.vel_leg_l,   data.vel_leg_r];   % n×12
tau_real = [data.torque_leg_l, data.torque_leg_r]; % n×12

joint_suffix = {'leg_l1_joint','leg_l2_joint','leg_l3_joint','leg_l4_joint','leg_l5_joint','leg_l6_joint', ...
    'leg_r1_joint','leg_r2_joint','leg_r3_joint','leg_r4_joint','leg_r5_joint','leg_r6_joint'};
joint_names_short = {'l1','l2','l3','l4','l5','l6','r1','r2','r3','r4','r5','r6'};

fprintf('数据点数: %d, 时长: %.3f s\n', length(time), time(end));

%% 2b. 若存在与 CSV 同名的 _meta.mat，则加载生成器名义段与 q_act，对齐时使用精炼模式
traj_meta = [];
[p_csv, n_csv, ~] = fileparts(opts.csv_file);
meta_path = fullfile(p_csv, [n_csv '_meta.mat']);
if exist(meta_path, 'file')
    traj_meta = load(meta_path);
    fprintf('已加载对齐 meta: %s（将使用名义+精炼对齐）\n', meta_path);
else
    fprintf('未找到 meta 文件（%s），使用时间对称对齐\n', meta_path);
end

%% 3. 为每个关节确定数据范围：运动区间或等分12段
N_total = length(time);
N_per_joint = fix(N_total / 12);  % 仅在不使用运动区间时用

fprintf('\n--- 数据分段信息 ---\n');
fprintf('总数据点数: %d\n', N_total);
if opts.use_motion_interval
    if opts.use_ordered_segments
        fprintf('分段方式: 运动按 L1→…→R6 顺序，块起点=上一关节结束前 %d 点、块终点=名义段尾+%d 点（|qd|>%.3f，间隔<=%d 合并，>=%d 点）\n', opts.overlap_points, opts.block_end_extend, opts.vel_threshold, opts.max_gap_merge, opts.min_motion_len);
    else
        fprintf('分段方式: 按每个关节连续运动区间（|qd|>%.3f rad/s，间隔<=%d 点合并为一段，连续>=%d 点）\n', opts.vel_threshold, opts.max_gap_merge, opts.min_motion_len);
    end
else
    fprintf('分段方式: 按文件等分12段，每段 %d 点\n', N_per_joint);
end

% 存储每个关节的对齐结果（含正向/反向用于绘图）
tau_aligned_all = cell(12, 1);
tau_forward_all = cell(12, 1);
tau_backward_all = cell(12, 1);
q_aligned_all = cell(12, 1);
qd_aligned_all = cell(12, 1);
q_act_aligned_all = cell(12, 1);
q_act_forward_all = cell(12, 1);
q_act_backward_all = cell(12, 1);
range1_all = cell(12, 1);
range2_all = cell(12, 1);
j_act_all = zeros(12, 1);
segment_end_prev = 0;  % 上一关节结束行号，用于 overlap_points（当前关节检测从该行前 overlap_points 点开始）

%% 4. 对每个关节段分别进行往返轨迹对齐
for joint_idx = 1:12
    fprintf('\n--- 处理关节 %d (%s) ---\n', joint_idx, joint_suffix{joint_idx});
    
    % 通过独立的“运动段检测函数”确定当前关节的往返运动区间
    [range_joint, start_idx, end_idx] = detect_single_joint_motion_segment( ...
        qd, joint_idx, joint_suffix{joint_idx}, N_total, N_per_joint, opts);
    
    N = length(range_joint);
    fprintf('关节段范围: 行 %d~%d (%d 点)\n', start_idx, end_idx, N);
    
    % 仅用运动区间前比例，排除后面的运动（如阶跃等）
    if opts.max_segment_ratio < 1
        N_use = max(1, round(N * opts.max_segment_ratio));
        range_joint = range_joint(1 : N_use);
        N = N_use;
        fprintf('仅用前 %.0f%%，有效范围: 行 %d~%d (%d 点)\n', opts.max_segment_ratio*100, range_joint(1), range_joint(end), N);
    end
    
    % 提取当前关节段的局部数据
    time_joint = time(range_joint);
    q_joint    = q(range_joint, :);
    qd_joint   = qd(range_joint, :);
    tau_joint  = tau_real(range_joint, :);
    
    % 当前关节段的主动关节（就是当前关节）
    j_act = joint_idx;
    j_act_all(joint_idx) = j_act;
    
    % 若有 meta，则用名义 q_act 做精炼对齐；否则时间对称
    if ~isempty(traj_meta) && isfield(traj_meta, 'meta_q_act') && ...
            iscell(traj_meta.meta_q_act) && joint_idx <= length(traj_meta.meta_q_act) && ...
            ~isempty(traj_meta.meta_q_act{joint_idx}) && length(traj_meta.meta_q_act{joint_idx}) >= 10
        [range1_global, range2_global, q_fwd_act, q_bwd_act, ...
            tau_forward, tau_backward, min_distance] = ...
            align_single_joint_round_trip(time, q_joint, qd_joint, tau_joint, ...
                range_joint, j_act, opts.forward_range, joint_suffix{joint_idx}, ...
                'nominal_q_act', traj_meta.meta_q_act{joint_idx}, ...
                'refine_window', opts.refine_window);
    else
        [range1_global, range2_global, q_fwd_act, q_bwd_act, ...
            tau_forward, tau_backward, min_distance] = ...
            align_single_joint_round_trip(time, q_joint, qd_joint, tau_joint, ...
                range_joint, j_act, opts.forward_range, joint_suffix{joint_idx});
    end
    
    % === 仅针对当前主动关节的对齐质量检查（姿态+速度方向）===
    % 这里再次用全局索引从原始数据中取轨迹，便于和其它绘图保持一致
    q_fwd_act  = q(range1_global, j_act);          % 当前关节正向轨迹
    q_bwd_act  = flip(q(range2_global, j_act), 1); % 当前关节反向逆序后轨迹
    d_act = norm(q_bwd_act - q_fwd_act, 2);
    % 阈值：按幅值 0.05rad 的量级估计，随点数缩放
    thr_act = 0.05 * sqrt(length(q_fwd_act));
    if d_act > thr_act
        warning('关节 %d (%s) 主动关节对齐误差=%.4f > %.4f，请检查该关节是否为理想往返', ...
            joint_idx, joint_suffix{joint_idx}, d_act, thr_act);
    end
    
    % 速度方向检查：理想往返中，正向与返向速度方向应大多相反
    qd_fwd_act = qd(range1_global, j_act);
    qd_bwd_act = flip(qd(range2_global, j_act), 1);
    sign_prod = sign(qd_fwd_act) .* sign(qd_bwd_act);
    ratio_opposite = mean(sign_prod < 0);  % 速度方向相反的比例
    if ratio_opposite < 0.8
        warning('关节 %d (%s) 速度方向相反比例仅为 %.2f (<0.80)，往返对齐可能不理想', ...
            joint_idx, joint_suffix{joint_idx}, ratio_opposite);
    end
    
    % === 局部轨迹绘图：当前关节的正向 / 反向逆序 ===
    % 直接绘制当前关节选中的正向/反向轨迹（便于肉眼检查）
    figure('Name', sprintf('关节 %d (%s) 对齐局部轨迹', joint_idx, joint_suffix{joint_idx}));
    t_local = time(range1_global);
    plot(t_local, q_fwd_act, 'b.-', 'DisplayName', '正向', 'LineWidth', 1.2); hold on;
    plot(t_local, q_bwd_act, 'r.-', 'DisplayName', '反向逆序', 'LineWidth', 1.2);
    xlabel('time (s)');
    ylabel(sprintf('q_{%s} (rad)', joint_suffix{joint_idx}));
    title(sprintf('关节 %d (%s) 对齐片段（速度符号往返检测）', joint_idx, joint_suffix{joint_idx}));
    legend('Location','best'); grid on; hold off;
    
    % 对齐后的数据：同一关节角处正向与反向力矩平均，抵消摩擦
    tau_aligned_all{joint_idx}  = (tau_forward + tau_backward) / 2;  % 同角处平均，抵消摩擦
    tau_forward_all{joint_idx}  = tau_forward;
    tau_backward_all{joint_idx} = tau_backward;
    
    q_aligned_all{joint_idx}     = q(range1_global, :);
    qd_aligned_all{joint_idx}    = qd(range1_global, :);
    q_act_aligned_all{joint_idx} = q_aligned_all{joint_idx}(:, j_act);  % 对齐后的主动关节角
    q_act_forward_all{joint_idx}  = q_fwd_act;
    q_act_backward_all{joint_idx} = q_bwd_act;   % 已经是逆序后、与正向同相位的轨迹
    
    range1_all{joint_idx} = range1_global;
    range2_all{joint_idx} = range2_global;
end

fprintf('\n--- 所有关节段对齐完成 ---\n');

% 合并所有关节的对齐数据（用于后续处理）
% 选择所有关节中的最小长度作为参考
n_min = inf;
for j = 1:12
    n_min = min(n_min, size(tau_aligned_all{j}, 1));
end

% 合并数据（截取到相同长度）
tau_aligned = zeros(n_min, 12);
q_aligned = zeros(n_min, 12);
qd_aligned = zeros(n_min, 12);
q_act_aligned = zeros(n_min, 12);

for j = 1:12
    n_j = size(tau_aligned_all{j}, 1);
    if n_j >= n_min
        tau_aligned(:, j) = tau_aligned_all{j}(1:n_min, j);
        q_aligned(:, j) = q_aligned_all{j}(1:n_min, j);
        qd_aligned(:, j) = qd_aligned_all{j}(1:n_min, j);
        q_act_aligned(:, j) = q_act_aligned_all{j}(1:n_min);
    else
        error('错误：关节 %d 对齐后数据点数不足', j);
    end
end

n = n_min;

%% 5. 计算加速度（用于逆动力学）
dt = median(diff(time));
if dt <= 0 || isnan(dt), dt = 0.002; end

qdd_aligned = zeros(size(q_aligned));
for j = 1:12
    qdd_aligned(:, j) = [0; diff(qd_aligned(:, j)) / dt];
end

% 平滑加速度（如果启用）
if opts.qdd_smooth > 0
    w = ones(opts.qdd_smooth, 1) / opts.qdd_smooth;
    for j = 1:12
        qdd_aligned(:, j) = conv(qdd_aligned(:, j), w, 'same');
    end
end

%% 6. 计算模型输出力矩（逆动力学）- 按肢体分开算，每条腿 6 维 q/qd/qdd，M/C/G 为 6×6/6×1
fprintf('\n--- 计算模型输出力矩（逆动力学）- 左/右腿分别用 compute_e1_limb_dynamics（6 自由度）---\n');

tau_model = zeros(n, 12);
tau_model_gravity = zeros(n, 12);

fprintf('正在逐点计算逆动力学 (共 %d 点)...\n', n);
for k = 1 : n
    q_left   = q_aligned(k, 1:6)';
    qd_left  = qd_aligned(k, 1:6)';
    qdd_left = qdd_aligned(k, 1:6)';
    [tau_L, ~, ~, G_L] = compute_e1_limb_dynamics('left_leg', q_left, qd_left, qdd_left);
    tau_model(k, 1:6) = tau_L';
    tau_model_gravity(k, 1:6) = G_L';

    q_right   = q_aligned(k, 7:12)';
    qd_right  = qd_aligned(k, 7:12)';
    qdd_right = qdd_aligned(k, 7:12)';
    [tau_R, ~, ~, G_R] = compute_e1_limb_dynamics('right_leg', q_right, qd_right, qdd_right);
    tau_model(k, 7:12) = tau_R';
    tau_model_gravity(k, 7:12) = G_R';
end

%% 7. 计算误差
tau_error = tau_aligned - tau_model;  % 对齐后的实际力矩 - 模型力矩
tau_error_gravity = tau_aligned - tau_model_gravity;  % 对齐后的实际力矩 - 模型重力项

rmse = sqrt(mean(tau_error.^2, 1));
mae  = mean(abs(tau_error), 1);
rmse_gravity = sqrt(mean(tau_error_gravity.^2, 1));
mae_gravity  = mean(abs(tau_error_gravity), 1);

fprintf('\n--- 误差统计 ---\n');
fprintf('各关节 RMSE(对齐实际-全模型): '); 
fprintf(' %.4f', rmse); fprintf(' N·m\n');
fprintf('各关节 MAE(对齐实际-全模型):  '); 
fprintf(' %.4f', mae); fprintf(' N·m\n');
fprintf('各关节 RMSE(对齐实际-重力项): '); 
fprintf(' %.4f', rmse_gravity); fprintf(' N·m\n');
fprintf('各关节 MAE(对齐实际-重力项):  '); 
fprintf(' %.4f', mae_gravity); fprintf(' N·m\n');

%% 8. 可视化
if opts.plot_all_joints
    joints_to_plot = 1:12;
else
    joints_to_plot = 1;  % 只绘制第一个关节
end

% 图1：对齐后的力矩对比（全模型）
figure('Name', '单关节往返对齐 - 力矩对比（全模型）', 'Position', [100, 100, 1400, 900]);
for idx = 1:length(joints_to_plot)
    j = joints_to_plot(idx);
    
    subplot(3, 4, j);
    r1 = range1_all{j};
    time_aligned = time(r1(1:n));  % 对齐后的时间（使用正向段时间）
    plot(time_aligned, tau_aligned(:, j), 'b.-', 'DisplayName', '实际力矩（对齐）', 'LineWidth', 1.5); hold on;
    plot(time_aligned, tau_model(:, j), 'r.-', 'DisplayName', '模型力矩（全动力学）', 'LineWidth', 1.5);
    plot(time_aligned, tau_error(:, j), 'k:', 'DisplayName', '误差', 'LineWidth', 1);
    xlabel('时间 (s)'); 
    ylabel('力矩 (N·m)');
    title(sprintf('%s (RMSE=%.3f)', joint_names_short{j}, rmse(j)));
    legend('Location', 'best');
    grid on;
end
sgtitle('单关节往返对齐: 实际力矩 vs 模型全动力学力矩（每关节独立对齐）');

% 图2：对齐后的力矩对比（重力项）
figure('Name', '单关节往返对齐 - 力矩对比（重力项）', 'Position', [100, 100, 1400, 900]);
for idx = 1:length(joints_to_plot)
    j = joints_to_plot(idx);
    
    subplot(3, 4, j);
    r1 = range1_all{j};
    time_aligned = time(r1(1:n));  % 对齐后的时间（使用正向段时间）
    plot(time_aligned, tau_aligned(:, j), 'b.-', 'DisplayName', '实际力矩（对齐）', 'LineWidth', 1.5); hold on;
    plot(time_aligned, tau_model_gravity(:, j), 'g.-', 'DisplayName', '模型重力项', 'LineWidth', 1.5);
    plot(time_aligned, tau_error_gravity(:, j), 'k:', 'DisplayName', '误差', 'LineWidth', 1);
    xlabel('时间 (s)'); 
    ylabel('力矩 (N·m)');
    title(sprintf('%s (RMSE=%.3f)', joint_names_short{j}, rmse_gravity(j)));
    legend('Location', 'best');
    grid on;
end
sgtitle('单关节往返对齐: 实际力矩 vs 模型重力项（每关节独立对齐）');

% 图3：误差分布
figure('Name', '单关节往返对齐 - 误差分布', 'Position', [100, 100, 1400, 600]);
for idx = 1:length(joints_to_plot)
    j = joints_to_plot(idx);
    
    subplot(2, 6, j);
    histogram(tau_error(:, j), 30, 'FaceColor', 'b', 'EdgeColor', 'k');
    xlabel('误差 (N·m)'); ylabel('频数');
    title(sprintf('%s\nRMSE=%.3f, MAE=%.3f', joint_names_short{j}, rmse(j), mae(j)));
    grid on;
end
sgtitle('误差分布直方图（对齐实际 - 模型全动力学）');

% 图4：每个关节的正向与反向轨迹（时间-力矩）
figure('Name', '单关节往返对齐 - 正向与反向轨迹', 'Position', [100, 100, 1400, 900]);
for idx = 1:length(joints_to_plot)
    j = joints_to_plot(idx);
    subplot(3, 4, j);
    r1 = range1_all{j};
    r2 = range2_all{j};
    time_fwd = time(r1);
    time_bwd = time(r2);
    tau_fwd = tau_forward_all{j}(:, j);
    tau_bwd = tau_backward_all{j}(:, j);
    plot(time_fwd, tau_fwd, 'b.-', 'DisplayName', '正向', 'LineWidth', 1); hold on;
    plot(time_bwd, tau_bwd, 'r.-', 'DisplayName', '反向', 'LineWidth', 1);
    plot(time(r1(1:n)), tau_aligned(1:n, j), 'k--', 'DisplayName', '对齐平均', 'LineWidth', 0.8);
    xlabel('时间 (s)'); ylabel('力矩 (N·m)');
    title(joint_names_short{j});
    legend('Location', 'best'); grid on; hold off;
end
sgtitle('正向与反向轨迹（蓝=正向，红=反向，黑虚线=对齐平均）');

% 图5：对齐前后的关节角轨迹（时间-关节角）
figure('Name', '单关节往返对齐 - 对齐前后关节角轨迹', 'Position', [100, 100, 1600, 900]);
for idx = 1:length(joints_to_plot)
    j = joints_to_plot(idx);
    
    % 左子图：对齐前（正向段和反向段的原始轨迹）
    subplot(3, 8, 2*j-1);
    r1 = range1_all{j};
    r2 = range2_all{j};
    time_fwd = time(r1);
    time_bwd = time(r2);
    q_fwd_raw = q(r1, j);  % 正向段原始关节角（未翻转）
    q_bwd_raw = q(r2, j);  % 反向段原始关节角（未翻转）
    plot(time_fwd, q_fwd_raw, 'b.-', 'DisplayName', '正向段', 'LineWidth', 1, 'MarkerSize', 3); hold on;
    plot(time_bwd, q_bwd_raw, 'r.-', 'DisplayName', '反向段', 'LineWidth', 1, 'MarkerSize', 3);
    xlabel('时间 (s)'); ylabel(sprintf('q_{%s} (rad)', joint_names_short{j}));
    title(sprintf('%s 对齐前', joint_names_short{j}));
    legend('Location', 'best'); grid on; hold off;
    
    % 右子图：对齐后 = 反向逆序后与正向相位对齐的对比（两线重合则对齐良好）
    subplot(3, 8, 2*j);
    time_aligned = time(r1(1:n));  % 以正向段时间为横轴
    q_fwd = q_act_forward_all{j}(1:n);   % 正向段
    q_bwd_rev = q_act_backward_all{j}(1:n);  % 反向段逆序后（与正向逐点相位对齐）
    plot(time_aligned, q_fwd, 'b.-', 'DisplayName', '正向', 'LineWidth', 1, 'MarkerSize', 3); hold on;
    plot(time_aligned, q_bwd_rev, 'r.-', 'DisplayName', '反向逆序', 'LineWidth', 1, 'MarkerSize', 3);
    xlabel('时间 (s)'); ylabel(sprintf('q_{%s} (rad)', joint_names_short{j}));
    title(sprintf('%s 对齐后（正向 vs 反向逆序）', joint_names_short{j}));
    legend('Location', 'best'); grid on; hold off;
end
sgtitle('对齐前后关节角轨迹（左=对齐前：正向蓝、反向红；右=对齐后：正向蓝、反向逆序红，相位对齐对比）');

%% 9. 打印机器人及相关参数
fprintf('\n===== 机器人及相关参数 =====\n');
fprintf('--- 对齐含义 ---\n');
fprintf('  后半段逆序后与前半段：轨迹完全相同（逐点关节角一致）、速度完全相反（返程）；同姿态处力矩平均抵消摩擦\n');
fprintf('--- 数据与文件 ---\n');
fprintf('  csv_file: %s\n', opts.csv_file);
fprintf('  总数据点数: %d, 时长: %.3f s\n', N_total, time(end));
fprintf('  采样周期 dt: %.6f s (%.1f Hz)\n', dt, 1/dt);
fprintf('--- 对齐与分段 ---\n');
fprintf('  use_motion_interval: %d (1=按运动区间, 0=等分12段)\n', opts.use_motion_interval);
fprintf('  use_ordered_segments: %d (1=运动按 L1..L6 R1..R6 顺序)\n', opts.use_ordered_segments);
fprintf('  overlap_points: %d (当前关节检测从上一关节结束前 N 点开始)\n', opts.overlap_points);
fprintf('  block_end_extend: %d (块终点在名义段尾后再延伸 N 点)\n', opts.block_end_extend);
fprintf('  vel_threshold: %.4f rad/s\n', opts.vel_threshold);
fprintf('  min_motion_len: %d 点\n', opts.min_motion_len);
fprintf('  max_gap_merge: %d 点\n', opts.max_gap_merge);
fprintf('  forward_range: [%.2f, %.2f]\n', opts.forward_range(1), opts.forward_range(2));
fprintf('  对齐后每关节点数 n: %d\n', n);
fprintf('--- 各关节段范围（全局行号）---\n');
for j = 1:12
    r1 = range1_all{j}; r2 = range2_all{j};
    fprintf('  关节 %2d (%s): 正向 %d~%d (%d 点), 反向 %d~%d (%d 点)\n', ...
        j, joint_names_short{j}, r1(1), r1(end), length(r1), r2(1), r2(end), length(r2));
end
fprintf('--- 机器人模型 ---\n');
fprintf('  逆动力学: compute_e1_limb_dynamics（左/右腿各 6 维，按肢体分开算）\n');
fprintf('--- 后处理 ---\n');
fprintf('  qdd_smooth: %d (0=不平滑)\n', opts.qdd_smooth);
fprintf('  plot_all_joints: %d\n', opts.plot_all_joints);
fprintf('========================================\n');

%% 10. 汇总输出
fprintf('\n===== 汇总 =====\n');
if opts.use_motion_interval
    if opts.use_ordered_segments
        fprintf('数据分段方式: 运动按 L1→…→R6 顺序，每关节在其时间块内检测运动区间\n');
    else
        fprintf('数据分段方式: 按每个关节连续运动区间（|qd|>%.3f，间隔<=%d 点合并，连续>=%d 点）\n', opts.vel_threshold, opts.max_gap_merge, opts.min_motion_len);
    end
else
    fprintf('数据分段方式: 文件等分12段，每段对应一个关节的往返轨迹\n');
end
fprintf('对齐后数据点数（每关节）: %d\n', n);
fprintf('全关节平均 RMSE(对齐实际-全模型): %.4f N·m\n', mean(rmse));
fprintf('全关节平均 RMSE(对齐实际-重力项): %.4f N·m\n', mean(rmse_gravity));
fprintf('\n说明：\n');
fprintf('  - 对齐准则：后半段逆序后与前半段轨迹完全相同、速度完全相反；同姿态处力矩平均抵消摩擦\n');
fprintf('    在相同关节角处取力矩平均以抵消摩擦，主要保留重力/惯性等项\n');
fprintf('  - 与模型重力项对比可验证重力/质量/质心参数\n');
fprintf('  - 与模型全动力学对比可验证完整动力学模型（包括惯性、科里奥利力等）\n');
fprintf('  - 图「正向与反向轨迹」中蓝=正向、红=反向、黑虚线=对齐平均\n');
fprintf('  - 图「对齐前后关节角轨迹」右列=反向逆序后与正向相位对齐的对比（两线重合则对齐良好）\n');
fprintf('  - 若任何关节段出现重叠错误，程序会直接退出\n');

end
