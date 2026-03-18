%% 激励轨迹生成 - 直接运行本脚本即可（单腿/单臂，BODY_JNT_CURRENT 辨识用）
% 在 MATLAB 中打开本文件，点击“运行”或按 F5，即生成傅里叶激励轨迹并绘图。
%
% ---------- 单腿/单臂 配置说明（E1 为例）----------
% 单腿：6 自由度。任选其一：
%   config.cond_limb = 'left_leg';   % 左腿（默认）
%   config.cond_limb = 'right_leg';  % 右腿
%   config.move_axis = [0, 1, 2, 3, 4, 5];
%   config.init_joint / upper_joint_bound / lower_joint_bound / max_velocity / max_acceleration 均为 6×1
%
% 单臂：3 自由度。任选其一：
%   config.cond_limb = 'left_arm';   % 左臂
%   config.cond_limb = 'right_arm';  % 右臂
%   config.move_axis = [0, 1, 2];
%   config.init_joint / upper_joint_bound / lower_joint_bound / max_velocity / max_acceleration 均为 3×1
%   config.robot_limb 用 get_e1_limb_robot('left_arm') 或 'right_arm'，config.max_effort 用 3×1（如 50*ones(3,1)）
% 切换时改 config 中与维度相关的项（init_joint、限位、速度/加速度）；单臂时 n_right 会自动置 0。见注释“可选：单臂”示例。
% ----------

% 添加路径（本文件在 applications/Excitation_Trajectory_E1/excitation_trajectory 内，仓库根为 model_e1）
clear;clc;close all;
this_dir   = fileparts(mfilename('fullpath'));
parent_dir = fullfile(this_dir, '..');                % Excitation_Trajectory_E1
repo_root  = fullfile(parent_dir, '..', '..');        % model_e1 仓库根
addpath(this_dir);
addpath(parent_dir);
% 需要调用 Body_GravityPara_Iden 下的 ensure_body_gravity_para_iden_path / ReMatrix_E1_limb_URDF 等
addpath(fullfile(repo_root, 'applications', 'Body_GravityPara_Iden'));
ensure_body_gravity_para_iden_path();   % utility_function, dynamics, robot_model

% 快速验证：true 时降低采样/周期/优化迭代，流程不变但更快跑通；正式生成改为 false
FAST_VERIFY = true;

% 配置：生成左腿激励轨迹，右腿关节位置恒为 0（输出为全身 12 关节：左 6 + 右 6 零）
config = struct();
config.type = 10;
% 左腿 6 关节（L1..L6），右腿不参与优化，执行时置 0
config.move_axis = [0, 1, 2, 3, 4, 5];
config.init_joint = [0; 0; 0; 0; 0; 0];
n_right = 6;   % 右腿关节数，输出时补零
config.traj_cycle = 5;
config.sample_frequency = 100;
% 关节角范围：二选一，注释掉不用的
config.upper_joint_bound = 2*pi * ones(6, 1);
config.lower_joint_bound = -2*pi * ones(6, 1);
% config.upper_joint_bound = [0.61;  0.436; 0.785; 2.44;  0.523; 0.262];
% config.lower_joint_bound = [-0.61; -0.261; -2.09; 0;    -0.872; -0.262];
config.max_velocity      = [16.75; 20.1;  20.1;  13.18; 12.46; 12.46];
config.max_acceleration  = [200.0;   200.0;   200.0;   200.0;   200.0;   200.0];
% 傅里叶阶数与周期：速度幅值 ∝ omega*系数，omega=2*pi/period。想提高速度峰值（如接近 max_velocity=20）可缩周期或升阶
config.order = 6;
config.period = 8;
% 速度偏小时可改用（二选一）：缩周期 period=6 或 8、或升阶 order=6，再配合下边系数界
% config.order = 6;  config.period = 6;
config.sample_number = 15;
% 若校验常因扭矩或速度超限失败，可改为：config.order = 3; config.period = 12;
% 幅值安全余量 alpha：sum(|a|+|b|)<=alpha*(upper-lower)/2，0.6~0.8
config.amplitude_alpha = 0.7;
% 默认进行碰撞检测（优化与校验均做自碰撞检测）；不需要时设 config.enable_collision_check = false
config.enable_collision_check = true;
% 与 TLBO_E1 完全一致：使用 config 原始限位（不收紧为完整范围的 0.4/0.9 速度），校核也用同一套限位
config.use_full_bounds_for_optim = true;
% 系数界：默认 ±0.5；想更大速度可放宽到 ±0.8 或 ±1（受 amplitude_alpha 幅值约束限制）
% config.coeff_lower = -0.8;  config.coeff_upper = 0.8;

if FAST_VERIFY
    config.sample_frequency = 100;
    config.traj_cycle = 1;
end

% 可选：先 4 关节（L2,L3,L4,L5，索引 1,2,3,4），L1/L6 固定中位，可行域更大
% config.move_axis = [1, 2, 3, 4];
% config.init_joint = [0; 0; 0; 0];
% config.upper_joint_bound = [0.436; 0.785; 2.44;  0.523];   % 对应 L2,L3,L4,L5
% config.lower_joint_bound = [-0.261; -2.09; 0;    -0.872];
% config.max_velocity      = [20.1;  20.1;  13.18; 12.46];
% config.max_acceleration  = [200; 200; 200; 200];

% 可选：单臂（3 轴）—— 取消下面整段注释并注释掉上面“左腿 6 关节”的 config 相关行
% config.move_axis = [0, 1, 2];
% config.init_joint = [0; 0; 0];
% config.upper_joint_bound = 2*pi * ones(3, 1);   % 按 E1 臂限位修改
% config.lower_joint_bound = -2*pi * ones(3, 1);
% config.max_velocity      = [10; 10; 10];        % 按实际限速修改
% config.max_acceleration  = [100; 100; 100];
% config.regressor_source = 'E1_limb';
% config.cond_limb = 'left_arm';   % 或 'right_arm'（n_right 会自动置 0，输出仅 3 列）

% robot_limb 与 max_effort：峰值力矩 (N·m)，校核限值=0.9*max_effort；单腿(6)来自 URDF 或 E1 默认
E1_LEG_EFFORT_DEFAULT = [110; 170; 170; 330; 70; 70];   % E1.urdf 腿关节 effort (N·m)
if ~isfield(config, 'robot_limb') || ~isfield(config, 'max_effort')
    limb = 'left_leg';
    if isfield(config, 'cond_limb') && ~isempty(config.cond_limb)
        limb = config.cond_limb;
    end
    try
        [config.robot_limb, n_limb] = get_e1_limb_robot(limb);
        urdf_path = fullfile(repo_root, 'noetix_description', 'urdf', 'E1.urdf');
        if ~exist(urdf_path, 'file')
            urdf_path = fullfile(repo_root, 'urdf', 'E1.urdf');
        end
        if n_limb == 6 && exist(urdf_path, 'file') && exist('get_limb_effort_from_urdf', 'file')
            config.max_effort = get_limb_effort_from_urdf(urdf_path, limb);
            nan_idx = ~isfinite(config.max_effort);
            if any(nan_idx), config.max_effort(nan_idx) = E1_LEG_EFFORT_DEFAULT(nan_idx); end
        else
            if n_limb == 6, config.max_effort = E1_LEG_EFFORT_DEFAULT; else, config.max_effort = 50 * ones(n_limb, 1); end
        end
    catch
        n = numel(config.move_axis);
        if n == 6, config.max_effort = E1_LEG_EFFORT_DEFAULT; else, config.max_effort = 50 * ones(n, 1); end
    end
end

% 单臂时输出不补右腿零列
if isfield(config, 'cond_limb') && ismember(config.cond_limb, {'left_arm', 'right_arm'})
    n_right = 0;
end

% 条件数：与 TLBO_E1 一致，采用方案 B（列归一化 + rref + sqrt(cond(W_min'*W_min))）
if ~isfield(config, 'regressor_source')
    config.regressor_source = 'E1_limb';
    config.cond_limb = 'left_leg';
end
cond_fun = cond_fun_from_regressor(config);
if isempty(cond_fun)
    fprintf('未配置辨识矩阵条件数（无 regressor_source 或 Robot），优化将使用激励量代理目标。\n');
end

% 生成轨迹：优化有三层结构（由外到内）——
%  【次】try_i：一次 = 整次“多轮优化 → 得到系数 → 生成轨迹 → 校验”。校验不过则再来第 2 次（重新做多轮优化+校验）。max_tries 控制最多几次。
%  【轮】retry：在一次里面，用不同随机初值跑多轮优化，每轮调用一次 fmincon；取“exitflag>0 且 cond<1000”的最好系数。max_retries 控制轮数。
%  【iter】在每轮里面，fmincon 的迭代步；每步更新系数、算目标/约束，直到满足停止条件或达到 MaxIterations/MaxFunctionEvaluations。
fprintf('===== 激励轨迹生成 =====\n');
if FAST_VERIFY
    fprintf('【快速验证模式】采样 %d Hz、周期数 %d、优化迭代/评估上限已缩小\n', config.sample_frequency, config.traj_cycle);
end
% 重试次数：2 次以缩短总时间（原 5 次约半小时级）
max_tries = 2;
for try_i = 1:max_tries
    use_optim = true;   % 始终优化，否则随机系数很难满足约束
    if use_optim
        if try_i == 1
            fprintf('\n-------- 第 %d 次尝试：系数优化 --------\n', try_i);
        else
            fprintf('\n-------- 第 %d 次尝试：校验未通过，重新系数优化 --------\n', try_i);
        end
    end
    gen_opts = {'validate', true, 'optimize', use_optim, 'display_optim', use_optim, 'cond_fun', cond_fun};
    if FAST_VERIFY
        gen_opts = [gen_opts, {'max_iter_optim', 30, 'max_feval_optim', 800}];
    end
    [trajectory, trajPara, refPos, refVel, refAcc, t_period, ~, ~, ~, valid_ok, violation_details] = ...
        generate_excitation_trajectory(config, gen_opts{:});
    if valid_ok
        if try_i > 1
            fprintf('第 %d 次尝试：优化后校验通过。\n', try_i);
        end
        break;
    end
    % 打印具体超限明细（最多前 25 条，避免刷屏）
    nv = numel(violation_details);
    max_show = 25;
    fprintf('\n【校验未通过】共 %d 处超限：\n', nv);
    for k = 1:min(nv, max_show)
        d = violation_details(k);
        switch d.type
            case 'pos_upper', tname = '位置上超';
            case 'pos_lower', tname = '位置下超';
            case 'vel',       tname = '速度超限';
            case 'acc',       tname = '加速度超限';
            case 'torque',    tname = '扭矩超限';
            case 'collision', tname = '自碰撞';
            otherwise,        tname = d.type;
        end
        if strcmp(d.type, 'collision')
            fprintf('  时刻 %d, %s\n', d.sample_idx, tname);
        else
            fprintf('  时刻 %d, 关节 %d, %s: 当前值=%.6f, 限值=%.6f, 超限量=%.6f\n', ...
                d.sample_idx, d.joint_idx, tname, d.value, d.limit, d.excess);
        end
    end
    if nv > max_show
        fprintf('  ... 其余 %d 处省略\n', nv - max_show);
    end
    if try_i < max_tries
        fprintf('将进行第 %d 次优化并重新校验。\n', try_i + 1);
    else
        warning('已尝试 %d 次，校验仍未通过，使用最后一次生成结果。', max_tries);
    end
end

% 扩展为全身轨迹（用于绘图）：未运行的一腿恒 0；CSV 输出时改为「0位→过渡入→n周期→过渡出→0位」+ 另一腿安全位
if n_right > 0
    if strcmp(limb, 'left_leg')
        trajectory = [trajectory(:, 1), trajectory(:, 2:end), zeros(size(trajectory, 1), n_right)];
    else
        trajectory = [trajectory(:, 1), zeros(size(trajectory, 1), n_right), trajectory(:, 2:end)];
    end
else
    trajectory = [trajectory(:, 1), trajectory(:, 2:end)];
end
refPos_full = [refPos, zeros(size(refPos, 1), n_right)];
% 另一腿安全位与过渡段（与 plan 一致，用于写 CSV）
E1_ROLL_SAFE_RIGHT_LOWER = -0.436;
E1_ROLL_SAFE_LEFT_UPPER  = 0.436;
TRANSITION_TIME = 1.5;   % 起止过渡段时长 (s)，增大可降低过渡段最大速度；配合 max_velocity_ramp 限速
refVel_full = [refVel, zeros(size(refVel, 1), n_right)];
refAcc_full = [refAcc, zeros(size(refAcc, 1), n_right)];

% 打印当前轨迹的限值参数与校核参数
fprintf('\n----- 当前轨迹限值参数 -----\n');
dim_limb = numel(config.move_axis);
fprintf('关节数（参与优化）: %d，右腿补零列数: %d\n', dim_limb, n_right);
fprintf('位置上界 upper_joint_bound (rad): '); fprintf('%g ', config.upper_joint_bound(1:min(6,dim_limb))); fprintf('\n');
fprintf('位置下界 lower_joint_bound (rad): '); fprintf('%g ', config.lower_joint_bound(1:min(6,dim_limb))); fprintf('\n');
fprintf('速度限值 max_velocity (rad/s): '); fprintf('%g ', config.max_velocity(1:min(6,dim_limb))); fprintf('\n');
fprintf('加速度限值 max_acceleration (rad/s²): '); fprintf('%g ', config.max_acceleration(1:min(6,dim_limb))); fprintf('\n');
if isfield(config, 'max_effort') && ~isempty(config.max_effort)
    fprintf('峰值力矩 max_effort (N·m): '); fprintf('%g ', config.max_effort(1:min(6,dim_limb))); fprintf('\n');
end
fprintf('----- 校核参数 -----\n');
fprintf('设计位置范围: 完整范围的 0.4（use_full_bounds_for_optim=false 时）；校核位置范围: 完整范围的 0.45\n');
fprintf('速度/加速度校核: relax_vel=relax_acc=1.0；扭矩校核限值 = 0.9*max_effort（峰值90%%）\n');
coll_str = '开启'; if ~(isfield(config,'enable_collision_check') && config.enable_collision_check), coll_str = '关闭'; end
fprintf('自碰撞检测: %s\n', coll_str);
fprintf('周期 period=%.2f s，采样 %d Hz，单周期点数 %d\n', config.period, config.sample_frequency, size(refPos, 1));
fprintf('\n');

% 检查点 1：初值 qd(0)、qdd(0) 是否接近 0
% 检查点 2：每关节四峰值及相对限位的裕度
dim_limb = size(refPos, 2);
qd0 = refVel(1,:);
qdd0 = refAcc(1,:);
fprintf('----- 检查点 1：初值 qd(0), qdd(0)（应接近 0）-----\n');
for j = 1:dim_limb
    fprintf('  关节 %d: qd(0)=%.6e, qdd(0)=%.6e\n', j, qd0(j), qdd0(j));
end
% 裕度 = 最朴素定义，便于判断谁在卡缩放（Standalone 用 config 中的限位，若 use_full=false 则为设计限位 0.4×完整范围）
fprintf('----- 检查点 2：每关节 max q, min q, max|qd|, max|qdd| 及裕度 -----\n');
fprintf('     裕度公式: up_pos=upper-max(q), lo_pos=min(q)-lower, vel=max_vel-max|qd|, acc=max_acc-max|qdd|\n');
upper = config.upper_joint_bound(1:dim_limb);
lower = config.lower_joint_bound(1:dim_limb);
max_vel = config.max_velocity(1:dim_limb);
max_acc = config.max_acceleration(1:dim_limb);
for j = 1:dim_limb
    max_q = max(refPos(:,j)); min_q = min(refPos(:,j));
    max_qd = max(abs(refVel(:,j))); max_qdd = max(abs(refAcc(:,j)));
    up_pos   = upper(j) - max_q;           % 位置上裕度
    lo_pos   = min_q - lower(j);          % 位置下裕度
    vel_margin = max_vel(j) - max_qd;     % 速度裕度
    acc_margin = max_acc(j) - max_qdd;    % 加速度裕度
    fprintf('  关节 %d: max q=%.4f, min q=%.4f, max|qd|=%.4f, max|qdd|=%.4f\n', j, max_q, min_q, max_qd, max_qdd);
    fprintf('         裕度: up_pos=%.4f, lo_pos=%.4f, vel=%.4f, acc=%.4f\n', up_pos, lo_pos, vel_margin, acc_margin);
end
fprintf('\n');

% 打印信息
fprintf('单周期点数 CN = %d，左腿 dim = %d，全身关节数 = %d（右腿恒 0）\n', size(refPos, 1), size(refPos, 2), size(refPos_full, 2));
fprintf('系数个数 = %d\n', numel(trajPara));
fprintf('单周期轨迹（左腿 6 轴）极值：\n');
for j = 1:size(refPos, 2)
    fprintf('  轴 %d  位置 (rad)     min = %.4e, max = %.4e\n', j, min(refPos(:,j)), max(refPos(:,j)));
    fprintf('       速度 (rad/s)   min = %.4e, max = %.4e\n',   j, min(refVel(:,j)), max(refVel(:,j)));
    fprintf('       加速度 (rad/s²) min = %.4e, max = %.4e\n', j, min(refAcc(:,j)), max(refAcc(:,j)));
end

% ---------- 图1：完整激励轨迹 - 全时间段 12 关节位置（含过渡段+周期重复+回零）----------
t_full = trajectory(:, 1);
n_joint_full = size(trajectory, 2) - 1;   % 12
figure('Name', '激励轨迹 - 完整轨迹 12 关节位置', 'Position', [50, 50, 1200, 900]);
for j = 1:n_joint_full
    subplot(3, 4, j);
    plot(t_full, trajectory(:, j+1), 'b-'); grid on;
    ylabel('q (rad)');
    if j <= 6
        title(sprintf('关节 %d（左腿）', j));
    else
        title(sprintf('关节 %d（右腿，恒0）', j));
    end
    if j > 8, xlabel('t (s)'); end
end
sgtitle('完整激励轨迹：全时间段关节位置');

% ---------- 图2：单周期 12 关节 位置/速度/加速度 ----------
figure('Name', '激励轨迹 - 单周期 12 关节 q / qd / qdd', 'Position', [80, 80, 1400, 900]);
for j = 1:size(refPos_full, 2)
    subplot(size(refPos_full, 2), 3, (j-1)*3 + 1);
    plot(t_period, refPos_full(:,j), 'b-'); grid on; ylabel('q (rad)');
    if j == 1, title('位置'); end
    if j <= 6, ylabel(sprintf('q%d(左)', j)); else, ylabel(sprintf('q%d(右)', j-6)); end
    subplot(size(refPos_full, 2), 3, (j-1)*3 + 2);
    plot(t_period, refVel_full(:,j), 'r-'); grid on; ylabel('qd (rad/s)');
    if j == 1, title('速度'); end
    subplot(size(refPos_full, 2), 3, (j-1)*3 + 3);
    plot(t_period, refAcc_full(:,j), 'k-'); grid on; ylabel('qdd (rad/s^2)');
    if j == 1, title('加速度'); end
end
xlabel('time (s)');
sgtitle('单周期参考：12 关节位置 / 速度 / 加速度');

% ---------- 图3：末端笛卡尔轨迹 xyz ----------
if isfield(config, 'robot_limb') && ~isempty(config.robot_limb)
    pillar_plot = [-0.5, 0.0, 0.1, 0.1];   % 柱体 [cx, cy, half_dx, half_dy]
    plot_end_effector_trajectory(config.robot_limb, refPos, t_period, 'Standalone', pillar_plot);
end

% 校核通过时生成 CSV（单腿时与 plan 一致：0位→过渡入→n周期激励→过渡出→0位，另一腿安全位）
if valid_ok
    csv_name = fullfile(this_dir, 'excitation_trajectory_standalone.csv');
    if size(trajectory, 2) == 13
        if strcmp(limb, 'left_leg')
            other_leg_safe_6 = [0, E1_ROLL_SAFE_RIGHT_LOWER, 0, 0, 0, 0];
        else
            other_leg_safe_6 = [0, E1_ROLL_SAFE_LEFT_UPPER, 0, 0, 0, 0];
        end
        Ts = 1 / config.sample_frequency;
        t_one = (0:size(refPos, 1)-1)' * Ts;
        max_vel_ramp = config.max_velocity(1:min(6, numel(config.max_velocity)));
        [t_full, Q12] = build_excitation_leg_trajectory_with_ramps(t_one, refPos, limb, config.traj_cycle, TRANSITION_TIME, Ts, other_leg_safe_6, refVel, refAcc, max_vel_ramp);
        write_leg_trajectory_csv_plan_format(csv_name, t_full, Q12);
        % 完整轨迹关节角/速度/加速度
        if numel(t_full) >= 2
            Ts_full = t_full(2) - t_full(1);
            qd_full = [zeros(1, 12); diff(Q12) / Ts_full];
            qdd_full = [zeros(1, 12); diff(qd_full) / Ts_full];
            figure('Name', 'Standalone 激励轨迹 - 完整 12 关节 q/qd/qdd', 'Position', [120, 120, 1400, 900]);
            for j = 1:12
                subplot(12, 3, (j-1)*3 + 1);
                plot(t_full, Q12(:, j), 'b-'); grid on; ylabel('q (rad)'); if j == 1, title('位置'); end
                subplot(12, 3, (j-1)*3 + 2);
                plot(t_full, qd_full(:, j), 'r-'); grid on; ylabel('qd (rad/s)'); if j == 1, title('速度'); end
                subplot(12, 3, (j-1)*3 + 3);
                plot(t_full, qdd_full(:, j), 'k-'); grid on; ylabel('qdd (rad/s^2)'); if j == 1, title('加速度'); end
            end
            xlabel('t (s)');
            sgtitle('Standalone 激励轨迹 - 完整 12 关节 q/qd/qdd');
        end
        fprintf('校核通过，已保存轨迹: %s（0位→过渡入→%d周期→过渡出→0位，另一腿安全位）\n', csv_name, config.traj_cycle);
    else
        var_names = [{'t'}, arrayfun(@(j) sprintf('q%d', j), 1:size(trajectory,2)-1, 'UniformOutput', false)];
        tbl = array2table(trajectory, 'VariableNames', var_names);
        writetable(tbl, csv_name);
        fprintf('校核通过，已保存轨迹: %s\n', csv_name);
    end
    % 保存单周期参考网格，供辨识流程 cycle_average 使用（与 CSV 同目录、同名_ref.mat）
    ref_t = t_period;
    ref_q = refPos;
    ref_qd = refVel;
    [csv_dir, base, ~] = fileparts(csv_name);
    ref_mat = fullfile(csv_dir, [base '_ref.mat']);
    save(ref_mat, 'ref_t', 'ref_q', 'ref_qd');
    fprintf('已保存单周期参考: %s（辨识时周期平均将使用该网格）\n', ref_mat);
end

% 可选：保存到 MAT（含全身轨迹 trajectory、refPos_full 等，右腿为 0）
% save(fullfile(this_dir, 'excite_traj.mat'), 'config', 'trajectory', 'trajPara', 'refPos', 'refVel', 'refAcc', 'refPos_full', 'refVel_full', 'refAcc_full', 't_period');

fprintf('系数（前 12 个）: '); fprintf('%.6f ', trajPara(1:min(12,end))); fprintf('...\n');
fprintf('===== 完成 =====\n');
