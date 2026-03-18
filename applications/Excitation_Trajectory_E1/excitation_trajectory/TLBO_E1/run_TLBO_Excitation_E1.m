%% TLBO 激励轨迹优化 - E1 单肢（与 AnaAp.PhyConst.ExcitOptimi.DynIDen 思路一致）
%
% 约束与参数与 run_excitation_trajectory_standalone.m 一致，便于对比两种方案效果：
% 同一套关节限位、速度/加速度、周期与采样。条件数与 standalone 一致：方案 B（列归一化 + rref + sqrt(cond(W_min'*W_min))）。
%
% 动力学辨识激励建议（可改善 cond(Y_min) 与辨识精度）：
%   - 激励总时长 30–60 s（period × traj_cycle）
%   - 频率范围约 0.2–2 Hz（傅里叶基频 1/period～L/period）
%   - 全关节 max|qdd| > 20 rad/s²（理想 30–80）
%   - 需更强激励时可设 LARGE_EXCITATION = true
%
% 运行前确保路径已加：本目录、Body_GravityPara_Iden、ensure_body_gravity_para_iden_path。

clear; clc; close all;

this_dir = fileparts(mfilename('fullpath'));
parent_dir = fullfile(this_dir, '..');
body_iden_dir = fullfile(parent_dir, '..');
repo_root = fullfile(body_iden_dir, '..');
addpath(this_dir);
addpath(parent_dir);
addpath(body_iden_dir);
if exist('ensure_body_gravity_para_iden_path', 'file')
    ensure_body_gravity_para_iden_path();
end

% ---------- 与 run_excitation_trajectory_standalone.m 一致的配置 ----------
limb = 'left_leg';   % 'left_leg' | 'right_leg' | 'left_arm' | 'right_arm'
[robot_limb, DOF] = get_e1_limb_robot(limb);

% 关节限位、速度、加速度：与 standalone 一致；若希望速度/加速度更大、进一步改善 cond(Y)，可设 true
LARGE_EXCITATION = false;   % true：宽位置 ±2π，速度/加速度易顶满，激励更强；false：紧限位便于与 standalone 对比
if DOF == 6
    if LARGE_EXCITATION
        q_max = 2*pi * ones(6, 1);
        q_min = -2*pi * ones(6, 1);
    else
        q_max = [0.61;  0.436; 0.785; 2.44;  0.523; 0.262];
        q_min = [-0.61; -0.261; -2.09; 0;    -0.872; -0.262];
    end
    dq_max = [16.75; 20.1;  20.1;  13.18; 12.46; 12.46].*0.9;
    ddq_max = [200.0; 200.0; 400.0; 400.0; 200.0; 200.0];
%     dq_max = [6.0; 3.0;  15.0;  10.0; 12.0; 8.0];
%     ddq_max = [200.0; 200.0; 400.0; 400.0; 200.0; 200.0].*0.9;
else
    q_max = 2*pi * ones(DOF, 1);
    q_min = -2*pi * ones(DOF, 1);
    dq_max = [10; 10; 10];
    ddq_max = [100; 100; 100];
end
q_max = q_max(:); q_min = q_min(:); dq_max = dq_max(:); ddq_max = ddq_max(:);
% 设计用位置范围：半宽 = 0.4×完整范围（q_max−q_min），以中间点对称；校核半宽 = 0.45×完整范围（在 validate_trajectory_E1 内）
center_pos = (q_max + q_min) / 2;
full_range_pos = q_max - q_min;   % 完整范围
q_max_design = center_pos + 0.44 * full_range_pos;
q_min_design = center_pos - 0.44 * full_range_pos;
% Tradeoff_Modify 用速度设计值：放宽为真实限值的 1.5 倍，使 Gama_2 不成为瓶颈
% 超速个体由 Constraints_Violation_E1 的惩罚项（用真实 dq_max）淘汰
dq_max_design = dq_max * 1.5;

% robot_limb 与 max_effort：峰值力矩 (N·m)，校核限值 = 0.9*max_effort（与 standalone 一致）
% 来源：URDF <limit effort="..."/>；若读取为 NaN 则用 E1 腿默认值（来自 E1.urdf：L1~L6 = 110,170,170,330,70,70）
E1_LEG_EFFORT_DEFAULT = [110; 170; 170; 330; 70; 70];   % 左/右腿相同，单位 N·m
try
    urdf_path = fullfile(repo_root, 'noetix_description', 'urdf', 'E1.urdf');
    if ~exist(urdf_path, 'file'), urdf_path = fullfile(repo_root, 'urdf', 'E1.urdf'); end
    if DOF == 6 && exist(urdf_path, 'file') && exist('get_limb_effort_from_urdf', 'file')
        max_effort = get_limb_effort_from_urdf(urdf_path, limb);
        nan_idx = ~isfinite(max_effort);
        if any(nan_idx)
            max_effort(nan_idx) = E1_LEG_EFFORT_DEFAULT(nan_idx);
        end
    else
        if DOF == 6, max_effort = E1_LEG_EFFORT_DEFAULT; else, max_effort = 50 * ones(DOF, 1); end
    end
catch
    if DOF == 6, max_effort = E1_LEG_EFFORT_DEFAULT; else, max_effort = 50 * ones(DOF, 1); end
end
% 校核时扭矩限值 = 0.9*max_effort（即“峰值90%”）；可行化时 λ_τ 也用此比例
TORQUE_FEASIBILITY_RATIO = 0.92;
% 另一腿安全位（与 plan 一致：左腿激励时右腿 roll=lower，右腿激励时左腿 roll=upper，防碰撞）
E1_ROLL_SAFE_RIGHT_LOWER = -0.436;   % r_leg_hip_roll lower
E1_ROLL_SAFE_LEFT_UPPER  = 0.436;    % l_leg_hip_roll upper
TRANSITION_TIME = 1;               % 起止过渡段时长 (s)，增大可降低过渡段最大速度；配合 max_velocity_ramp 限速
PILLAR_CFG = [-0.5, 0.0, 0.1, 0.1]; % 柱体避障 [cx, cy, half_dx, half_dy]（截面 0.2×0.2m，中心 x=-0.5 y=0，即 x∈[-0.6,-0.4]）
if DOF >= 1, fprintf('峰值力矩 max_effort (N·m) = '); fprintf('%g ', max_effort); fprintf('\n'); end

% 周期与采样：动力学辨识建议 激励总时长 30–60 s、基频 0.2–2 Hz、max|qdd| > 20 rad/s²（理想 30–80）
% period 单周期时长(s)；傅里叶 L=4 时频率为 1/period～4/period (Hz)。period=8 → 0.125～0.5 Hz
period = 8.0;
% period = 2;  % 短周期：基频高、单周期内 qdd 幅值易大，但总激励时间需靠 traj_cycle 拉长
sample_frequency = 500;
Exciting_Time = period;
Sampling_Time = 1 / sample_frequency;
Tf = Exciting_Time;
wf = 2*pi/Tf;

% 条件数采样：单周期内按 sample_frequency 采样，与 standalone 单周期点数一致
Calculate_Num = floor(period * sample_frequency) + 1;   % 801
Calculate_Init = 0;
Calculate_Interval = 1;

Num_Design_Variate_OneDof = 11;   % 每关节系数数 = 2*L+3，L=4
Num_Design_Variate = DOF * Num_Design_Variate_OneDof;

Population = 10;
Iteration = 20;

% 是否对约束违反加惩罚（类似 Heuristic TLBO）
use_constraint_penalty = true;
penalty_scale = 10000;
% 碰撞检测：校核与（可选）约束惩罚均使用；与 standalone 的 config.enable_collision_check 对应
Enable_Collision_Check = true;
% 输出轨迹的周期重复次数：优化与校核仍按单周期；CSV 与多周期绘图按此重复（与 standalone 的 traj_cycle 一致）
% 激励总时长 ≈ period * traj_cycle（不含过渡段），建议 30–60 s → 例如 period=8, traj_cycle=6 得 48 s
traj_cycle = 6;   % 6 周期 × 8 s = 48 s 激励；可改为 4～8 以落在 30–60 s

% ---------- 初始化种群：随机系数 + Tradeoff_Modify 保证在限位内 ----------
Coefficient_ExTra_Current = -3 + 6*rand(Population, Num_Design_Variate);

for i = 1:Population
    for Joint = 1:DOF
        XI = Coefficient_ExTra_Current(i, (Num_Design_Variate_OneDof*(Joint-1)+1):Num_Design_Variate_OneDof*Joint);
        XI = Tradeoff_Modify(XI, wf, q_max_design(Joint), q_min_design(Joint), dq_max_design(Joint), ddq_max(Joint));
        Coefficient_ExTra_Current(i, (Num_Design_Variate_OneDof*(Joint-1)+1):Num_Design_Variate_OneDof*Joint) = XI;
    end
    % 扭矩缩放 λ_τ：使 max|τ| ≤ TORQUE_FEASIBILITY_RATIO*max_effort
    Coefficient_ExTra_Current(i,:) = apply_torque_scale_E1(Coefficient_ExTra_Current(i,:), wf, DOF, Num_Design_Variate_OneDof, robot_limb, max_effort, TORQUE_FEASIBILITY_RATIO);
end

for i = 1:Population
    Value(i) = Objective_E1_limb(Coefficient_ExTra_Current(i,:), wf, Calculate_Num, Calculate_Interval, Calculate_Init, Sampling_Time, DOF, Num_Design_Variate_OneDof, limb);
    if use_constraint_penalty
        vio = Constraints_Violation_E1(Coefficient_ExTra_Current(i,:), wf, q_max_design, q_min_design, dq_max, ddq_max, DOF, Num_Design_Variate_OneDof, robot_limb, Enable_Collision_Check, PILLAR_CFG);
        if vio > 0
            Value(i) = Value(i) + penalty_scale * vio;
        end
    end
end

[~, i_best] = min(Value);
Y_total_kbese_i = zeros(1, Iteration+1);
Y_total_kbese_i(1) = Value(i_best);

% ---------- TLBO 迭代 ----------
for k = 1:Iteration

    % 教师相：向最优个体学习
    Mean_Result = sum(Coefficient_ExTra_Current, 1) / Population;
    TF = round(1 + rand);
    Difference_Mean_i = rand(1, Num_Design_Variate) .* (Coefficient_ExTra_Current(i_best, :) - TF*Mean_Result);

    for i = 1:Population
        Coefficient_ExTra_Current_New(i,:) = Coefficient_ExTra_Current(i,:) + Difference_Mean_i;
        for Joint = 1:DOF
            XI = Coefficient_ExTra_Current_New(i, (Num_Design_Variate_OneDof*(Joint-1)+1):Num_Design_Variate_OneDof*Joint);
            XI = Tradeoff_Modify(XI, wf, q_max_design(Joint), q_min_design(Joint), dq_max_design(Joint), ddq_max(Joint));
            Coefficient_ExTra_Current_New(i, (Num_Design_Variate_OneDof*(Joint-1)+1):Num_Design_Variate_OneDof*Joint) = XI;
        end
        Coefficient_ExTra_Current_New(i,:) = apply_torque_scale_E1(Coefficient_ExTra_Current_New(i,:), wf, DOF, Num_Design_Variate_OneDof, robot_limb, max_effort, TORQUE_FEASIBILITY_RATIO);
    end

    for i = 1:Population
        Value_new(i) = Objective_E1_limb(Coefficient_ExTra_Current_New(i,:), wf, Calculate_Num, Calculate_Interval, Calculate_Init, Sampling_Time, DOF, Num_Design_Variate_OneDof, limb);
        if use_constraint_penalty
            vio = Constraints_Violation_E1(Coefficient_ExTra_Current_New(i,:), wf, q_max_design, q_min_design, dq_max, ddq_max, DOF, Num_Design_Variate_OneDof, robot_limb, Enable_Collision_Check, PILLAR_CFG);
            if vio > 0
                Value_new(i) = Value_new(i) + penalty_scale * vio;
            end
        end
        if Value_new(i) < Value(i)
            Coefficient_ExTra_Current(i,:) = Coefficient_ExTra_Current_New(i,:);
            Value(i) = Value_new(i);
        end
    end

    % 学员相：随机选另一个体互学
    for i = 1:Population
        if i == 1
            Order = 2:Population;
        elseif i == Population
            Order = 1:(Population-1);
        else
            Order = [1:(i-1), (i+1):Population];
        end
        h = Order(randi(length(Order)));
        Rand_Subject = rand(1, Num_Design_Variate);

        if Value(i) <= Value(h)
            Coefficient_ExTra_Current_New(i,:) = Coefficient_ExTra_Current(i,:) + Rand_Subject .* (Coefficient_ExTra_Current(i,:) - Coefficient_ExTra_Current(h,:));
        else
            Coefficient_ExTra_Current_New(i,:) = Coefficient_ExTra_Current(i,:) + Rand_Subject .* (Coefficient_ExTra_Current(h,:) - Coefficient_ExTra_Current(i,:));
        end

        for Joint = 1:DOF
            XI = Coefficient_ExTra_Current_New(i, (Num_Design_Variate_OneDof*(Joint-1)+1):Num_Design_Variate_OneDof*Joint);
            XI = Tradeoff_Modify(XI, wf, q_max_design(Joint), q_min_design(Joint), dq_max_design(Joint), ddq_max(Joint));
            Coefficient_ExTra_Current_New(i, (Num_Design_Variate_OneDof*(Joint-1)+1):Num_Design_Variate_OneDof*Joint) = XI;
        end
        Coefficient_ExTra_Current_New(i,:) = apply_torque_scale_E1(Coefficient_ExTra_Current_New(i,:), wf, DOF, Num_Design_Variate_OneDof, robot_limb, max_effort, TORQUE_FEASIBILITY_RATIO);

        Value_new(i) = Objective_E1_limb(Coefficient_ExTra_Current_New(i,:), wf, Calculate_Num, Calculate_Interval, Calculate_Init, Sampling_Time, DOF, Num_Design_Variate_OneDof, limb);
        if use_constraint_penalty
            vio = Constraints_Violation_E1(Coefficient_ExTra_Current_New(i,:), wf, q_max_design, q_min_design, dq_max, ddq_max, DOF, Num_Design_Variate_OneDof, robot_limb, Enable_Collision_Check, PILLAR_CFG);
            if vio > 0
                Value_new(i) = Value_new(i) + penalty_scale * vio;
            end
        end
        if Value_new(i) < Value(i)
            Coefficient_ExTra_Current(i,:) = Coefficient_ExTra_Current_New(i,:);
            Value(i) = Value_new(i);
        end
    end

    [Y_total_kbese_i(k+1), i_best] = min(Value);
    fprintf('Iter %d, best cond = %.4e\n', k, Y_total_kbese_i(k+1));
end

% ---------- 输出与绘图 ----------
Best_Solution = Coefficient_ExTra_Current(i_best,:);

for Joint = 1:DOF
    Coefficient_ExTra(Joint, 1:Num_Design_Variate_OneDof) = Best_Solution((Num_Design_Variate_OneDof*(Joint-1)+1):Num_Design_Variate_OneDof*Joint);
end

figure('Name', 'TLBO E1 条件数收敛');
plot(0:Iteration, Y_total_kbese_i, 'd-b', 'LineWidth', 1.2);
xlabel('Iteration');
ylabel('Condition number');
title(sprintf('TLBO 激励优化 - E1 %s', limb));
grid on;

% 单周期轨迹示例（用于检查）
t_period = linspace(0, Tf, 500);
[Qn, dQn, ddQn] = Exciting_Trajectory(Coefficient_ExTra, t_period, wf);

figure('Name', 'TLBO E1 单周期轨迹');
for j = 1:DOF
    subplot(DOF, 3, (j-1)*3+1);
    plot(t_period, Qn(j,:)); ylabel('q (rad)'); grid on; if j==1, title('位置'); end
    subplot(DOF, 3, (j-1)*3+2);
    plot(t_period, dQn(j,:)); ylabel('qd'); grid on; if j==1, title('速度'); end
    subplot(DOF, 3, (j-1)*3+3);
    plot(t_period, ddQn(j,:)); ylabel('qdd'); grid on; if j==1, title('加速度'); end
end
xlabel('t (s)');
sgtitle(sprintf('E1 %s 最优系数对应单周期轨迹', limb));

% ---------- 末端笛卡尔轨迹 xyz ----------
if ~isempty(robot_limb)
    refPos_plot = Qn';   % N×DOF
    plot_end_effector_trajectory(robot_limb, refPos_plot, t_period(:), sprintf('TLBO %s', limb), PILLAR_CFG);
end

% ---------- 打印当前轨迹限值参数与校核参数 ----------
fprintf('\n----- 当前轨迹限值参数 -----\n');
fprintf('肢体: %s, DOF: %d\n', limb, DOF);
fprintf('位置上界 q_max (rad): '); fprintf('%g ', q_max); fprintf('\n');
fprintf('位置下界 q_min (rad): '); fprintf('%g ', q_min); fprintf('\n');
fprintf('速度限值 dq_max (rad/s): '); fprintf('%g ', dq_max); fprintf('\n');
fprintf('加速度限值 ddq_max (rad/s²): '); fprintf('%g ', ddq_max); fprintf('\n');
fprintf('峰值力矩 max_effort (N·m): '); fprintf('%g ', max_effort); fprintf('\n');
fprintf('----- 校核参数 -----\n');
fprintf('设计位置范围: 完整范围的 0.4；校核位置范围: 完整范围的 0.45（名义 q_max/q_min 为全范围，以中间点对称）\n');
fprintf('速度/加速度校核: relax_vel=relax_acc=1.0\n');
fprintf('扭矩校核限值 = 0.9*max_effort（峰值90%%）\n');
coll_str = '开启'; if ~Enable_Collision_Check, coll_str = '关闭'; end
fprintf('自碰撞检测: %s\n', coll_str);
fprintf('周期 period=%.2f s，采样 %d Hz，校核采样点数 %d\n', period, sample_frequency, Calculate_Num);
fprintf('\n');

% ---------- 检查点 1：初值 qd(0)、qdd(0) 是否接近 0 ----------
% ---------- 检查点 2：每关节四峰值及相对设计限位的裕度 ----------
% t_sample / refPos_val, refVel_val, refAcc_val 在下面校核前统一计算
t_sample = (0:Calculate_Num-1) * Sampling_Time;
[Qn_val, dQn_val, ddQn_val] = Exciting_Trajectory(Coefficient_ExTra, t_sample, wf);
refPos_val = Qn_val'; refVel_val = dQn_val'; refAcc_val = ddQn_val';
qd0 = refVel_val(1,:);
qdd0 = refAcc_val(1,:);
fprintf('----- 检查点 1：初值 qd(0), qdd(0)（应接近 0）-----\n');
for j = 1:DOF
    fprintf('  关节 %d: qd(0)=%.6e, qdd(0)=%.6e\n', j, qd0(j), qdd0(j));
end
fprintf('----- 检查点 2：每关节 max q, min q, max|qd|, max|qdd| 及裕度（设计限位）-----\n');
for j = 1:DOF
    max_q = max(refPos_val(:,j)); min_q = min(refPos_val(:,j));
    max_qd = max(abs(refVel_val(:,j))); max_qdd = max(abs(refAcc_val(:,j)));
    margin_pos_upper = q_max_design(j) - max_q;
    margin_pos_lower = min_q - q_min_design(j);
    margin_vel = dq_max(j) - max_qd;
    margin_acc = ddq_max(j) - max_qdd;
    fprintf('  关节 %d: max q=%.4f, min q=%.4f, max|qd|=%.4f, max|qdd|=%.4f\n', j, max_q, min_q, max_qd, max_qdd);
    fprintf('         裕度: 位置上=%.4f, 位置下=%.4f, 速度=%.4f, 加速度=%.4f\n', margin_pos_upper, margin_pos_lower, margin_vel, margin_acc);
end
% 检查点 3：激励质量（动力学辨识建议：总时长 30–60 s，max|qdd| > 20 rad/s²，频率约 0.2–2 Hz）
total_excitation_s = period * traj_cycle;
max_qdd_all = max(abs(refAcc_val(:)));
freq_base_Hz = 1 / period;
L_harm = 4;   % 与 Num_Design_Variate_OneDof = 2*L+3 一致
freq_max_Hz = L_harm / period;
ok_time_s = ''; if total_excitation_s >= 30 && total_excitation_s <= 60, ok_time_s = ' [OK]'; end
ok_acc_s  = ''; if max_qdd_all >= 20, ok_acc_s = ' [OK]'; end
ok_freq_s = ''; if freq_base_Hz >= 0.2 && freq_max_Hz <= 2, ok_freq_s = ' [OK]'; end
fprintf('----- 检查点 3：激励质量（辨识建议） -----\n');
fprintf('  激励总时长（周期×重复）: %.1f s  （建议 30–60 s）%s\n', total_excitation_s, ok_time_s);
fprintf('  全关节 max|qdd|: %.2f rad/s²  （建议 >20，理想 30–80）%s\n', max_qdd_all, ok_acc_s);
fprintf('  傅里叶频率范围: %.3g ～ %.3g Hz  （建议 0.2–2 Hz）%s\n', freq_base_Hz, freq_max_Hz, ok_freq_s);
fprintf('\n');

% ---------- 校核：与 run_excitation_trajectory_standalone 完全一致（位置/速度/加速度/扭矩，400 点）----------
[valid_ok, val_msg, violation_details] = validate_trajectory_E1(refPos_val, refVel_val, refAcc_val, ...
    q_max, q_min, dq_max, ddq_max, robot_limb, max_effort, [], 1.0, 1.0, Enable_Collision_Check);
if valid_ok
    fprintf('校核通过（与 standalone 同一套限位、扭矩 0.9*max_effort、自碰撞）。\n');
    % 校核通过时生成 CSV：与 plan 一致——0 位 → 过渡入 → n 周期激励 → 过渡出 → 0 位；另一腿为安全位（roll）
    csv_name = fullfile(this_dir, 'excitation_trajectory_TLBO_E1.csv');
    N_one = size(refPos_val, 1);
    if DOF == 6
        is_left = strcmp(limb, 'left_leg');
        if is_left
            other_leg_safe_6 = [0, E1_ROLL_SAFE_RIGHT_LOWER, 0, 0, 0, 0];
        else
            other_leg_safe_6 = [0, E1_ROLL_SAFE_LEFT_UPPER, 0, 0, 0, 0];
        end
        [t_full, Q12] = build_excitation_leg_trajectory_with_ramps(t_sample(:), refPos_val, limb, traj_cycle, TRANSITION_TIME, Sampling_Time, other_leg_safe_6, refVel_val, refAcc_val, dq_max);
        write_leg_trajectory_csv_plan_format(csv_name, t_full, Q12);
        % 完整轨迹关节角/速度/加速度
        if numel(t_full) >= 2
            Ts_full = t_full(2) - t_full(1);
            qd_full = [zeros(1, 12); diff(Q12) / Ts_full];
            qdd_full = [zeros(1, 12); diff(qd_full) / Ts_full];
            figure('Name', 'TLBO 激励轨迹 - 完整 12 关节 q/qd/qdd', 'Position', [100, 100, 1400, 900]);
            for j = 1:12
                subplot(12, 3, (j-1)*3 + 1);
                plot(t_full, Q12(:, j), 'b-'); grid on; ylabel('q (rad)'); if j == 1, title('位置'); end
                subplot(12, 3, (j-1)*3 + 2);
                plot(t_full, qd_full(:, j), 'r-'); grid on; ylabel('qd (rad/s)'); if j == 1, title('速度'); end
                subplot(12, 3, (j-1)*3 + 3);
                plot(t_full, qdd_full(:, j), 'k-'); grid on; ylabel('qdd (rad/s^2)'); if j == 1, title('加速度'); end
            end
            xlabel('t (s)');
            sgtitle('TLBO 激励轨迹 - 完整 12 关节 q/qd/qdd');
        end
        fprintf('已保存轨迹: %s（0位→过渡入→%d周期激励→过渡出→0位，另一腿安全位）\n', csv_name, traj_cycle);
    else
        % 非腿（如臂）：保留原表格式 t, q1..qN, qd1..qdN, qdd1..qddN
        var_names = [{'t'}, arrayfun(@(j) sprintf('q%d', j), 1:DOF, 'UniformOutput', false), ...
            arrayfun(@(j) sprintf('qd%d', j), 1:DOF, 'UniformOutput', false), ...
            arrayfun(@(j) sprintf('qdd%d', j), 1:DOF, 'UniformOutput', false)];
        if traj_cycle <= 1
            traj_csv = [t_sample(:), refPos_val, refVel_val, refAcc_val];
        else
            t_multi = (0:(N_one * traj_cycle - 1))' * Sampling_Time;
            refPos_multi = repmat(refPos_val, traj_cycle, 1);
            refVel_multi = repmat(refVel_val, traj_cycle, 1);
            refAcc_multi = repmat(refAcc_val, traj_cycle, 1);
            traj_csv = [t_multi, refPos_multi, refVel_multi, refAcc_multi];
        end
        tbl = array2table(traj_csv, 'VariableNames', var_names);
        writetable(tbl, csv_name);
        fprintf('已保存轨迹: %s（%d 周期）\n', csv_name, traj_cycle);
    end
else
    fprintf('校核未通过: %s\n', val_msg);
    if ~isempty(violation_details)
        d = violation_details(1);
        if strcmp(d.type, 'collision')
            fprintf('  首条: 时刻 %d, 自碰撞\n', d.sample_idx);
        else
            fprintf('  首条: 时刻 %d, 关节 %d, %s, 值=%.4f, 限值=%.4f, 超量=%.4f\n', d.sample_idx, d.joint_idx, d.type, d.value, d.limit, d.excess);
        end
    end
end

fprintf('===== TLBO E1 完成 =====\n');
fprintf('肢体: %s, DOF: %d, 最终条件数: %.4e\n', limb, DOF, Y_total_kbese_i(end));
fprintf('最优系数已保存在 Best_Solution 与 Coefficient_ExTra（每行一关节）。\n');
