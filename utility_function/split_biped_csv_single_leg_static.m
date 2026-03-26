function out_paths = split_biped_csv_single_leg_static(csv_in, varargin)
% split_biped_csv_single_leg_static  双腿诊断 CSV → 两条「单腿名义」CSV
%
% 与激励生成一致：静止腿在整个时段内为**单一固定姿态** other_leg_safe_6（非沿参考轨迹逐点插值）。
% 默认值与 run_excitation_trajectory_standalone / run_TLBO_Excitation_E1 相同：
%   左腿为动腿 → 右腿静止 [0, E1_ROLL_SAFE_RIGHT_LOWER, 0, 0, 0, 0]，LOWER = -0.436 rad
%   右腿为动腿 → 左腿静止 [0, E1_ROLL_SAFE_LEFT_UPPER,  0, 0, 0, 0]，UPPER  =  0.436 rad
% （与 build_excitation_leg_trajectory_with_ramps 中 repmat(other_leg_safe_6, N, 1) 同义。）
%
% 输入：csv_in — read_leg_joint_csv 兼容格式（仅此一个文件）。
%   若日志里「轨迹」记的是期望（指令）而非编码器反馈，可将 use_cmd_for_desired 设为 true：
%   在写文件前把 cmd_* 拷到 pos_*（两腿），便于后续仍按 pos 读期望的脚本一致。
%
% 输出 CSV 与激励生成相同（write_leg_trajectory_csv_plan_format）：
%   表头 time, leg_l1_joint, ..., leg_r6_joint；数值 time 用 %.5f、关节用 %.6f
%   *_stillleg_safe.csv — 静止腿恒为上述安全位（或 still_leg_q 自定义）
%   *_stillleg_zero.csv — 静止腿全 0
%   仅写出关节位置 Q12；无 cmd/vel/tau 列（与 plan 激励轨迹文件一致）。
%
% 名值对：
%   'active_limb' - 'left_leg'（默认）| 'right_leg'
%   'still_leg_q' - 1×6，静止腿固定关节角 (rad)；默认 [] 表示用 E1 激励同款 safe 位
%   'out_dir'     - 输出目录，默认与 csv_in 同目录
%   'use_cmd_for_desired' - 默认 false；true 时读入后 pos_leg_* := cmd_leg_*（期望在 cmd 列时）
%   'do_plot'     - 默认 false；true 时写文件前预览（记录轨迹 vs 恒值 safe vs 零）
%   'skip_write'  - 默认 false；true 时只绘图
%
% 返回 struct：.safe, .zero 为完整路径
%
% 依赖：read_leg_joint_csv, get_e1_leg_joint_names, write_leg_trajectory_csv_plan_format

p = inputParser;
addParameter(p, 'still_leg_q', [], @(x) isnumeric(x) && (isempty(x) || numel(x) == 6));
addParameter(p, 'active_limb', 'left_leg', @(s) ischar(s) || isstring(s));
addParameter(p, 'out_dir', '', @(s) ischar(s) || isstring(s));
addParameter(p, 'use_cmd_for_desired', false, @islogical);
addParameter(p, 'do_plot', false, @islogical);
addParameter(p, 'skip_write', false, @islogical);
parse(p, varargin{:});
still_leg_q = p.Results.still_leg_q(:).';
active_limb = char(p.Results.active_limb);
out_dir = char(p.Results.out_dir);
use_cmd_for_desired = p.Results.use_cmd_for_desired;
do_plot = p.Results.do_plot;
skip_write = p.Results.skip_write;

if ~isfile(csv_in)
    error('split_biped_csv_single_leg_static: 输入不存在: %s', csv_in);
end

data = read_leg_joint_csv(csv_in);
if use_cmd_for_desired
    data.pos_leg_l = data.cmd_leg_l;
    data.pos_leg_r = data.cmd_leg_r;
end

if isempty(out_dir)
    out_dir = fileparts(csv_in);
end
if ~skip_write && ~isfolder(out_dir)
    mkdir(out_dir);
end

[~, base, ext] = fileparts(csv_in);
if isempty(ext), ext = '.csv'; end

limb_l = lower(strtrim(active_limb));
if contains(limb_l, 'left')
    i_act = 'l';
elseif contains(limb_l, 'right')
    i_act = 'r';
else
    error('active_limb 需为 left_leg 或 right_leg');
end

if isempty(still_leg_q)
    q_safe = e1_default_other_leg_safe_6(i_act);
else
    q_safe = still_leg_q(:).';
end

d_safe = apply_still_pose(data, q_safe, i_act);
d_zero = apply_still_pose(data, zeros(1, 6), i_act);

out_paths.safe = fullfile(out_dir, [base, '_stillleg_safe', ext]);
out_paths.zero = fullfile(out_dir, [base, '_stillleg_zero', ext]);

if do_plot
    [q_sta_meas, jnames_sta, sta_label] = get_still_leg_q_and_names(data, i_act);
    plot_split_still_leg_preview(data.time(:), q_sta_meas, q_safe, jnames_sta, ...
        sprintf('静止腿预览 — 记录为期望轨迹 (%s)', sta_label));
end

if ~skip_write
    t_out = d_safe.time(:);
    Q12_safe = [d_safe.pos_leg_l, d_safe.pos_leg_r];
    Q12_zero = [d_zero.pos_leg_l, d_zero.pos_leg_r];
    write_leg_trajectory_csv_plan_format(out_paths.safe, t_out, Q12_safe);
    write_leg_trajectory_csv_plan_format(out_paths.zero, t_out, Q12_zero);
end

% --- nested ---
    function d_out = apply_still_pose(d_in, q6, act)
        d_out = d_in;
        N = numel(d_in.time);
        q_new = repmat(q6(:).', N, 1);
        cmd_new = q_new;
        qd_new = zeros(N, 6);
        tau_new = zeros(N, 6);
        if strcmp(act, 'l')
            d_out.pos_leg_r = q_new;
            d_out.cmd_leg_r = cmd_new;
            d_out.vel_leg_r = qd_new;
            d_out.torque_leg_r = tau_new;
        else
            d_out.pos_leg_l = q_new;
            d_out.cmd_leg_l = cmd_new;
            d_out.vel_leg_l = qd_new;
            d_out.torque_leg_l = tau_new;
        end
        if isfield(d_out, 'acc_leg_l')
            if strcmp(act, 'l')
                d_out.acc_leg_r = zeros(N, 6);
            else
                d_out.acc_leg_l = zeros(N, 6);
            end
        end
    end
end

function q6 = e1_default_other_leg_safe_6(act)
% act: 'l' = 左腿动，静止腿为右腿；'r' = 右腿动，静止腿为左腿
E1_ROLL_SAFE_RIGHT_LOWER = -0.436;
E1_ROLL_SAFE_LEFT_UPPER  = 0.436;
if strcmp(act, 'l')
    q6 = [0, E1_ROLL_SAFE_RIGHT_LOWER, 0, 0, 0, 0];
else
    q6 = [0, E1_ROLL_SAFE_LEFT_UPPER, 0, 0, 0, 0];
end
end

function [q_sta, jnames_sta, label] = get_still_leg_q_and_names(data, act)
[~, ~, jnames] = get_e1_leg_joint_names();
if strcmp(act, 'l')
    q_sta = data.pos_leg_r;
    jnames_sta = jnames(7:12);
    label = '右腿静止（左腿为动腿，激励同款 safe：r2=lower）';
else
    q_sta = data.pos_leg_l;
    jnames_sta = jnames(1:6);
    label = '左腿静止（右腿为动腿，激励同款 safe：l2=upper）';
end
end

function plot_split_still_leg_preview(t, q_recorded, q_safe_vec, jnames_sta, fig_title)
N = numel(t);
figure('Name', fig_title, 'Position', [80 80 900 640]);
for j = 1:6
    subplot(3, 2, j);
    hold on;
    plot(t, q_recorded(:, j), 'Color', [0.65 0.65 0.65], 'LineWidth', 0.8, 'DisplayName', '记录(原静止腿)');
    plot(t, zeros(N, 1), 'k--', 'LineWidth', 0.6, 'DisplayName', '零位方案');
    plot(t, repmat(q_safe_vec(j), N, 1), 'b', 'LineWidth', 1.0, 'DisplayName', '激励 safe 恒值');
    xlabel('t (s)');
    ylabel('q (rad)');
    title(jnames_sta{j}, 'Interpreter', 'none');
    legend('Location', 'best', 'FontSize', 7);
    grid on;
    hold off;
end
sgtitle(fig_title, 'Interpreter', 'none');
drawnow;
end
