function [range1_global, range2_global, q_fwd_act, q_bwd_act, ...
          tau_forward, tau_backward, min_distance] = ...
          align_single_joint_round_trip(time, q_joint, qd_joint, tau_joint, ...
                                        range_joint, j_act, forward_range, joint_name, varargin)
%ALIGN_SINGLE_JOINT_ROUND_TRIP
% 在给定关节的“运动段”内，对单关节往返轨迹做对齐预处理。
%
% 两种模式：
%   1) 时间对称（默认）：正向 = 段内 forward_range(1)~forward_range(2)；
%      反向 = 段尾对称区间，逆序后与正向一一对应。
%   2) 名义+精炼（可选）：若传入 nominal 的 q_act，则在实测段内用 ±refine_window
%      按关节角最近邻精炼正向/反向索引，再输出 range1_global/range2_global。
%
% 输入：
%   time          - 全局时间向量（可选使用）
%   q_joint       - 当前关节段内的 12 维关节角，N×12
%   qd_joint      - 当前关节段内的 12 维速度，N×12
%   tau_joint     - 当前关节段内的 12 维力矩，N×12
%   range_joint   - 当前关节段对应的全局行号索引（长度 N）
%   j_act         - 主动关节索引（1~12）
%   forward_range - 正向段比例 [start_ratio, end_ratio]，如 [0.10, 0.45]
%   joint_name    - 字符串，用于打印
%   varargin      - 可选：'nominal_q_act', q_vec 与 'refine_window', W（默认 20）
%
% 输出：
%   range1_global, range2_global - 正向/反向段的全局行号
%   q_fwd_act, q_bwd_act         - 主动关节正向与反向逆序后的角度
%   tau_forward, tau_backward    - 正向与反向逆序后的力矩
%   min_distance                 - 对齐误差（时间对称时为 0；精炼时为平均角差等）

N = size(q_joint, 1);
q_act = q_joint(:, j_act);

% 解析可选参数：名义 q_act、精炼窗口
nominal_q_act = [];
refine_window = 20;
for k = 1:2:length(varargin)
    if strcmpi(varargin{k}, 'nominal_q_act')
        nominal_q_act = varargin{k+1};
    elseif strcmpi(varargin{k}, 'refine_window')
        refine_window = varargin{k+1};
    end
end

use_refine = ~isempty(nominal_q_act) && isvector(nominal_q_act) && length(nominal_q_act) >= 10;

if use_refine
    % ---------- 名义 + 精炼：用名义段的正向/反向比例，在实测段内按 q 最近邻精炼
    L_nom = length(nominal_q_act);
    start_ratio = forward_range(1);
    end_ratio   = forward_range(2);
    start_nom   = max(1, round(L_nom * start_ratio));
    stop_nom    = min(L_nom, round(L_nom * end_ratio));
    if stop_nom <= start_nom
        error('关节 %s forward_range 导致名义正向段为空', joint_name);
    end
    % 名义正向索引、名义反向索引（时间对称）
    idx_fwd_nom = start_nom : stop_nom;
    idx_bwd_nom = (L_nom - stop_nom + 1) : (L_nom - start_nom + 1);
    q_nom_fwd   = nominal_q_act(idx_fwd_nom);
    q_nom_bwd   = nominal_q_act(idx_bwd_nom);
    L_fwd       = length(q_nom_fwd);
    L_bwd       = length(q_nom_bwd);
    L_meas      = N;
    W           = max(1, min(refine_window, floor(L_meas/4)));

    % 对正向：每个名义点 i 在实测段内 [j_center-W, j_center+W] 找最小 |q_act(j)-q_nom_fwd(i)|
    range1_rel = zeros(1, L_fwd);
    for i = 1:L_fwd
        j_center = 1 + round((i - 1) / max(1, L_fwd - 1) * (L_meas - 1));
        j_lo     = max(1, j_center - W);
        j_hi     = min(L_meas, j_center + W);
        [~, j_best] = min(abs(q_act(j_lo:j_hi) - q_nom_fwd(i)));
        range1_rel(i) = j_lo + j_best - 1;
    end
    % 去重并保持单调（同一实测点只保留一次，按首次出现顺序）
    [range1_rel, ~] = unique(range1_rel, 'stable');
    if length(range1_rel) < 2
        error('关节 %s 精炼后正向点数过少', joint_name);
    end

    % 对反向：同理
    range2_rel = zeros(1, L_bwd);
    for i = 1:L_bwd
        j_center = 1 + round((i - 1) / max(1, L_bwd - 1) * (L_meas - 1));
        j_lo     = max(1, j_center - W);
        j_hi     = min(L_meas, j_center + W);
        [~, j_best] = min(abs(q_act(j_lo:j_hi) - q_nom_bwd(i)));
        range2_rel(i) = j_lo + j_best - 1;
    end
    [range2_rel, ~] = unique(range2_rel, 'stable');
    if length(range2_rel) < 2
        error('关节 %s 精炼后反向点数过少', joint_name);
    end

    % 取相同长度以便力矩平均
    n = min(length(range1_rel), length(range2_rel));
    range1_rel = range1_rel(1:n);
    range2_rel = range2_rel(1:n);
    range1_global = range_joint(range1_rel);
    range2_global = range_joint(range2_rel);
    q_fwd_act     = q_act(range1_rel);
    q_bwd_act     = flip(q_act(range2_rel), 1);
    tau_forward   = tau_joint(range1_rel, :);
    tau_backward  = flip(tau_joint(range2_rel, :), 1);
    % 对齐误差：正向与反向逆序后的角度差
    q_bwd_flip = flip(q_act(range2_rel), 1);
    n_min = min(length(q_fwd_act), length(q_bwd_flip));
    if n_min > 0
        min_distance = norm(q_fwd_act(1:n_min) - q_bwd_flip(1:n_min), 2) / sqrt(n_min);
    else
        min_distance = nan;
    end
    fprintf('  [align] %s: 精炼模式, 正向 %d 点, 反向 %d 点, 平均角差=%.6f rad\n', ...
        joint_name, length(range1_rel), length(range2_rel), min_distance);
else
    % ---------- 时间对称：无搜索，直接按比例取正向与对称反向
    start_ratio = forward_range(1);
    end_ratio   = forward_range(2);
    start_point = max(1, fix(N * start_ratio));
    stop_point  = min(N, fix(N * end_ratio));
    if stop_point <= start_point
        error('关节 %s forward_range 导致正向段为空，请检查参数', joint_name);
    end
    range1 = start_point : stop_point;
    n      = length(range1);
    % 反向段：时间对称 (N - stop_point + 1) : (N - start_point + 1)
    range2_rel = (N - stop_point + 1) : (N - start_point + 1);
    if any(range2_rel < 1) || any(range2_rel > N)
        error('关节 %s 反向段越界', joint_name);
    end

    range1_global = range_joint(range1);
    range2_global = range_joint(range2_rel);
    q_fwd_act     = q_act(range1);
    q_bwd_act     = flip(q_act(range2_rel), 1);
    tau_forward   = tau_joint(range1, :);
    tau_backward  = flip(tau_joint(range2_rel, :), 1);
    min_distance  = 0;
    fprintf('  [align] %s: 时间对称, 段内正向 %d~%d, 反向 %d~%d\n', ...
        joint_name, range1(1), range1(end), range2_rel(1), range2_rel(end));
end

end
