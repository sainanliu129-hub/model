function [t_full, Q12] = build_excitation_leg_trajectory_with_ramps(t_one, refPos_one, limb, n_cycles, transition_time, Ts, other_leg_safe_6, refVel_one, refAcc_one, max_velocity_ramp)
% build_excitation_leg_trajectory_with_ramps  构建「0 位 → 静止腿到安全位 → 激励腿 n 周期 → 激励腿回零 → 静止腿回零」的 12 关节轨迹
%
% 左右腿不同时动；所有拼接段采用五次多项式，使每段起止处速度、加速度均为 0，变化均匀。
% 若传入 refVel_one、refAcc_one，则在激励段首尾各加一段平滑衔接，使与过渡段衔接处速度、加速度为 0。
% 为获得最佳衔接，单周期 refPos 建议配合 refVel(1)=refVel(end)=0、refAcc(1)=refAcc(end)=0 使用。
%
% 输入:
%   t_one            - 单周期时间列 (N_one×1)，单位 s
%   refPos_one       - 单周期位置 (N_one×6)，当前激励腿
%   limb             - 'left_leg' | 'right_leg'
%   n_cycles        - 激励周期数
%   transition_time  - 每段过渡时长 (s)，建议 ≥1.0，增大可降低过渡段最大速度
%   Ts               - 采样周期 (s)
%   other_leg_safe_6 - 另一腿安全位 (1×6)
%   refVel_one       - 可选，单周期速度 (N_one×6)；传入则激励段首尾做平滑衔接
%   refAcc_one       - 可选，单周期加速度 (N_one×6)；与 refVel_one 同时传入
%   max_velocity_ramp- 可选，激励腿 6 关节过渡段速度上限 (6×1 或 1×6) rad/s；传入则按限速自适应延长过渡时长
% 输出:
%   t_full - 总时间列 (N×1)，从 0 开始
%   Q12    - 关节位置 (N×12)，列顺序 [左腿 1..6, 右腿 1..6]

if nargin < 8, refVel_one = []; end
if nargin < 9, refAcc_one = []; end
if nargin < 10, max_velocity_ramp = []; end
use_vel_acc = ~isempty(refVel_one) && ~isempty(refAcc_one) && ...
    size(refVel_one,1) == size(refPos_one,1) && size(refAcc_one,1) == size(refPos_one,1);

other_leg_safe_6 = other_leg_safe_6(:)';
N_one = size(refPos_one, 1);
n_trans = max(round(transition_time / Ts), 20);   % 至少 20 点使过渡充分、最大速度不过大
T_seg = (n_trans - 1) * Ts;

zero_6 = zeros(1, 6);
if strcmp(limb, 'left_leg')
    state_other_safe = [zero_6, other_leg_safe_6];
    state_exc_first  = [refPos_one(1,:), other_leg_safe_6];
    state_exc_last   = [refPos_one(end,:), other_leg_safe_6];
else
    state_other_safe = [other_leg_safe_6, zero_6];
    state_exc_first  = [other_leg_safe_6, refPos_one(1,:)];
    state_exc_last   = [other_leg_safe_6, refPos_one(end,:)];
end

% 可选：按过渡段速度上限自适应延长过渡时长（五次多项式 max qd 系数约 1.875）
if ~isempty(max_velocity_ramp) && numel(max_velocity_ramp) >= 6
    vmax = reshape(max_velocity_ramp(1:6), 6, 1);
    delta_2 = abs(refPos_one(1, :))';
    delta_4 = abs(refPos_one(end, :))';
    T_req_2 = 0; T_req_4 = 0;
    for j = 1:6
        if vmax(j) > 1e-6
            T_req_2 = max(T_req_2, delta_2(j) * 1.875 / vmax(j));
            T_req_4 = max(T_req_4, delta_4(j) * 1.875 / vmax(j));
        end
    end
    T_min = max(T_req_2, T_req_4);
    T_seg_eff = max(T_seg, T_min);
    n_trans = max(20, round(T_seg_eff / Ts) + 1);
    T_seg = (n_trans - 1) * Ts;
end

% 五次多项式 phi(s): phi(0)=0, phi(1)=1, phi'(0)=phi'(1)=0, phi''(0)=phi''(1)=0
quintic_blend = @(s) 10*min(max(s,0),1).^3 - 15*min(max(s,0),1).^4 + 6*min(max(s,0),1).^5;
s_unif = (0:n_trans-1)' / max(n_trans-1, 1);   % 严格 0 到 1，保证起止速度/加速度为 0

t_current = 0;

% 1) 静止腿到安全位（激励腿保持 0）
t_1 = (0:n_trans-1)' * Ts + t_current;
phi_1 = quintic_blend((t_1 - t_current) / T_seg);
Q_1 = zeros(n_trans, 12);
for j = 1:12
    Q_1(:, j) = state_other_safe(j) * phi_1;
end
t_current = t_1(end) + Ts;

% 2) 激励腿从 0 到首点（静止腿保持安全位）
t_2 = t_current + (0:n_trans-1)' * Ts;
phi_2 = quintic_blend((t_2 - t_2(1)) / T_seg);
Q_2 = zeros(n_trans, 12);
for j = 1:12
    Q_2(:, j) = state_other_safe(j) + (state_exc_first(j) - state_other_safe(j)) * phi_2;
end
t_current = t_2(end) + Ts;

% 3) 激励段（可选：首尾加平滑衔接，使与段2/段4 衔接处速度、加速度为 0）
n_blend = min(max(round(0.05 / Ts), 8), floor(N_one/4));   % 约 50ms 或 8~N_one/4 点
if use_vel_acc && n_blend >= 4 && N_one > 2*n_blend + 2
    % 衔接入：从 (refPos(1), 0, 0) 到 (refPos(n_blend+1), refVel(n_blend+1), refAcc(n_blend+1))
    T_b = n_blend * Ts;
    t_b = (0:n_blend)' * Ts;
    blend_in = zeros(n_blend+1, 6);
    for j = 1:6
        p0 = refPos_one(1,j); v0 = 0; a0 = 0;
        p1 = refPos_one(n_blend+1,j); v1 = refVel_one(n_blend+1,j); a1 = refAcc_one(n_blend+1,j);
        [a3,a4,a5] = quintic_coeffs(T_b, p0,v0,a0, p1,v1,a1);
        blend_in(:,j) = p0 + v0*t_b + a0/2*t_b.^2 + a3*t_b.^3 + a4*t_b.^4 + a5*t_b.^5;
    end
    % 衔接出：从 (refPos(end-n_blend), refVel(end-n_blend), refAcc(end-n_blend)) 到 (refPos(end), 0, 0)
    blend_out = zeros(n_blend+1, 6);
    for j = 1:6
        p0 = refPos_one(end-n_blend,j); v0 = refVel_one(end-n_blend,j); a0 = refAcc_one(end-n_blend,j);
        p1 = refPos_one(end,j); v1 = 0; a1 = 0;
        [a3,a4,a5] = quintic_coeffs(T_b, p0,v0,a0, p1,v1,a1);
        blend_out(:,j) = p0 + v0*t_b + a0/2*t_b.^2 + a3*t_b.^3 + a4*t_b.^4 + a5*t_b.^5;
    end
    % 第一周期：blend_in + refPos(n_blend+2 : end)
    % 中间周期：完整 refPos_one
    % 最后周期：refPos(1 : end-n_blend-1) + blend_out
    % n_cycles==1 时只输出一个周期：blend_in + 中间段 + blend_out，否则会拼成两段（第一周期尾+最后周期头）导致看起来像两周期
    if n_cycles == 1
        N_exc = (n_blend+1) + (N_one - 2*n_blend - 2) + (n_blend+1);   % = N_one
        refPos_multi = [blend_in; refPos_one(n_blend+2 : end-n_blend-1, :); blend_out];
    else
        N_mid = (n_cycles - 1) * N_one;
        N_exc = (n_blend+1) + (N_one - n_blend - 1) + N_mid + (N_one - n_blend - 1) + (n_blend+1);
        refPos_multi = zeros(N_exc, 6);
        refPos_multi(1:n_blend+1, :) = blend_in;
        refPos_multi(n_blend+2 : n_blend+1 + (N_one - n_blend - 1), :) = refPos_one(n_blend+2:end, :);
        idx = n_blend+1 + (N_one - n_blend - 1) + 1;
        for c = 1:n_cycles-2
            refPos_multi(idx:idx+N_one-1, :) = refPos_one;
            idx = idx + N_one;
        end
        refPos_multi(idx:idx+N_one-n_blend-2, :) = refPos_one(1:end-n_blend-1, :);
        idx = idx + N_one - n_blend - 1;
        refPos_multi(idx:idx+n_blend, :) = blend_out;
    end
else
    refPos_multi = repmat(refPos_one, n_cycles, 1);
    N_exc = size(refPos_multi, 1);
end

t_3 = t_current + (0:N_exc-1)' * Ts;
other_mat = repmat(other_leg_safe_6, N_exc, 1);
if strcmp(limb, 'left_leg')
    Q_3 = [refPos_multi, other_mat];
else
    Q_3 = [other_mat, refPos_multi];
end
t_current = t_3(end) + Ts;

% 4) 激励腿从末点回 0（静止腿保持安全位）
t_4 = t_current + (0:n_trans-1)' * Ts;
phi_4 = quintic_blend((t_4 - t_4(1)) / T_seg);
Q_4 = zeros(n_trans, 12);
for j = 1:12
    Q_4(:, j) = state_exc_last(j) + (state_other_safe(j) - state_exc_last(j)) * phi_4;
end
t_current = t_4(end) + Ts;

% 5) 静止腿从安全位回 0
t_5 = t_current + (0:n_trans-1)' * Ts;
phi_5 = quintic_blend((t_5 - t_5(1)) / T_seg);
Q_5 = zeros(n_trans, 12);
for j = 1:12
    Q_5(:, j) = state_other_safe(j) * (1 - phi_5);
end

t_full = [t_1; t_2; t_3; t_4; t_5];
Q12 = [Q_1; Q_2; Q_3; Q_4; Q_5];
end

function [a3, a4, a5] = quintic_coeffs(T, p0, v0, a0, p1, v1, a1)
% 五次多项式 q(t)=p0+v0*t+a0/2*t^2+a3*t^3+a4*t^4+a5*t^5 满足 q(T)=p1, q'(T)=v1, q''(T)=a1
% 解 A*[a3;a4;a5]=b
if T <= 0
    a3 = 0; a4 = 0; a5 = 0; return
end
d = p1 - p0 - v0*T - a0/2*T^2;
r = v1 - v0 - a0*T;
s = a1 - a0;
A = [T^3,   T^4,   T^5; ...
     3*T^2, 4*T^3, 5*T^4; ...
     6*T,   12*T^2, 20*T^3];
x = A \ [d; r; s];
a3 = x(1); a4 = x(2); a5 = x(3);
end
