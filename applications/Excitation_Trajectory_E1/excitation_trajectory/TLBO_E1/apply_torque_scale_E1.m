function XI_out = apply_torque_scale_E1(XI, wf, DOF, Num_Design_Variate_OneDof, robot_limb, max_effort, torque_ratio)
% apply_torque_scale_E1  对候选解系数施加扭矩缩放因子 λ_τ，使 max|τ_j| ≤ torque_ratio*max_effort
%
% 在 Tradeoff_Modify（位置/速度/加速度可行化）之后调用，作为可行化流程的一步。
% 若 max|τ| 超限，则 λ_τ = min_j (effort_limit(j) / max_t|τ_j(t)|)，整行系数 XI_out = λ_τ*XI。
% 整轨迹 (q,dq,ddq) 与 τ 同比例缩放，故缩放后仍满足位置/速度/加速度限位。
%
% 输入:
%   XI  - 1×(DOF*Num_Design_Variate_OneDof) 当前个体系数
%   wf, DOF, Num_Design_Variate_OneDof - 周期与维度
%   robot_limb, max_effort - 刚体树与峰值力矩 (DOF×1)；任一为空则跳过，返回 XI
%   torque_ratio - 校核限值比例，默认 0.9（与 validate_trajectory_E1 一致）
% 输出: XI_out - 缩放后的系数行（λ_τ≤1 时 XI_out=λ_τ*XI，否则 XI_out=XI）

if nargin < 7 || isempty(torque_ratio), torque_ratio = 0.9; end
XI_out = XI;
if isempty(robot_limb) || isempty(max_effort) || ~exist('inverseDynamics', 'file')
    return;
end

max_effort = max_effort(:);
if numel(max_effort) < DOF
    return;
end
max_effort = max_effort(1:DOF);
effort_limit = torque_ratio * max_effort;

Tf = 2*pi/wf;
t = 0:0.01:Tf;
Coefficient_ExTra = zeros(DOF, Num_Design_Variate_OneDof);
for Joint = 1:DOF
    Coefficient_ExTra(Joint,:) = XI((Joint-1)*Num_Design_Variate_OneDof+1 : Joint*Num_Design_Variate_OneDof);
end
[q, dq, ddq] = Exciting_Trajectory(Coefficient_ExTra, t, wf);
% q,dq,ddq 为 DOF×N；每列一个时刻
N = size(q, 2);
max_tau = zeros(DOF, 1);
try
    for ti = 1:N
        q_c = q(:, ti);
        qd_c = dq(:, ti);
        qdd_c = ddq(:, ti);
        tau = inverseDynamics(robot_limb, q_c, qd_c, qdd_c);
        tau = tau(:);
        for j = 1:DOF
            if abs(tau(j)) > max_tau(j)
                max_tau(j) = abs(tau(j));
            end
        end
    end
catch
    return;
end

% λ_τ = min(1, min_j effort_limit(j)/max_tau(j))
ratio_j = effort_limit ./ max(max_tau, 1e-10);
lambda_tau = min(1, min(ratio_j));
if lambda_tau < 1
    XI_out = lambda_tau * XI;
end
end
