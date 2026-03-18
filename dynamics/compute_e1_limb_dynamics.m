function varargout = compute_e1_limb_dynamics(limb, q, qd, qdd)
% compute_e1_limb_dynamics  按肢体单独计算 E1 逆动力学（每条腿/每条臂）
%
% 每条腿 6 自由度、每条臂 3 自由度；q/qd/qdd 及 M、C、G、tau 均为当前肢体维度。
%
% 用法:
%   tau = compute_e1_limb_dynamics(limb, q, qd, qdd)
%   [tau, M, C, G] = compute_e1_limb_dynamics(limb, q, qd, qdd)
%   [M, C, G] = compute_e1_limb_dynamics(limb, q, qd)   % 仅质量矩阵与 C、G，无 qdd
%
% 输入:
%   limb - 肢体名称: 'left_leg' | 'right_leg' | 'left_arm' | 'right_arm'
%   q    - 关节位置 (rad)，列向量，长度 6(腿) 或 3(臂)
%   qd   - 关节速度 (rad/s)，同长度
%   qdd  - 关节加速度 (rad/s^2)，同长度
%
% 输出:
%   tau - 关节力矩 (N·m)，与 q 同维度
%   M   - 质量矩阵，n×n，n 为肢体自由度
%   C   - 科氏/向心项，n×1
%   G   - 重力项，n×1
%
% 依赖: E1.urdf，Robotics System Toolbox (inverseDynamics, massMatrix)

[robot_limb, n] = get_e1_limb_robot(limb);

if isrow(q),  q = q';  end
if isrow(qd), qd = qd'; end
if length(q) ~= n || length(qd) ~= n
    error('compute_e1_limb_dynamics: 肢体 %s 自由度为 %d，输入长度为 %d/%d', limb, n, length(q), length(qd));
end

if nargin >= 4 && ~isempty(qdd)
    if isrow(qdd), qdd = qdd'; end
    if length(qdd) ~= n
        error('compute_e1_limb_dynamics: qdd 长度应为 %d', n);
    end
    % 逆动力学 tau = M*qdd + C + G
    tau = inverseDynamics(robot_limb, q', qd', qdd');
    tau = tau';
    varargout{1} = tau;
    if nargout > 1
        M = massMatrix(robot_limb, q');
        varargout{2} = M;
        G = inverseDynamics(robot_limb, q', zeros(1,n), zeros(1,n));
        G = G';
        varargout{4} = G;
        if nargout > 2
            C = tau - M * qdd - G;
            varargout{3} = C;
        end
    end
else
    % 仅 q, qd：返回 M, C, G（无 tau）
    M = massMatrix(robot_limb, q');
    varargout{1} = M;
    G = inverseDynamics(robot_limb, q', zeros(1,n), zeros(1,n));
    G = G';
    varargout{3} = G;
    tau_zero_acc = inverseDynamics(robot_limb, q', qd', zeros(1,n));
    C = tau_zero_acc' - G;
    varargout{2} = C;
end
end
