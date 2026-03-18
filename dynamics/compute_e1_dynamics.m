% compute_e1_dynamics  全机 18 维逆动力学（含肢体间耦合）
%
% 需全机耦合动力学时使用本函数；**推荐**新代码按肢体计算使用：
%   compute_e1_limb_dynamics(limb, q, qd, qdd)
% 参见 E1_DYNAMICS_README.md。
%
% 用法:
%   [tau, M, C, G] = compute_e1_dynamics(q, qd, qdd);
%   [M, C, G] = compute_e1_dynamics(q, qd);
%
% 输入: q, qd, qdd 为 18×1（或 1×18），顺序：左臂 1–3，右臂 4–6，左腿 7–12，右腿 13–18（与 URDF 一致）
% 输出: tau/M/C/G 为 18×1 或 18×18
% 依赖: RobotModel/get_e1_full_robot.m

function varargout = compute_e1_dynamics(q, qd, qdd)

[robot, num_joints] = get_e1_full_robot();

if isrow(q),  q = q'; end
if isrow(qd), qd = qd'; end
if length(q) ~= num_joints || length(qd) ~= num_joints
    error('关节数量不匹配: URDF 有 %d 个关节，输入为 %d', num_joints, length(q));
end

if nargin >= 3 && ~isempty(qdd)
    if isrow(qdd), qdd = qdd'; end
    tau = inverseDynamics(robot, q', qd', qdd');
    tau = tau';
    varargout{1} = tau;
    if nargout > 1
        M = massMatrix(robot, q');
        varargout{2} = M;
        G = inverseDynamics(robot, q', zeros(size(qd')), zeros(size(qdd')));
        varargout{4} = G';
        if nargout > 2
            C = tau - M * qdd - G;
            varargout{3} = C;
        end
    end
else
    M = massMatrix(robot, q');
    varargout{1} = M;
    G = inverseDynamics(robot, q', zeros(size(qd')), zeros(size(qd')));
    varargout{3} = G';
    tau_zero_acc = inverseDynamics(robot, q', qd', zeros(size(qd')));
    C = tau_zero_acc' - G';
    varargout{2} = C;
end
end
