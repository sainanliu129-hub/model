function robot_out = set_limb_theta_to_URDF(robot_limb, pi_vec, para_order, force_psd)
% set_limb_theta_to_URDF  用全参向量 π 更新 rigidBodyTree 的动力学参数（不修改原树）
%
% 与 get_limb_theta_from_URDF 互逆：先 get 得到 π，修改 π 后再 set 回树，则
% inverseDynamics/forwardDynamics(robot_out, ...) 使用更新后的参数。
% 用于：用 π_rec/π_phys 更新 limb 模型后，用 forwardDynamics 做正动力学，与 forward_dynamics_full 交叉验证。
% π_rec 仅保证力矩等价，惯量可能非半正定；写入前可将每体 3×3 惯量投影到最近 PSD（见 force_psd）。
% π_phys 虽有物理约束，实践中仍可能因非对角元或优化数值产生非 PSD，建议同样做投影（或 force_psd=true）。
%
% 约定（与 get_limb_theta_from_URDF、ReMatrix 一致）：
%   para_order 1: 每体 10 参 [m mx my mz Ixx Ixy Ixz Iyy Iyz Izz]
%   para_order 2: 每体 10 参 [Ixx Ixy Ixz Iyy Iyz Izz mx my mz m]
%   rigidBody.Inertia 为 [Ixx Iyy Izz Iyz Ixz Ixy]（Toolbox 约定）
%
% 输入：
%   robot_limb - get_e1_limb_robot 返回的 rigidBodyTree（单腿/单臂）
%   pi_vec     - 10*n×1 全参，n = robot_limb.NumBodies
%   para_order - 1 或 2
%   force_psd  - true（默认）：惯量写入前投影到半正定，适合 π_rec；false：不投影，适合 π_phys/π_cad（若已 PSD）
%
% 输出：
%   robot_out  - 动力学参数更新后的副本（原 robot_limb 不变）

if nargin < 3
    para_order = 1;
end
if nargin < 4
    force_psd = true;
end

robot_out = copy(robot_limb);
n = robot_out.NumBodies;
pi_vec = pi_vec(:);
if numel(pi_vec) ~= 10 * n
    error('set_limb_theta_to_URDF: pi_vec 长度 %d 与 Bodies 数 %d 不一致（应为 10*n）。', numel(pi_vec), n);
end

for i = 1:n
    body = robot_out.Bodies{i};
    idx = (i - 1) * 10 + (1:10);
    p = pi_vec(idx);
    if para_order == 1
        m = p(1);
        mx = p(2); my = p(3); mz = p(4);
        Ixx = p(5); Ixy = p(6); Ixz = p(7);
        Iyy = p(8); Iyz = p(9); Izz = p(10);
    else
        Ixx = p(1); Ixy = p(2); Ixz = p(3);
        Iyy = p(4); Iyz = p(5); Izz = p(6);
        mx = p(7); my = p(8); mz = p(9);
        m = p(10);
    end
    body.Mass = max(m, 1e-12);   % Toolbox 要求质量为正
    if m > 1e-12
        body.CenterOfMass = [mx; my; mz] / m;
    else
        body.CenterOfMass = [0; 0; 0];
    end
    % 惯量矩阵须为对称半正定。force_psd 时投影到最近 PSD（π_rec 常用）；否则直接写入（π_phys/π_cad 已约束时可用）
    if force_psd
        I3 = [Ixx Ixy Ixz; Ixy Iyy Iyz; Ixz Iyz Izz];
        I3 = (I3 + I3.') / 2;
        [V, D] = eig(I3);
        d = max(diag(D), 1e-6);
        I3_psd = V * diag(d) * V';
        Ixx = I3_psd(1,1); Iyy = I3_psd(2,2); Izz = I3_psd(3,3);
        Iyz = I3_psd(2,3); Ixz = I3_psd(1,3); Ixy = I3_psd(1,2);
    end
    % rigidBody.Inertia: [Ixx Iyy Izz Iyz Ixz Ixy]
    body.Inertia = [Ixx; Iyy; Izz; Iyz; Ixz; Ixy];
end

end
