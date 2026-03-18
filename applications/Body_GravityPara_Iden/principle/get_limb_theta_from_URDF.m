function theta = get_limb_theta_from_URDF(robot_limb, para_order)
% get_limb_theta_from_URDF  从 rigidBodyTree 提取动力学参数向量 θ（与 ReMatrix 列顺序一致）
%
% 用于辨识矩阵验收：τ_ref = inverseDynamics(robot,q,qd,qdd) 与 τ_Y = Y*θ 对比。
%
% 约定（与 inverseDynamics 一致）：
%   - URDF 中惯量由质心定义；importrobot 导入后 Inertia 已转换为 body 系（连杆原点）。
%   - 顺序：Bodies{1}..Bodies{n}，每体 10 参；para_order 1 即 [m mx my mz Ixx Ixy Ixz Iyy Iyz Izz]。
%
% 输入：
%   robot_limb - get_e1_limb_robot 返回的 rigidBodyTree（单腿 6 体）
%   para_order - 1 或 2，与 ReMatrix_E1_limb_URDF 一致
%         1: [m mx my mz Ixx Ixy Ixz Iyy Iyz Izz]
%         2: [Ixx Ixy Ixz Iyy Iyz Izz mx my mz m]
%
% 输出：
%   theta - 60×1（10×6），与 Y 的列顺序一致，满足 tau = Y*theta

n = robot_limb.NumBodies;
theta = zeros(10 * n, 1);

for i = 1:n
    body = robot_limb.Bodies{i};
    m = body.Mass;
    com = body.CenterOfMass(:);   % body 系下质心 (m)
    if numel(com) < 3
        com = [0; 0; 0];
    end
    % body 系惯量（导入时已由 URDF 质心惯量转换到连杆原点）: [Ixx Iyy Izz Iyz Ixz Ixy]
    I = body.Inertia(:);
    if numel(I) < 6
        I = [1; 1; 1; 0; 0; 0];
    end
    Ixx = I(1); Iyy = I(2); Izz = I(3);
    Iyz = I(4); Ixz = I(5); Ixy = I(6);
    mx = m * com(1);
    my = m * com(2);
    mz = m * com(3);

    if para_order == 1
        % [m mx my mz Ixx Ixy Ixz Iyy Iyz Izz]
        theta((i-1)*10 + (1:10)) = [m; mx; my; mz; Ixx; Ixy; Ixz; Iyy; Iyz; Izz];
    else
        % [Ixx Ixy Ixz Iyy Iyz Izz mx my mz m]
        theta((i-1)*10 + (1:10)) = [Ixx; Ixy; Ixz; Iyy; Iyz; Izz; mx; my; mz; m];
    end
end
end
