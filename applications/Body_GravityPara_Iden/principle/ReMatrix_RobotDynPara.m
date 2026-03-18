function Y = ReMatrix_RobotDynPara(Robot, qfull, qdfull, qddfull, output_type, para_order)
% ReMatrix_RobotDynPara  6DOF 机械臂全动力学参数观测矩阵（辨识矩阵）
%
% 给定运动状态 q,qd,qdd，计算全动力学参数对应的观测矩阵 Y_Fbase / Y_Tauj。
%
% 输入：
%   Robot    - 结构体，含 .DH、.gravity（DH 参数与重力加速度方向）
%   qfull    - N×6 关节位置 (rad)
%   qdfull   - N×6 关节速度 (rad/s)
%   qddfull  - N×6 关节加速度 (rad/s^2)
%   output_type - 输出形式：
%         1: 60 参数观测矩阵（基于电流，映射至关节力矩）
%         2: 66 参数观测矩阵（基于电流，含转子惯量 Ia，映射至关节力矩）
%         3: 60 参数观测矩阵（基于底座，映射至底座六维力）
%   para_order - 全动力学参数顺序：
%         1: [m mx my mz Ixx Ixy Ixz Iyy Iyz Izz (Ia)]
%         2: [Ixx Ixy Ixz Iyy Iyz Izz mx my mz m (Ia)]
%     Y_Fbase 不含 Ia；仅 Y_Tauj 含 Ia。
%
% 输出：
%   Y - 观测矩阵。Y_Tauj*theta = Tau；Y_Fbase*theta = F（底座六维力）
%
% 依赖（本仓库内未提供，需自行在路径上）：
%   calMatrix_kinHomTrans(q, Robot.DH) 返回 T 为 1×6 cell，T{i} 为 4×4 齐次变换。
%   通常与 get_cad_model_para 等同属一套 DH 机器人库（如 aubo）。
% 本文件内提供 skew、skewstar 子函数。
%
% 更改构型后是否还能用：
%   - 若新构型仍是 6DOF、且用 DH 描述（Robot.DH + Robot.gravity）：只需更新 Robot.DH，
%     并保证 calMatrix_kinHomTrans 能正确根据该 DH 计算 T，则本函数可直接用。
%   - 若新构型仅用 URDF（如本仓库 E1），没有 Robot.DH：本函数不能直接用，需在外部
%     为 URDF 推导/导出 DH 并实现 calMatrix_kinHomTrans，或另写基于 URDF/rigidBodyTree 的观测矩阵。
%
% 版本 V1.0 2024/3/19；V2.0 修改 2024/3/19

if nargin == 4
    output_type = 1;
end
if nargin < 6
    para_order = 1;
end

Y = [];
N = size(qfull, 1);
gravity = Robot.gravity;

for iPoint = 1 : N
    q = qfull(iPoint, :);
    qd = qdfull(iPoint, :);
    qdd = qddfull(iPoint, :);

    [T, ~, ~] = calMatrix_kinHomTrans(q, Robot.DH);
    w0 = [0 0 0]'; wd0 = [0 0 0]'; vd0 = -[gravity(1); gravity(2); gravity(3)];
    z = [0 0 1]';

    % 连杆 1 到 6 向外运动学迭代
    R = cell(1, 6);
    p = cell(1, 6);
    w = cell(1, 6);
    wd = cell(1, 6);
    vd = cell(1, 6);
    for i = 1:6
        R{i} = T{i}(1:3, 1:3);
        p{i} = T{i}(1:3, 4);
        if i == 1
            w{i} = R{i}' * w0 + qd(i) * z;
            wd{i} = R{i}' * wd0 + qdd(i) * z + cross(R{i}' * w0, qd(i) * z);
            vd{i} = R{i}' * (vd0 + cross(w0, cross(w0, p{i})) + cross(wd0, p{i}));
        else
            w{i} = R{i}' * w{i-1} + qd(i) * z;
            wd{i} = R{i}' * wd{i-1} + qdd(i) * z + cross(R{i}' * w{i-1}, qd(i) * z);
            vd{i} = R{i}' * (vd{i-1} + cross(w{i-1}, cross(w{i-1}, p{i})) + cross(wd{i-1}, p{i}));
        end
    end

    % 计算 A
    A = cell(6, 1);
    if para_order == 1
        for i = 1:6
            A{i, 1} = [vd{i}, skew(wd{i}) + skew(w{i})*skew(w{i}), zeros(3, 6);
                       zeros(3, 1), -skew(vd{i}), skewstar(wd{i}) + skew(w{i})*skewstar(w{i})];
        end
    else
        for i = 1:6
            A{i, 1} = [zeros(3, 6), skew(wd{i}) + skew(w{i})*skew(w{i}), vd{i};
                       skewstar(wd{i}) + skew(w{i})*skewstar(w{i}), -skew(vd{i}), zeros(3, 1)];
        end
    end

    % 计算 HT
    HT = cell(6, 1);
    for i = 1:6
        HT{i, 1} = zeros(6, 6);
    end
    for i = 1:5
        HT{i, 1}(1:3, 1:3) = R{i+1};
        HT{i, 1}(4:6, 1:3) = skew(p{i+1}) * R{i+1};
        HT{i, 1}(4:6, 4:6) = R{i+1};
    end

    % 计算 Yii / YFii
    if output_type == 1 || output_type == 2
        Yii = cell(6, 6);
        Yii{1, 1} = [0 0 0 0 0 1] * A{1, 1};
        Yii{1, 2} = [0 0 0 0 0 1] * HT{1, 1} * A{2, 1};
        Yii{1, 3} = [0 0 0 0 0 1] * HT{1, 1} * HT{2, 1} * A{3, 1};
        Yii{1, 4} = [0 0 0 0 0 1] * HT{1, 1} * HT{2, 1} * HT{3, 1} * A{4, 1};
        Yii{1, 5} = [0 0 0 0 0 1] * HT{1, 1} * HT{2, 1} * HT{3, 1} * HT{4, 1} * A{5, 1};
        Yii{1, 6} = [0 0 0 0 0 1] * HT{1, 1} * HT{2, 1} * HT{3, 1} * HT{4, 1} * HT{5, 1} * A{6, 1};
        Yii{2, 2} = [0 0 0 0 0 1] * A{2, 1};
        Yii{2, 3} = [0 0 0 0 0 1] * HT{2, 1} * A{3, 1};
        Yii{2, 4} = [0 0 0 0 0 1] * HT{2, 1} * HT{3, 1} * A{4, 1};
        Yii{2, 5} = [0 0 0 0 0 1] * HT{2, 1} * HT{3, 1} * HT{4, 1} * A{5, 1};
        Yii{2, 6} = [0 0 0 0 0 1] * HT{2, 1} * HT{3, 1} * HT{4, 1} * HT{5, 1} * A{6, 1};
        Yii{3, 3} = [0 0 0 0 0 1] * A{3, 1};
        Yii{3, 4} = [0 0 0 0 0 1] * HT{3, 1} * A{4, 1};
        Yii{3, 5} = [0 0 0 0 0 1] * HT{3, 1} * HT{4, 1} * A{5, 1};
        Yii{3, 6} = [0 0 0 0 0 1] * HT{3, 1} * HT{4, 1} * HT{5, 1} * A{6, 1};
        Yii{4, 4} = [0 0 0 0 0 1] * A{4, 1};
        Yii{4, 5} = [0 0 0 0 0 1] * HT{4, 1} * A{5, 1};
        Yii{4, 6} = [0 0 0 0 0 1] * HT{4, 1} * HT{5, 1} * A{6, 1};
        Yii{5, 5} = [0 0 0 0 0 1] * A{5, 1};
        Yii{5, 6} = [0 0 0 0 0 1] * HT{5, 1} * A{6, 1};
        Yii{6, 6} = [0 0 0 0 0 1] * A{6, 1};
    else
        % Y_Fbase
        HT_0 = [R{1}, zeros(3, 3); skew(p{1}) * R{1}, R{1}];
        YFii = cell(1, 6);
        YFii{1, 1} = HT_0 * A{1, 1};
        YFii{1, 2} = HT_0 * HT{1, 1} * A{2, 1};
        YFii{1, 3} = HT_0 * HT{1, 1} * HT{2, 1} * A{3, 1};
        YFii{1, 4} = HT_0 * HT{1, 1} * HT{2, 1} * HT{3, 1} * A{4, 1};
        YFii{1, 5} = HT_0 * HT{1, 1} * HT{2, 1} * HT{3, 1} * HT{4, 1} * A{5, 1};
        YFii{1, 6} = HT_0 * HT{1, 1} * HT{2, 1} * HT{3, 1} * HT{4, 1} * HT{5, 1} * A{6, 1};
    end

    % 组装单点 Y
    if output_type == 1
        Y_singlepoint = [Yii{1, 1}, Yii{1, 2}, Yii{1, 3}, Yii{1, 4}, Yii{1, 5}, Yii{1, 6};
                         zeros(1, 10), Yii{2, 2}, Yii{2, 3}, Yii{2, 4}, Yii{2, 5}, Yii{2, 6};
                         zeros(1, 10), zeros(1, 10), Yii{3, 3}, Yii{3, 4}, Yii{3, 5}, Yii{3, 6};
                         zeros(1, 10), zeros(1, 10), zeros(1, 10), Yii{4, 4}, Yii{4, 5}, Yii{4, 6};
                         zeros(1, 10), zeros(1, 10), zeros(1, 10), zeros(1, 10), Yii{5, 5}, Yii{5, 6};
                         zeros(1, 10), zeros(1, 10), zeros(1, 10), zeros(1, 10), zeros(1, 10), Yii{6, 6}];
    elseif output_type == 2
        Y_singlepoint = [Yii{1, 1}, qdd(1), Yii{1, 2}, 0, Yii{1, 3}, 0, Yii{1, 4}, 0, Yii{1, 5}, 0, Yii{1, 6}, 0;
                         zeros(1, 10), 0, Yii{2, 2}, qdd(2), Yii{2, 3}, 0, Yii{2, 4}, 0, Yii{2, 5}, 0, Yii{2, 6}, 0;
                         zeros(1, 10), 0, zeros(1, 10), 0, Yii{3, 3}, qdd(3), Yii{3, 4}, 0, Yii{3, 5}, 0, Yii{3, 6}, 0;
                         zeros(1, 10), 0, zeros(1, 10), 0, zeros(1, 10), 0, Yii{4, 4}, qdd(4), Yii{4, 5}, 0, Yii{4, 6}, 0;
                         zeros(1, 10), 0, zeros(1, 10), 0, zeros(1, 10), 0, zeros(1, 10), 0, Yii{5, 5}, qdd(5), Yii{5, 6}, 0;
                         zeros(1, 10), 0, zeros(1, 10), 0, zeros(1, 10), 0, zeros(1, 10), 0, zeros(1, 10), 0, Yii{6, 6}, qdd(6)];
    elseif output_type == 3
        Y_singlepoint = [YFii{1, 1}, YFii{1, 2}, YFii{1, 3}, YFii{1, 4}, YFii{1, 5}, YFii{1, 6}];
    end

    Y = [Y; Y_singlepoint];
end

end

%% 子函数：反对称矩阵（cross(x,y) = skew(x)*y）
function S = skew(v)
v = v(:);
S = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
end

%% 子函数：惯量相关 3×3 块（与 skew 配合用于动力学回归）
function S = skewstar(v)
v = v(:);
S = [0, v(3), -v(2); -v(3), 0, v(1); v(2), -v(1), 0];
end
