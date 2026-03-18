function Y = ReMatrix_E1_limb_URDF(limb, qfull, qdfull, qddfull, output_type, para_order)
% ReMatrix_E1_limb_URDF  E1 单腿/单臂基于电流的动力学辨识矩阵（URDF，无底座）
%
% 单腿或单臂、基于关节电流（力矩）的动力学参数辨识矩阵 Y_Tauj，满足
%   tau = Y * theta
% 其中 tau 为关节力矩堆叠向量 [tau_1; ...; tau_n]，Y 为 (N*n)×(10*n 或 11*n) 回归矩阵，
% theta 为按 link1..link_n 依次堆叠的动力学参数向量。
% 不依赖 DH 与 calMatrix_kinHomTrans，由 get_e1_limb_robot(limb) 的 rigidBodyTree 和
% getTransform 得到齐次变换，再按与 ReMatrix_RobotDynPara 相同的 A/HT/Yii 公式计算。
%
% ========== 原理与约定（不靠试，按标准串联链回归） ==========
%
% 1) 回归结构（固定基串联链）
%    牛顿-欧拉逆递推：关节 r 的力矩 τ_r = z_r' * (link r 上的力矩分量)。
%    link r 上的力矩只由 link r 及下游 link r+1..n 的惯性/重力贡献叠加得到，
%    与上游 link 1..r-1 无关。故：
%      τ_r = sum_{c=r}^{n} Y_{r,c} θ_c  （仅 c>=r，对应块上三角结构）
%    上游 link 的 θ_c (c<r) 不进入 τ_r。见 Khalil/Kleinfinger 或《机器人动力学》回归形式。
%
% 2) 变换与平移的约定
%    MATLAB getTransform(robot,config, source, target) 返回 ^targetT_source，即把 source 系表达的点变换到 target 系表达。
%    本实现直接构造相对变换：
%      Trel{1} = ^baseT_1
%      Trel{i} = ^{i-1}T_i,  i>=2
%    于是：
%      R{i} = ^{i-1}R_i
%      p{i} = link i 原点在 i-1 系下的位置
%    递推中使用 R{i}' 将上一级坐标系中的运动量转到当前 link 坐标系。
%
% 3) 重力 vd0
%    基座加速度取为 a_0 = vd0，使重力等效为惯性力：通常 vd0 = -g（g 为重力加速度向量，如 [0;0;-9.81]），
%    表示基座“向上加速 g”，等价于重力向下。若与 inverseDynamics(robot_limb) 的重力项整体反号，
%    再检查 Gravity 参考系与符号约定。
%
% 4) HT 与 Yii
%    HT{i}：wrench 从 link i+1 传到 link i（child→parent）的 6×6 伴随。Yii{r,c}=e_r*(HT{r}*..*HT{c-1})*A{c}（c>r）
%    表示下游 link c 的动力学块 A{c}*θ_c 传到 frame r 后沿关节 r 轴投影。e_r = [0 0 0 z_r']。
%
% ========== 输入：
%   limb      - 'left_leg' | 'right_leg'（单腿 6 自由度）| 'left_arm' | 'right_arm'（单臂 3 自由度）
%   qfull     - N×n 关节位置 (rad)，n=6(腿) 或 3(臂)
%   qdfull    - N×n 关节速度 (rad/s)
%   qddfull   - N×n 关节加速度 (rad/s^2)
%   output_type - 仅基于电流的关节力矩回归，theta 的含义为：
%         1: 60 参数 (10×6)，仅刚体动力学参数：
%               对于第 i 个 link，10 维子向量为（见 para_order）：
%                 para_order = 1: [m, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]
%                 para_order = 2: [Ixx, Ixy, Ixz, Iyy, Iyz, Izz, mx, my, mz, m]
%               整体 theta = [theta_link1; theta_link2; ...; theta_linkn]，长度 10*n。
%         2: 66 参数 (11×6)，在上述每个 link 的 10 维后再加 1 个转子惯量 Ia_i：
%               对于第 i 个 link，11 维子向量为：
%                 [ theta_link_i(1:10), Ia_i ]  （Ia_i 乘以关节加速度 qdd(i) 的系数也在 Y 中）
%               整体 theta = [theta_link1; Ia_1; theta_link2; Ia_2; ...; theta_linkn; Ia_n]，长度 11*n。
%   para_order - 1 或 2，单个 link 内 10 维参数的顺序约定（与 ReMatrix_RobotDynPara 一致）：
%         1: [m, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]
%         2: [Ixx, Ixy, Ixz, Iyy, Iyz, Izz, mx, my, mz, m]
%
% 输出：
%   Y - 观测矩阵 (N*n)×(10*n) 或 (N*n)×(11*n)，tau = Y * theta；腿 n=6，臂 n=3
%
% 依赖：get_e1_limb_robot，Robotics System Toolbox（getTransform）。

if nargin < 5
    output_type = 1;
end
if nargin < 6
    para_order = 1;
end
if ~ismember(output_type, [1, 2])
    error('ReMatrix_E1_limb_URDF: 仅支持基于电流的 output_type 1 或 2。');
end
if ~ismember(para_order, [1, 2])
    error('ReMatrix_E1_limb_URDF: para_order 仅支持 1 或 2。');
end

valid_limbs = {'left_leg', 'right_leg', 'left_arm', 'right_arm'};
if ~ismember(limb, valid_limbs)
    error('ReMatrix_E1_limb_URDF: 仅支持 ''left_leg'' | ''right_leg'' | ''left_arm'' | ''right_arm''，当前 ''%s''。', limb);
end

[robot_limb, n] = get_e1_limb_robot(limb);
if n ~= 6 && n ~= 3
    error('ReMatrix_E1_limb_URDF: 肢体 %s 自由度为 %d，仅支持腿(6)或臂(3)。', limb, n);
end

% 连杆名称顺序：仍沿用 limb subtree 的 Bodies 顺序（用于在整机中按名字索引）
bodyNames = cell(1, n);
for i = 1:n
    bodyNames{i} = robot_limb.Bodies{i}.Name;
end

% 统一用整机 + 整机 Base 计算 ^0T_i，重力必须与 T0 参考系一致（见下）
if isempty(which('get_e1_full_robot'))
    addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), '..', '..', '..', 'robot_model'));
end
[robot_full, ~] = get_e1_full_robot();
[idx_ll, idx_rl, idx_la, idx_ra] = get_e1_full_robot_limb_indices();
switch limb
    case 'left_leg',  limb_idx = idx_ll;
    case 'right_leg', limb_idx = idx_rl;
    case 'left_arm',  limb_idx = idx_la;
    case 'right_arm', limb_idx = idx_ra;
    otherwise, limb_idx = [];
end
if isempty(limb_idx) || numel(limb_idx) ~= n
    error('ReMatrix_E1_limb_URDF: 无法获取肢体 %s 在整机中的索引。', limb);
end
body1_full = getBody(robot_full, bodyNames{1});
% T0 与重力须与 inverseDynamics(robot_limb) 一致：肢体基座 base_link 为参考系
if ~isempty(body1_full.Parent)
    base_name_T0 = body1_full.Parent.Name;  % 通常 'base_link'，与 limb subtree 基座一致
else
    base_name_T0 = robot_full.BaseName;
end
gravity = robot_limb.Gravity(:);
if numel(gravity) ~= 3
    gravity = [0; 0; -9.81];
end
config0_full = homeConfiguration(robot_full);
if ~isnumeric(config0_full)
    error('ReMatrix_E1_limb_URDF: 整机 homeConfiguration 非数值，无法用整机计算 T0。');
end
config0_full = config0_full(:)';
% 各关节轴（URDF JointAxis，在子 link 系下；不写死 z）
z_axis = cell(1, n);
for i = 1:n
    ax = robot_limb.Bodies{i}.Joint.JointAxis;
    if isempty(ax) || norm(ax) < 1e-10
        z_axis{i} = [0; 0; 1];
    else
        z_axis{i} = ax(:) / norm(ax);
    end
end

% 配置：DataFormat='row' 时 homeConfiguration 返回 1×n 数值行向量；否则为 struct/struct 数组
config0 = homeConfiguration(robot_limb);
if isnumeric(config0)
    % DataFormat='row' 或 'column'：config 为关节位置向量，getTransform 直接接受
    config0 = config0(:)';
    if numel(config0) ~= n
        error('ReMatrix_E1_limb_URDF: 肢体 %s 的 home 配置长度 %d 与 n=%d 不一致。', limb, numel(config0), n);
    end
    use_numeric_config = true;
elseif isstruct(config0) && numel(config0) >= n && isfield(config0, 'JointName')
    jointNames = {config0.JointName};
    if numel(jointNames) ~= n
        error('ReMatrix_E1_limb_URDF: 肢体 %s 的关节数 %d 与自由度 n=%d 不一致。', limb, numel(jointNames), n);
    end
    use_numeric_config = false;
    use_struct_array = true;
elseif isstruct(config0)
    jointNames = fieldnames(config0);
    if numel(jointNames) ~= n
        error('ReMatrix_E1_limb_URDF: 肢体 %s 的关节数 %d 与自由度 n=%d 不一致。', limb, numel(jointNames), n);
    end
    use_numeric_config = false;
    use_struct_array = false;
else
    error('ReMatrix_E1_limb_URDF: homeConfiguration 返回类型不可用。');
end

N = size(qfull, 1);
n10 = 10 * n;
n11 = 11 * n;

% 维度一致性检查
if size(qfull, 2) ~= n
    error('ReMatrix_E1_limb_URDF: qfull 列数 = %d, 但 n = %d。', size(qfull, 2), n);
end
if size(qdfull, 2) ~= n || size(qddfull, 2) ~= n
    error('ReMatrix_E1_limb_URDF: qdfull/qddfull 列数必须等于 n = %d。', n);
end
if size(qdfull, 1) ~= N || size(qddfull, 1) ~= N
    error('ReMatrix_E1_limb_URDF: qfull/qdfull/qddfull 行数必须一致，当前 [%d, %d, %d]。', ...
        N, size(qdfull, 1), size(qddfull, 1));
end

if output_type == 1
    Y = zeros(N * n, n10);
else
    Y = zeros(N * n, n11);
end

for iPoint = 1:N
    q = qfull(iPoint, :);
    qd = qdfull(iPoint, :);
    qdd = qddfull(iPoint, :);
    if iscolumn(q), q = q'; end
    if iscolumn(qd), qd = qd'; end
    if iscolumn(qdd), qdd = qdd'; end

    if use_numeric_config
        config = q;   % 1×n 行向量，与 DataFormat='row' 一致
    elseif use_struct_array
        config = config0;
        for j = 1:n
            config(j).JointPosition = q(j);
        end
    else
        config = config0;
        for j = 1:n
            config.(jointNames{j}) = q(j);
        end
    end

    % 相对变换 ^{i-1}T_i（parent -> child），直接用 getTransform 获取，避免自己拼 T0 方向出错
    % getTransform(robot, config, source, target) 返回 ^targetT_source
    % 因此：
    %   Trel{1} = getTransform(..., body1, base)      -> ^baseT_1
    %   Trel{i} = getTransform(..., body_i, body_{i-1}) -> ^{i-1}T_i
    Trel = cell(1, n);
    config_full = config0_full;
    config_full(limb_idx) = q;
    Trel{1} = getTransform(robot_full, config_full, bodyNames{1}, base_name_T0);  % ^base T_1
    for i = 2:n
        Trel{i} = getTransform(robot_full, config_full, bodyNames{i}, bodyNames{i-1}); % ^{i-1}T_i
    end
    R = cell(1, n);
    p = cell(1, n);
    for i = 1:n
        R{i} = Trel{i}(1:3, 1:3);   % ^{i-1}R_i
        p{i} = Trel{i}(1:3, 4);     % ^{i-1}p_i（相邻平移）
    end

    w0 = [0; 0; 0];
    wd0 = [0; 0; 0];
    vd0 = -gravity(:);   % 若与 inverseDynamics 的重力项整体反号，再检查 Gravity 参考系与符号约定

    w = cell(1, n);
    wd = cell(1, n);
    vd = cell(1, n);
    % 递推用「link i 原点在上一级系下」：见文件头原理 2)。若 getTransform 的 p=link 在 base 下则直接用 p{1}；若 p=base 在 link 下则用 -R{1}'*p{1}
    for i = 1:n
        z = z_axis{i};
        if i == 1
            w{i} = R{1}' * w0 + qd(1) * z;
            wd{i} = R{1}' * wd0 + qdd(1) * z + cross(R{1}' * w0, qd(1) * z);
            vd{i} = R{1}' * (vd0 + cross(wd0, p{1}) + cross(w0, cross(w0, p{1})));
        else
            w{i} = R{i}' * w{i-1} + qd(i) * z;
            wd{i} = R{i}' * wd{i-1} + qdd(i) * z + cross(R{i}' * w{i-1}, qd(i) * z);
            vd{i} = R{i}' * (vd{i-1} + cross(w{i-1}, cross(w{i-1}, p{i})) + cross(wd{i-1}, p{i}));
        end
        w{i} = w{i}(:);
        wd{i} = wd{i}(:);
        vd{i} = vd{i}(:);
    end

    % A 矩阵（与 ReMatrix_RobotDynPara 一致）
    A = cell(n, 1);
    if para_order == 1
        for i = 1:n
            % 惯量块需 3×6 算子（I_vec = [Ixx Ixy Ixz Iyy Iyz Izz]'），skewstar6：S*I_vec = I*v
            A{i, 1} = [vd{i}, skew(wd{i}) + skew(w{i})*skew(w{i}), zeros(3, 6);
                       zeros(3, 1), -skew(vd{i}), skewstar6(wd{i}) + skew(w{i})*skewstar6(w{i})];
        end
    else
        for i = 1:n
            A{i, 1} = [zeros(3, 6), skew(wd{i}) + skew(w{i})*skew(w{i}), vd{i}(:);
                       skewstar6(wd{i}) + skew(w{i})*skewstar6(w{i}), -skew(vd{i}), zeros(3, 1)];
        end
    end

    % HT 矩阵：必须用相对 R、p（已为 Rrel、prel）
    HT = cell(n, 1);
    for i = 1:n
        HT{i, 1} = zeros(6, 6);
    end
    for i = 1:n-1
        HT{i, 1}(1:3, 1:3) = R{i+1};
        HT{i, 1}(4:6, 1:3) = skew(p{i+1}) * R{i+1};
        HT{i, 1}(4:6, 4:6) = R{i+1};
    end

    % Yii 仅保留 c>=r（块上三角）：标准串联链 τ_r = sum_{c=r}^{n} Y_{r,c} θ_c，上游 link c<r 不进入 τ_r
    e_r = cell(1, n);
    for r = 1:n
        e_r{r} = [zeros(1, 3), z_axis{r}'];
    end
    Yii = cell(n, n);
    for r = 1:n
        for c = r:n
            if c == r
                Yii{r, c} = e_r{r} * A{r, 1};
            else
                prod_HT = HT{r, 1};
                for k = r+1:c-1
                    prod_HT = prod_HT * HT{k, 1};
                end
                Yii{r, c} = e_r{r} * prod_HT * A{c, 1};
            end
        end
    end

    % ===== 第一节本地退化排查（仅 iPoint==1 打印一次）=====
    if iPoint == 1
        persistent first_link_debug_done;
        if isempty(first_link_debug_done)
            first_link_debug_done = true;
            fprintf('\n===== Debug first link geometry =====\n');
            fprintf('base_name_T0 = %s\n', base_name_T0);
            fprintf('body1 = %s\n', bodyNames{1});
            fprintf('Trel{1} =\n'); disp(Trel{1});
            fprintf('p{1} = [%.6e %.6e %.6e]\n', p{1}(1), p{1}(2), p{1}(3));
            fprintf('R{1} =\n'); disp(R{1});
            fprintf('z_axis{1} = [%.6e %.6e %.6e]\n', z_axis{1}(1), z_axis{1}(2), z_axis{1}(3));

            fprintf('\n===== Debug first link kinematics =====\n');
            fprintf('w{1}  = [%.6e %.6e %.6e]\n', w{1}(1), w{1}(2), w{1}(3));
            fprintf('wd{1} = [%.6e %.6e %.6e]\n', wd{1}(1), wd{1}(2), wd{1}(3));
            fprintf('vd{1} = [%.6e %.6e %.6e]\n', vd{1}(1), vd{1}(2), vd{1}(3));

            fprintf('\n===== First-link local: A{1} and Yii{1,1} =====\n');
            disp('A{1} = '); disp(A{1});
            disp('Yii{1,1} = '); disp(Yii{1,1});

            ax = robot_limb.Bodies{1}.Joint.JointAxis(:);
            if norm(ax) < 1e-12, ax = [0;0;1]; end
            ax = ax / norm(ax);
            z_try1 = R{1}' * ax; z_try1 = z_try1 / norm(z_try1);
            z_try2 = R{1}  * ax; z_try2 = z_try2 / norm(z_try2);
            Yii11_try1 = [zeros(1,3), z_try1'] * A{1};
            Yii11_try2 = [zeros(1,3), z_try2'] * A{1};
            disp('Yii11_try1 = [0 0 0 R''*z_joint]*A{1} :'); disp(Yii11_try1);
            disp('Yii11_try2 = [0 0 0 R*z_joint]*A{1} :');  disp(Yii11_try2);
        end
    end

    % 单点观测矩阵
    n10 = 10 * n;
    n11 = 11 * n;
    if output_type == 1
        Y_singlepoint = zeros(n, n10);
        for r = 1:n
            for c = r:n
                Y_singlepoint(r, (c-1)*10 + (1:10)) = Yii{r, c};
            end
        end
    else
        % output_type == 2: 66 参数，含 Ia
        Y_singlepoint = zeros(n, n11);
        for r = 1:n
            col = 0;
            for c = 1:n
                if c < r
                    col = col + 11;
                else
                    Y_singlepoint(r, col + (1:10)) = Yii{r, c};
                    Y_singlepoint(r, col + 11) = (c == r) * qdd(c);
                    col = col + 11;
                end
            end
        end
    end

    rows = (iPoint - 1) * n + (1 : n);
    Y(rows, :) = Y_singlepoint;
end

end

function S = skew(v)
v = v(:);
S = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
end

%% 3×6 惯量算子：S*I_vec = I*v，I_vec = [Ixx Ixy Ixz Iyy Iyz Izz]'，用于 A 矩阵 10 列一致
function S = skewstar6(v)
v = v(:);
S = [v(1), v(2), v(3), 0,    0,    0;
     0,    v(1), 0,    v(2), v(3), 0;
     0,    0,    v(1), 0,    v(2), v(3)];
end
