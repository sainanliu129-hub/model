function [trajectory, trajPara, refPos, refVel, refAcc, t_period, A_pos, A_vel, A_acc, valid_ok, violation_details] = generate_excitation_trajectory(config, varargin)
% generate_excitation_trajectory  傅里叶级数激励轨迹生成（BODY_JNT_CURRENT 辨识用）
%
% 激励轨迹生成相关代码均在本文件夹中（本文件含主函数与子函数；fourier_trajectory_design_matrix 为封装）。
%
% 根据文档「基于关节电流的本体动力学辨识——激励轨迹生成方案」实现：
% 傅里叶级数 q_j(t) = q_{j,0} + sum_k( a_{j,k}*sin(k*omega*t) + b_{j,k}*cos(k*omega*t) )，
% 系数 coeff 可由优化得到（最小化观测矩阵条件数），或直接给定/随机初值。
%
% 用法:
%   [trajectory, trajPara, refPos, refVel, refAcc, t_period] = generate_excitation_trajectory(config)
%   [...] = generate_excitation_trajectory(config, 'optimize', true)
%   [...] = generate_excitation_trajectory(config, 'coeff', coeff0)  % 仅用给定系数求轨迹，不优化
%   [..., t_period, A_pos, A_vel, A_acc] = generate_excitation_trajectory(config, 'output_design_matrix', true)
%
% 输入:
%   config - 结构体，字段见下方「配置结构 config」
%
% 可选名值对:
%   'optimize'   - true/false，是否优化系数以最小化条件数（默认 true；false 时仅用初值或随机系数）
%   'coeff'      - 若提供则直接用于生成轨迹（忽略 optimize）；否则用 config 中的初值或随机
%   'cond_fun'   - 函数句柄 @(q,qd,qdd) cond_number，用于优化目标；不提供则用激励量代理
%   'max_retries'- 优化失败时重试次数（默认 5）
%   'validate'   - 生成后是否做约束校验（默认 true）
%   'output_design_matrix' - true 时多输出 A_pos, A_vel, A_acc（辨识用设计矩阵）
%   'display_optim' - true 时优化时打印 fmincon 迭代日志（默认 false）
%
% 输出:
%   trajectory - 完整关节位置序列：每行一个时刻 [t, q1, q2, ... ]，含过渡段+周期重复
%   trajPara   - 傅里叶系数向量 (dim*(2*order+1)×1)，辨识时需与采集一致
%   refPos     - 单周期参考位置 (CN×dim)，CN = period*sample_frequency
%   refVel     - 单周期参考速度 (CN×dim)
%   refAcc     - 单周期参考加速度 (CN×dim)
%   t_period   - 单周期时间列向量 (CN×1)
%   A_pos, A_vel, A_acc -（可选）设计矩阵 (CN*dim)×COL，仅当 output_design_matrix=true 时有效
%   valid_ok            -（可选）校验是否通过，仅当 validate=true 时有效；nargout>=10 时返回
%   violation_details   -（可选）校验未通过时的超限明细（结构体数组），nargout>=11 时返回
%
% 配置结构 config（与文档 ExciteTrajectoryConfig 对应）:
%   .type                  - 辨识类型，本体电流为 10（可忽略，仅兼容）
%   .move_axis              - 参与运动的关节下标 [0,1,...,dim-1]，本体全轴则 [0,1,2,3,4,5]
%   .init_joint             - 各关节初始位置 (rad)，轨迹起点/终点，dim×1 或 1×dim
%   .upper_joint_bound      - 关节位置上限 (rad)，dim×1
%   .lower_joint_bound      - 关节位置下限 (rad)，dim×1
%   .upper_cartesian_bound  - 末端笛卡尔上限 [x,y,z]（可选，校验用）
%   .lower_cartesian_bound  - 末端笛卡尔下限（可选）
%   .max_velocity           - 关节速度上限 (rad/s)，dim×1
%   .max_acceleration       - 关节加速度上限 (rad/s^2)，dim×1
%   .traj_cycle             - 周期重复次数（采集时重复）
%   .sample_frequency       - 采样频率 (Hz)，如 200
%   .order                  - 傅里叶阶数（默认 5）
%   .period                 - 周期 T (s)（默认 10）
%   .sample_number          - 约束检查用每周期离散点数（默认 15）
%   .coeff_lower / .coeff_upper - 傅里叶系数 a_jk、b_jk 的优化上下界（常数项 c_j 固定为 init_joint）；默认 [-0.5, 0.5]
%   .amplitude_alpha        - 幅值约束 A_max=alpha*(upper-lower)/2（默认 0.7）
%   .enable_collision_check - true 时优化与校验才做自碰撞检测（默认 true，由 run_excitation_trajectory_standalone 设置）
%   .traj_parameter         - 若已有优化好的系数可填入，否则将生成或优化
%
% 依赖: 无（纯数学）；若 optimize=true 需 Optimization Toolbox（fmincon）；若未提供 cond_fun，使用激励量代理目标。
%
% 参见: fourier_trajectory_design_matrix（同文件夹，封装调用本函数并返回设计矩阵）

p = inputParser;
addParameter(p, 'optimize', true, @(x) islogical(x) || (isnumeric(x) && isscalar(x)));
addParameter(p, 'coeff', [], @(x) isempty(x) || isvector(x));
addParameter(p, 'cond_fun', [], @(x) isempty(x) || isa(x, 'function_handle'));
addParameter(p, 'max_retries', 5, @isnumeric);
addParameter(p, 'validate', true, @(x) islogical(x) || (isnumeric(x) && isscalar(x)));
addParameter(p, 'output_design_matrix', false, @(x) islogical(x) || (isnumeric(x) && isscalar(x)));
addParameter(p, 'display_optim', false, @(x) islogical(x) || (isnumeric(x) && isscalar(x)));
addParameter(p, 'max_iter_optim', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x)));
addParameter(p, 'max_feval_optim', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x)));
parse(p, varargin{:});
opts = p.Results;
opts.optimize = logical(opts.optimize);
opts.validate = logical(opts.validate);
opts.output_design_matrix = logical(opts.output_design_matrix);
opts.display_optim = logical(opts.display_optim);

%% 解析 config，补默认
dim = numel(config.move_axis);
if isfield(config, 'order') && ~isempty(config.order)
    order = config.order;
else
    order = 5;
end
if isfield(config, 'period') && ~isempty(config.period)
    period = config.period;
else
    period = 10;
end
if isfield(config, 'sample_number') && ~isempty(config.sample_number)
    sample_number = config.sample_number;
else
    sample_number = 15;
end
if isfield(config, 'sample_frequency') && ~isempty(config.sample_frequency)
    sample_frequency = config.sample_frequency;
else
    sample_frequency = 200;
end
traj_cycle = 1;
if isfield(config, 'traj_cycle') && ~isempty(config.traj_cycle)
    traj_cycle = config.traj_cycle;
end

init_joint = config.init_joint(:);
if length(init_joint) ~= dim
    error('init_joint 长度须与 move_axis 一致 (dim=%d)。', dim);
end
upper_joint = config.upper_joint_bound(:);
lower_joint = config.lower_joint_bound(:);
max_vel = config.max_velocity(:);
max_acc = config.max_acceleration(:);
if numel(upper_joint) ~= dim || numel(lower_joint) ~= dim
    error('关节限位长度须为 dim=%d。', dim);
end
if numel(max_vel) ~= dim || numel(max_acc) ~= dim
    error('max_velocity / max_acceleration 长度须为 dim=%d。', dim);
end

% 生成轨迹用范围：默认设计=完整范围的 0.4、速度=0.9*上限；若 use_full_bounds_for_optim=true 则用 config 原始限位。校核位置=完整范围的 0.45（见 validate_trajectory）。
config.upper_joint_bound_orig = upper_joint;
config.lower_joint_bound_orig = lower_joint;
center = (upper_joint + lower_joint) / 2;
full_range = upper_joint - lower_joint;   % 完整范围
use_full = isfield(config, 'use_full_bounds_for_optim') && config.use_full_bounds_for_optim;
if ~use_full
    upper_joint = center + 0.4 * full_range;   % 设计半宽 = 0.4×完整范围
    lower_joint = center - 0.4 * full_range;
    max_vel = 0.9 * max_vel;
end
% 写回 config，供约束与校验使用
config.upper_joint_bound = upper_joint;
config.lower_joint_bound = lower_joint;
config.max_velocity = max_vel;
config.max_acceleration = max_acc;
config.init_joint = center;   % 轨迹常值取中点
init_joint = center;

% 系数边界含义：对傅里叶系数 a_jk、b_jk（sin/cos 幅值）的优化上下界；常数项 c_j = init_joint 固定。
% 默认 [coeff_lower, coeff_upper] = [-0.5, 0.5]，使轨迹幅值适中、易满足位置/速度/加速度约束。
coeff_lower = -0.5;
coeff_upper = 0.5;
if isfield(config, 'coeff_lower') && ~isempty(config.coeff_lower)
    coeff_lower = config.coeff_lower;
end
if isfield(config, 'coeff_upper') && ~isempty(config.coeff_upper)
    coeff_upper = config.coeff_upper;
end

% 与 C++ 一致：系数个数 = dim×(2×order+1)
coeff_col = dim * (2 * order + 1);
COL = coeff_col;
CN = round(period * sample_frequency);  % 每周期输出点数

fprintf('开始生成（dim=%d, order=%d, period=%.1fs, CN=%d）\n', dim, order, period, CN);

%% 确定系数：用户给定 / config 已有 / 随机
final_cond = NaN;
if ~isempty(opts.coeff)
    coeff = opts.coeff(:);
    if numel(coeff) ~= COL
        error('提供的 coeff 长度应为 %d，当前为 %d。', COL, numel(coeff));
    end
    trajPara = coeff;
elseif isfield(config, 'traj_parameter') && ~isempty(config.traj_parameter)
    trajPara = config.traj_parameter(:);
    if numel(trajPara) ~= COL
        error('config.traj_parameter 长度应为 %d。', COL, numel(trajPara));
    end
    coeff = trajPara;
elseif opts.optimize
    % 优化：常数项固定为 init_joint，初值小范围 [-0.1,0.1]，cos 和=0 使 q(0)=init_joint。
    % 层级关系：本循环为【轮】retry（多轮不同初值）；每轮内调用 optimize_fourier_coeff，其内 fmincon 的步数为【iter】。
    % “优化结束”两层含义（与 C++ 一致）：
    %  1) 优化器何时停止：fmincon 的停止条件（见 optimize_fourier_coeff：stopval/ftol/xtol 等），满足任一即返回；exitflag>0 为正常结束。
    %  2) 本模块何时认为“这次优化算成功”：exitflag>0 且 条件数 < CALIB_TRAJECTORY_MAX_COND_NUM(1000)。不满足则重试；即使成功仍须通过后续 validate_trajectory 才用于输出。
    CALIB_TRAJECTORY_MAX_COND_NUM = 1000;
    best_cond = inf;
    best_coeff = [];
    init_range = 0.1;   % 初值在 [-0.1, 0.1]，避免初值过大导致可行域难找
    for retry = 1:opts.max_retries
        coeff0 = init_range * (2*rand(COL, 1) - 1);
        coeff0(1:dim) = init_joint;  % q_0 = 区间中点，固定
        for j = 1:dim
            b_start = dim + dim*order + (j-1)*order + 1;
            b_end   = dim + dim*order + j*order;
            b_j = coeff0(b_start:b_end);
            b_j(order) = -sum(b_j(1:order-1));   % q_j(0)=c_j+sum(b_j)=init_joint
            coeff0(b_start:b_end) = b_j;
        end
        fprintf('  第 %d 轮优化中...\n', retry);
        [coeff_r, cond_r, exitflag] = optimize_fourier_coeff(coeff0, dim, order, period, sample_frequency, ...
            sample_number, init_joint, lower_joint, upper_joint, max_vel, max_acc, ...
            coeff_lower, coeff_upper, config, opts.cond_fun, opts.display_optim, ...
            opts.max_iter_optim, opts.max_feval_optim);
        % 有 cond_fun 时 cond_r 为条件数（正）；无 cond_fun 时为目标值 = -激励量（负，越负激励越强）
        if ~isempty(opts.cond_fun)
            fprintf('  第 %d 轮优化结束，条件数 = %.4f，exitflag = %d\n', retry, cond_r, exitflag);
        else
            fprintf('  第 %d 轮优化结束，目标值 = %.4f（激励量代理），exitflag = %d\n', retry, cond_r, exitflag);
        end
        % 仅当“优化器正常结束”且“条件数合格”时接受该轮系数
        if ~isempty(coeff_r) && exitflag > 0 && cond_r < best_cond && cond_r < CALIB_TRAJECTORY_MAX_COND_NUM
            best_cond = cond_r;
            best_coeff = coeff_r;
        end
    end
    if isempty(best_coeff)
        warning('优化未得到满足条件数的系数（需 exitflag>0 且 cond<%d），使用最后一轮系数。', CALIB_TRAJECTORY_MAX_COND_NUM);
        coeff = coeff_r;
        final_cond = cond_r;
    else
        coeff = best_coeff;
        final_cond = best_cond;
    end
    trajPara = coeff;
else
    % 随机系数，常数项取 init_joint；约束每关节 cos 系数之和为 0，使 q(0)=init_joint，首点不超限
    trajPara = coeff_lower + (coeff_upper - coeff_lower) * rand(COL, 1);
    trajPara(1:dim) = init_joint;
    for j = 1:dim
        b_start = dim + dim*order + (j-1)*order + 1;
        b_end   = dim + dim*order + j*order;
        b_j = trajPara(b_start:b_end);
        b_j(order) = -sum(b_j(1:order-1));   % 使 sum(b_j)=0 => q_j(0)=c_j=init_joint(j)
        trajPara(b_start:b_end) = b_j;
    end
    coeff = trajPara;
end

%% 由系数生成单周期 refPos, refVel, refAcc
t_period = linspace(0, period, CN)';
if t_period(end) > period - 1e-9
    t_period(end) = period;
end
[refPos, refVel, refAcc] = evaluate_fourier_trajectory(coeff, t_period, dim, order, period, init_joint);

if ~isempty(opts.cond_fun)
    final_cond = opts.cond_fun(refPos, refVel, refAcc);
end
if ~isnan(final_cond)
    if ~isempty(opts.cond_fun)
        fprintf('生成轨迹条件数 = %.4e\n', final_cond);
        if final_cond > 1e5
            fprintf('  （60 全参+激励有限时 1e6~1e8 较常见；可运行 diagnose_regressor_condition 看 σ_min/独立列数）\n');
        end
    else
        fprintf('生成轨迹目标值 = %.4f（激励量代理，越负激励越强）\n', final_cond);
    end
end

%% 约束校验（生成后复检，不放宽；任一点超限则轨迹不合格）
% 有效范围已在上面按“完整范围的 0.4、速度 0.9 倍、加速度由扭矩/模型”设定，此处严格按该范围校验。
valid_ok = true;
violation_details = [];
if opts.validate
    % 校核位置=完整范围的 0.45（名义限位在 config.upper_joint_bound_orig）；速度/加速度不放宽
    [ok, msg, violation_details] = validate_trajectory(refPos, refVel, refAcc, config, 1.0, 1.0, 1.0);
    valid_ok = ok;
    if ~ok
        if opts.optimize
            warning('轨迹未通过校验: %s（已开启优化；可缩小系数范围、缩周期/降阶，或校核时放宽扭矩）', msg);
        else
            warning('轨迹未通过校验: %s（可用 optimize=true 或缩小系数范围）', msg);
        end
    end
end

%% 拼接完整 trajectory：过渡段 + 周期重复 + 过渡段回 init
% 简化：过渡段用线性插值从 init_joint 到周期起点/终点，时长 1s
t_trans = 1.0;
n_trans = max(1, round(t_trans * sample_frequency));
q_start = refPos(1, :)';
q_end   = refPos(end, :)';

t_ramp_up   = linspace(0, t_trans, n_trans)';
t_ramp_down = linspace(0, t_trans, n_trans)';
% 插值：每行一个时刻，n_trans×dim
q_ramp_up   = (init_joint + (q_start - init_joint) .* (t_ramp_up' / t_trans))';
q_ramp_down = (q_end + (init_joint - q_end) .* (t_ramp_down' / t_trans))';

t_mid = (0:(traj_cycle * CN - 1))' / sample_frequency;
refPos_rep = repmat(refPos, traj_cycle, 1);
refVel_rep = repmat(refVel, traj_cycle, 1);
refAcc_rep = repmat(refAcc, traj_cycle, 1);
for c = 1:traj_cycle-1
    t_mid((c-1)*CN+1:c*CN) = t_mid((c-1)*CN+1:c*CN) + (c-1)*period;
end
t_mid(traj_cycle*CN) = traj_cycle * period;

t_full = [t_ramp_up; t_trans + t_mid; t_trans + traj_cycle*period + t_ramp_down(2:end)];
q_full = [q_ramp_up; refPos_rep; q_ramp_down(2:end,:)];
trajectory = [t_full, q_full];

%% 可选：输出设计矩阵（辨识用）
if nargout >= 9 && opts.output_design_matrix
    [A_pos, A_vel, A_acc] = fourier_design_matrices(t_period, dim, order, period);
else
    A_pos = []; A_vel = []; A_acc = [];
end
% valid_ok、violation_details 已在约束校验处赋值

end

%% ========== 傅里叶轨迹求值（与文档 8.2–8.4 一致）==========
function [pos, vel, acc] = evaluate_fourier_trajectory(coeff, t, dim, order, period, init_joint)
% 输入: coeff (coeff_col×1), t (N×1), dim, order, period, init_joint (dim×1)；coeff_col = dim×(2×order+1) 与 C++ 一致
% 输出: pos, vel, acc 均为 N×dim
COL = dim * (2 * order + 1);   % dim×(2×order+1)
if numel(coeff) ~= COL
    error('evaluate_fourier_trajectory: coeff 长度应为 %d。', COL);
end
omega = 2*pi / period;
N = numel(t);
pos = zeros(N, dim);
vel = zeros(N, dim);
acc = zeros(N, dim);
for j = 1:dim
    c_j = coeff(j);
    a_start = dim + (j-1)*order + 1;
    a_end   = dim + j*order;
    b_start = dim + dim*order + (j-1)*order + 1;
    b_end   = dim + dim*order + j*order;
    a_j = coeff(a_start:a_end);
    b_j = coeff(b_start:b_end);
    for i = 1:N
        ti = t(i);
        q_j = c_j;
        qd_j = 0;
        qdd_j = 0;
        for k = 1:order
            kwt = k * omega * ti;
            sk = sin(kwt);
            ck = cos(kwt);
            q_j   = q_j   + a_j(k)*sk + b_j(k)*ck;
            qd_j  = qd_j  + k*omega * (a_j(k)*ck - b_j(k)*sk);
            qdd_j = qdd_j - (k*omega)^2 * (a_j(k)*sk + b_j(k)*ck);
        end
        pos(i,j) = q_j;
        vel(i,j) = qd_j;
        acc(i,j) = qdd_j;
    end
end
end

%% ========== 设计矩阵（单时刻 → 用于观测矩阵/条件数）==========
function [A_pos, A_vel, A_acc] = fourier_design_matrices(t, dim, order, period)
omega = 2*pi/period;
COL = dim * (2*order+1);   % 与 C++ 一致: dim×(2×order+1)
N = numel(t);
A_pos = zeros(N*dim, COL);
A_vel = zeros(N*dim, COL);
A_acc = zeros(N*dim, COL);
for i = 1:N
    ti = t(i);
    for j = 1:dim
        A_pos((i-1)*dim+j, j) = 1;
        for k = 1:order
            kwt = k*omega*ti;
            sk = sin(kwt); ck = cos(kwt);
            dsk = k*omega*ck; dck = -k*omega*sk;
            ddsk = -(k*omega)^2*sk; ddck = -(k*omega)^2*ck;
            col_sin = dim + (j-1)*order + k;
            col_cos = dim + dim*order + (j-1)*order + k;
            A_pos((i-1)*dim+j, col_sin) = sk;
            A_pos((i-1)*dim+j, col_cos) = ck;
            A_vel((i-1)*dim+j, col_sin) = dsk;
            A_vel((i-1)*dim+j, col_cos) = dck;
            A_acc((i-1)*dim+j, col_sin) = ddsk;
            A_acc((i-1)*dim+j, col_cos) = ddck;
        end
    end
end
end

%% ========== 约束校验（有一处超限即不合格并立即返回）==========
% 即使“优化成功”（exitflag>0 且 cond<1000）也须通过本校验才用于轨迹输出。
function [ok, msg, details] = validate_trajectory(refPos, refVel, refAcc, config, relax_pos, relax_vel, relax_acc)
% 位置校核：若有 upper_joint_bound_orig 则用 完整范围的 0.45（以中间点对称）；否则用 upper/lower*relax_pos。点数>400 时仅校验 400 点。
if nargin < 5, relax_pos = 1.0; end
if nargin < 6, relax_vel = 1.0; end
if nargin < 7, relax_acc = 1.0; end
tol_pos = 1e-4;   % rad
dim = size(refPos, 2);
upper_joint = config.upper_joint_bound(:);
lower_joint = config.lower_joint_bound(:);
if isfield(config, 'upper_joint_bound_orig') && isfield(config, 'lower_joint_bound_orig')
    u_orig = config.upper_joint_bound_orig(:);
    l_orig = config.lower_joint_bound_orig(:);
    center_pos = (u_orig + l_orig) / 2;
    full_range_orig = u_orig - l_orig;   % 完整范围
    pos_valid_upper = center_pos + 0.45 * full_range_orig;   % 校核半宽 = 0.45×完整范围
    pos_valid_lower = center_pos - 0.45 * full_range_orig;
else
    pos_valid_upper = upper_joint * relax_pos;
    pos_valid_lower = lower_joint * relax_pos;
end
max_vel = config.max_velocity(:);
max_acc = config.max_acceleration(:);
details = struct('sample_idx', {}, 'joint_idx', {}, 'type', {}, 'value', {}, 'limit', {}, 'excess', {});
N = size(refPos, 1);
idx_validate = round(linspace(1, N, min(400, N)));   % 最多 400 点以加速
for ii = 1:numel(idx_validate)
    i = idx_validate(ii);
    q = refPos(i,:)';
    qd = refVel(i,:)';
    qdd = refAcc(i,:)';
    % 位置超上限/下限（超量 < tol_pos 忽略）
    for j = 1:dim
        if q(j) > pos_valid_upper(j) + tol_pos
            details(1).sample_idx = i; details(1).joint_idx = j; details(1).type = 'pos_upper';
            details(1).value = q(j); details(1).limit = pos_valid_upper(j); details(1).excess = q(j) - pos_valid_upper(j);
            ok = false; msg = sprintf('时刻 %d 关节 %d 位置上超', i, j); return;
        end
        if q(j) < pos_valid_lower(j) - tol_pos
            details(1).sample_idx = i; details(1).joint_idx = j; details(1).type = 'pos_lower';
            details(1).value = q(j); details(1).limit = pos_valid_lower(j); details(1).excess = pos_valid_lower(j) - q(j);
            ok = false; msg = sprintf('时刻 %d 关节 %d 位置下超', i, j); return;
        end
    end
    % 速度超限
    for j = 1:dim
        if abs(qd(j)) > max_vel(j) * relax_vel
            details(1).sample_idx = i; details(1).joint_idx = j; details(1).type = 'vel';
            details(1).value = qd(j); details(1).limit = max_vel(j)*relax_vel; details(1).excess = abs(qd(j))-max_vel(j)*relax_vel;
            ok = false; msg = sprintf('时刻 %d 关节 %d 速度超限', i, j); return;
        end
    end
    % 加速度超限
    for j = 1:dim
        if abs(qdd(j)) > max_acc(j) * relax_acc
            details(1).sample_idx = i; details(1).joint_idx = j; details(1).type = 'acc';
            details(1).value = qdd(j); details(1).limit = max_acc(j)*relax_acc; details(1).excess = abs(qdd(j))-max_acc(j)*relax_acc;
            ok = false; msg = sprintf('时刻 %d 关节 %d 加速度超限', i, j); return;
        end
    end
    % 自碰撞（文档 4.5）：仅当 config.enable_collision_check=true 且提供 robot_limb 时检测
    if isfield(config, 'enable_collision_check') && config.enable_collision_check && ...
            isfield(config, 'robot_limb') && ~isempty(config.robot_limb)
        try
            cfg = refPos(i,:);
            if size(cfg, 2) == 1, cfg = cfg'; end
            isColl = checkCollision(config.robot_limb, cfg);
            if isColl
                details(1).sample_idx = i; details(1).joint_idx = 0; details(1).type = 'collision';
                details(1).value = NaN; details(1).limit = NaN; details(1).excess = 1;
                ok = false; msg = sprintf('时刻 %d 自碰撞', i); return;
            end
        catch
        end
    end
end

% 扭矩校核：限值为峰值扭矩的 90%（留 10% 余量），|τ| ≤ 0.9*max_effort（同 idx_validate 以加速）
TORQUE_VALIDATE_RATIO = 0.9;
if isfield(config, 'robot_limb') && ~isempty(config.robot_limb) && isfield(config, 'max_effort') && ~isempty(config.max_effort)
    max_effort = config.max_effort(:);
    if numel(max_effort) >= dim
        max_effort = max_effort(1:dim);
        effort_limit = TORQUE_VALIDATE_RATIO * max_effort;
        for ii = 1:numel(idx_validate)
            i = idx_validate(ii);
            q = refPos(i,:); qd = refVel(i,:); qdd = refAcc(i,:);
            if iscolumn(q), q = q'; end
            if iscolumn(qd), qd = qd'; end
            if iscolumn(qdd), qdd = qdd'; end
            try
                tau = inverseDynamics(config.robot_limb, q, qd, qdd);
                tau = tau(:);
                for j = 1:dim
                    if abs(tau(j)) > effort_limit(j)
                        details(1).sample_idx = i; details(1).joint_idx = j; details(1).type = 'torque';
                        details(1).value = tau(j); details(1).limit = effort_limit(j); details(1).excess = abs(tau(j)) - effort_limit(j);
                        ok = false; msg = sprintf('时刻 %d 关节 %d 扭矩超限（校核限值=峰值%.0f%%）', i, j, TORQUE_VALIDATE_RATIO*100); return;
                    end
                end
            catch
            end
        end
    end
end
% 柱体避障校核：柱体中心 x=-0.5, y=0，截面 0.2×0.2（x∈[-0.6,-0.4], y∈[-0.1,0.1]），z 不限
% 末端连杆坐标系原点不得落入柱体截面内（同 idx_validate 以加速）
if isfield(config, 'robot_limb') && ~isempty(config.robot_limb)
    pillar  = get_pillar_config(config);   % [cx, cy, half_dx, half_dy]
    tol_col = 1e-3;   % 留 1mm 余量
    try
        robot    = config.robot_limb;
        baseName = robot.BaseName;
        n_bodies = robot.NumBodies;
        body_names = cell(1, n_bodies);
        for bi = 1:n_bodies, body_names{bi} = robot.Bodies{bi}.Name; end
        for ii = 1:numel(idx_validate)
            i = idx_validate(ii);
            cfg = refPos(i,:);
            if iscolumn(cfg), cfg = cfg'; end
            for bi = 1:n_bodies
                T    = getTransform(robot, cfg, body_names{bi}, baseName);
                px   = T(1,4); py = T(2,4);
                in_x = abs(px - pillar(1)) < pillar(3) - tol_col;
                in_y = abs(py - pillar(2)) < pillar(4) - tol_col;
                if in_x && in_y
                    details(1).sample_idx = i; details(1).joint_idx = bi; details(1).type = 'pillar';
                    details(1).value = [px, py]; details(1).limit = pillar; details(1).excess = 0;
                    ok = false;
                    msg = sprintf('时刻 %d 连杆 %d(%s) 进入柱体 (x=%.3f, y=%.3f)', i, bi, body_names{bi}, px, py);
                    return;
                end
            end
        end
    catch
    end
end
ok = true;
msg = '';
end

%% ========== 优化系数：最小化 κ(W) 或最大化激励量 ==========
% 本函数对应【轮】内的一次完整优化；内部 fmincon 的每一步为【iter】（迭代步）。
% C++ 侧使用 NLopt（LN_AUGLAG_EQ）；MATLAB 使用 fmincon，停止条件与 C++ 对应：
%   stopval=113：目标 ≤ 113 时立即停止（OutputFcn）；ftol_abs=1e-1；xtol_abs=1e-5；xtol_rel≈StepTolerance。
% 返回 exitflag：>0 表示正常结束（1=一阶最优，2=步长小，3=目标变化小等），≤0 表示未收敛或异常。
function [coeff_out, cond_val, exitflag] = optimize_fourier_coeff(coeff0, dim, order, period, sample_frequency, ...
    sample_number, init_joint, lower_joint, upper_joint, max_vel, max_acc, ...
    coeff_lower, coeff_upper, config, cond_fun, display_optim, max_iter_optim, max_feval_optim)
if nargin < 15, display_optim = false; end
if nargin < 16, max_iter_optim = []; end
if nargin < 17, max_feval_optim = []; end

% 与 C++ 一致的停止/容差常数（仅在有 cond_fun 时 stopval 生效）
STOPVAL = 113;           % 目标（条件数）≤ 113 时立即停止
FTOL_ABS = 1e-1;         % 目标绝对变化 < 0.1 时停止
XTOL_ABS = 1e-5;        % 系数向量绝对变化 < 1e-5 时停止

% 优化时用较低采样（12 Hz）以加速目标与约束评估；最终轨迹仍用 config.sample_frequency
OPTIM_SAMPLE_FREQ = 12;

% 与 C++ 一致：dim×(2×order+1)
COL = dim * (2 * order + 1);
% 约束点：优化阶段 12 Hz，减少单次迭代耗时
n_constraint = max(2, round(period * OPTIM_SAMPLE_FREQ));
t_constraint = linspace(0, period, n_constraint)';
if t_constraint(end) > period, t_constraint(end) = period; end

objective = @(c) eval_objective(c, dim, order, period, sample_frequency, init_joint, cond_fun, OPTIM_SAMPLE_FREQ);
nonlcon = @(c) eval_constraints(c, t_constraint, dim, order, period, init_joint, ...
    lower_joint, upper_joint, max_vel, max_acc, config);

lb = coeff_lower * ones(COL, 1);
ub = coeff_upper * ones(COL, 1);
lb(1:dim) = init_joint;
ub(1:dim) = init_joint;
% 限制迭代/评估次数以缩短单轮时间；需更优解可传 max_iter_optim/max_feval_optim 放大
MAX_ITER = 60;
MAX_FEVAL = 1500;
if ~isempty(max_iter_optim) && max_iter_optim > 0, MAX_ITER = max_iter_optim; end
if ~isempty(max_feval_optim) && max_feval_optim > 0, MAX_FEVAL = max_feval_optim; end

if display_optim
    options = optimoptions('fmincon', 'Display', 'iter', 'MaxIterations', MAX_ITER, ...
        'MaxFunctionEvaluations', MAX_FEVAL, 'ConstraintTolerance', 1e-5, ...
        'StepTolerance', XTOL_ABS, 'FunctionTolerance', FTOL_ABS);
else
    options = optimoptions('fmincon', 'Display', 'off', 'MaxIterations', MAX_ITER, ...
        'MaxFunctionEvaluations', MAX_FEVAL, 'ConstraintTolerance', 1e-5, ...
        'StepTolerance', XTOL_ABS, 'FunctionTolerance', FTOL_ABS);
end
% 目标 ≤ STOPVAL 时立即停止（对应 C++ stopval）
if ~isempty(cond_fun)
    options.OutputFcn = @(x, optimValues, state) (strcmp(state, 'iter') && optimValues.fval <= STOPVAL);
end

exitflag = 0;
try
    [coeff_out, fval, exitflag] = fmincon(objective, coeff0, [], [], [], [], lb, ub, nonlcon, options);
    cond_val = fval;
    if exitflag == -1
        exitflag = 5;   % 5 = 达到 stopval，视为正常结束（与 C++ result=5 对应）
    end
catch
    coeff_out = coeff0;
    cond_val = inf;
    exitflag = 0;
end
end

function f = eval_objective(coeff, dim, order, period, sample_frequency, init_joint, cond_fun, optim_sample_freq)
% 有 cond_fun 时用优化采样频率（12 Hz）算条件数，否则用 config.sample_frequency
if nargin < 8, optim_sample_freq = []; end
N_full = round(period * sample_frequency);
if ~isempty(cond_fun) && ~isempty(optim_sample_freq)
    N_obj = max(40, round(period * optim_sample_freq));
elseif ~isempty(cond_fun)
    N_obj = min(N_full, 300);
else
    N_obj = N_full;
end
t_obj = linspace(0, period, N_obj)';
if t_obj(end) > period, t_obj(end) = period; end
[refPos, refVel, refAcc] = evaluate_fourier_trajectory(coeff, t_obj, dim, order, period, init_joint);
if ~isempty(cond_fun)
    f = cond_fun(refPos, refVel, refAcc);
else
    f = - (sum(var(refPos)) + sum(var(refVel)) + sum(var(refAcc)));
end
end

% 文档 4.6 优化阶段：离散点上不等式约束（满足时 c≤0），硬限；幅值约束；文档 4.5 自碰撞
function [c, ceq] = eval_constraints(coeff, t, dim, order, period, init_joint, ...
    lower_joint, upper_joint, max_vel, max_acc, config)
[refPos, refVel, refAcc] = evaluate_fourier_trajectory(coeff, t, dim, order, period, init_joint);
c = [];
% 幅值安全余量：sum_k(|a_jk|+|b_jk|) <= A_max_j => |q_j(t)-q_0j|<=A_max_j 保证在限位内
alpha = 0.7;
if isfield(config, 'amplitude_alpha') && ~isempty(config.amplitude_alpha)
    alpha = config.amplitude_alpha;
end
for j = 1:dim
    A_max_j = alpha * (upper_joint(j) - lower_joint(j)) / 2;
    a_start = dim + (j-1)*order + 1;
    a_end   = dim + j*order;
    b_start = dim + dim*order + (j-1)*order + 1;
    b_end   = dim + dim*order + j*order;
    c = [c; sum(abs(coeff(a_start:a_end))) + sum(abs(coeff(b_start:b_end))) - A_max_j];
end
for i = 1:size(refPos, 1)
    % 关节位置: q - q_max ≤ 0, q_min - q ≤ 0
    c = [c; refPos(i,:)' - upper_joint; lower_joint - refPos(i,:)'];
    % 关节速度/加速度: |qd| - v_max ≤ 0 等
    c = [c; abs(refVel(i,:)') - max_vel];
    c = [c; abs(refAcc(i,:)') - max_acc];
end
% 自碰撞：仅当 config.enable_collision_check=true 时加入约束
if isfield(config, 'enable_collision_check') && config.enable_collision_check && ...
        isfield(config, 'robot_limb') && ~isempty(config.robot_limb)
    try
        robot = config.robot_limb;
        for i = 1:size(refPos, 1)
            cfg = refPos(i,:);   % row，与 get_e1_limb_robot 的 DataFormat='row' 一致
            if size(cfg, 2) == 1, cfg = cfg'; end
            isColl = checkCollision(robot, cfg);
            c = [c; double(isColl) - 0.5];   % 碰撞则 0.5>0 违反，无碰撞则 -0.5≤0 满足
        end
    catch
        % 无碰撞几何或 checkCollision 不可用时跳过
    end
end
% 柱体避障（优化时仅检查末端连杆，减少约束维度；校验时检查所有连杆）
% 用平滑惩罚替代非凸"或"约束：overlap_x * overlap_y ≤ 0（在截面外时自然 ≤ 0）
if isfield(config, 'robot_limb') && ~isempty(config.robot_limb)
    pillar_c = get_pillar_config(config);   % [cx, cy, half_dx, half_dy]
    try
        robot_fk      = config.robot_limb;
        baseName_fk   = robot_fk.BaseName;
        end_body_name = robot_fk.Bodies{robot_fk.NumBodies}.Name;
        for i = 1:size(refPos, 1)
            cfg = refPos(i,:);
            if iscolumn(cfg), cfg = cfg'; end
            T    = getTransform(robot_fk, cfg, end_body_name, baseName_fk);
            px   = T(1,4); py = T(2,4);
            % 穿入量（>0 表示在该轴上与柱体重叠，≤0 表示已在该轴外侧）
            ov_x = pillar_c(3) - abs(px - pillar_c(1));
            ov_y = pillar_c(4) - abs(py - pillar_c(2));
            % 两个穿入量之积 > 0 ⟺ 同时在 x 和 y 轴均重叠（即在截面内）
            % fmincon 约束：ov_x * ov_y ≤ 0 → 加入 c
            c = [c; ov_x * ov_y];
        end
    catch
    end
end
% 扭矩约束：|τ| ≤ max_effort，与校验一致
if isfield(config, 'robot_limb') && ~isempty(config.robot_limb) && isfield(config, 'max_effort') && ~isempty(config.max_effort)
    max_effort = config.max_effort(:);
    if numel(max_effort) >= dim
        max_effort = max_effort(1:dim);
        for i = 1:size(refPos, 1)
            q = refPos(i,:); qd = refVel(i,:); qdd = refAcc(i,:);
            if iscolumn(q), q = q'; end
            if iscolumn(qd), qd = qd'; end
            if iscolumn(qdd), qdd = qdd'; end
            try
                tau = inverseDynamics(config.robot_limb, q, qd, qdd);
                tau = tau(:);
                c = [c; abs(tau) - max_effort];   % |τ_j| - effort_j ≤ 0
            catch
            end
        end
    end
end
ceq = [];
end

function pillar = get_pillar_config(config)
% 返回柱体参数 [cx, cy, half_dx, half_dy]（基坐标系 xy 平面截面，z 方向不限）
% 默认：中心 (-0.5, 0)，截面 0.2×0.2m → half = 0.1（即 x∈[-0.6,-0.4], y∈[-0.1,0.1]）
if isfield(config, 'pillar') && ~isempty(config.pillar)
    pillar = config.pillar(:)';   % 用户自定义 [cx, cy, half_dx, half_dy]
else
    pillar = [-0.5, 0.0, 0.1, 0.1];
end
end
