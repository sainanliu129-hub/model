function avg_data = cycle_average(t, q, tau, K, T, varargin)
% cycle_average  多周期激励数据的周期对齐 + 相位重采样 + 周期平均
%
% 用法示例：
%   avg = cycle_average(t, q, tau, 5, T);
%   avg = cycle_average(t, q, tau, 5, [], 'qd', qd, 'anchor_joint', 1);
%
% 输入：
%   t    - 时间序列 (N×1)
%   q    - 关节位置 (N×n)
%   tau  - 关节力矩 (N×n)
%   K    - 周期数（例如 5）
%   T    - 单周期时长 (s)，若为空则由数据估计
%
% 可选参数（Name-Value）：
%   'qd'           - 关节速度 (N×n)，若未提供将在相位均值上估算
%   'M'            - 相位网格长度（默认：round(N/K) 或 1000 取较大者）
%   'trim_ramp'    - 过渡段时长 (s)，该时段内不参与锚点检测（默认 0；有约 1s 过渡时可设 1）
%   'anchor_joint' - 用于锚点检测的关节索引（1～n）；0 表示自动选幅值最大的关节
%   'anchor_type'  - 'zero_cross' | 'peak' | 'fixed_period'（'fixed_period'=按 T 从 trim_ramp 起切 K 段，与生成激励的单周期一致）
%
% 输出 avg_data 结构体：
%   .phi      - 相位网格 (M×1)，[0,1)
%   .q_bar    - 周期平均位置 (M×n)
%   .qd_bar   - 周期平均速度 (M×n)（若输入 qd，则为平均后结果，否则为后续估算时的占位）
%   .qdd_bar  - 周期平均加速度 (M×n)，需结合周期 T 在外部调用时根据相位导数换算
%   .tau_bar  - 周期平均力矩 (M×n)
%   .tau_std  - 周期间标准差 (M×n)
%   .per_cycle - 每周期重采样数据（用于调试）：
%                .q{c}, .qd{c}, .tau{c}，尺寸均为 (M×n)

parser = inputParser;
parser.addRequired('t', @(x) isnumeric(x) && isvector(x));
parser.addRequired('q', @isnumeric);
parser.addRequired('tau', @isnumeric);
parser.addRequired('K', @(x) isscalar(x) && x > 0);
parser.addRequired('period_sec', @(x) isempty(x) || (isscalar(x) && x > 0));  % 单周期时长，避免与可选名 'T' 冲突
parser.addParameter('qd', [], @isnumeric);
parser.addParameter('M', [], @(x) isempty(x) || (isscalar(x) && x > 0));
parser.addParameter('trim_ramp', 0, @(x) isscalar(x) && x >= 0);
parser.addParameter('anchor_joint', 1, @(x) isscalar(x) && x >= 0);
parser.addParameter('anchor_type', 'zero_cross', @(s) ischar(s) && ismember(lower(s), {'zero_cross','peak','fixed_period'}));
parser.addParameter('ref_t', [], @(x) isempty(x) || (isnumeric(x) && isvector(x)));
parser.addParameter('ref_q', [], @(x) isempty(x) || isnumeric(x));
parser.addParameter('ref_qd', [], @(x) isempty(x) || isnumeric(x));
parser.parse(t, q, tau, K, T, varargin{:});
opts = parser.Results;
T = opts.period_sec;  % 后续代码仍用 T

% 先占位输出，避免内部报错时只显示「未对输出参数赋值」
avg_data = struct('phi', [], 'q_bar', [], 'qd_bar', [], 'qdd_bar', [], 'tau_bar', [], 'tau_std', [], 'per_cycle', struct());

t   = t(:);
N   = numel(t);
n   = size(q, 2);

if size(q,1) ~= N || size(tau,1) ~= N
    error('cycle_average: t, q, tau 行数需一致。');
end

if isempty(T)
    % 简单估计周期：总时长 / K
    T = (t(end) - t(1)) / K;
end

if isempty(opts.M)
    avg_per_period = round(N / K);
    opts.M = max(1000, avg_per_period);
end
M = opts.M;
use_ref_grid = ~isempty(opts.ref_t) && ~isempty(opts.ref_q) && numel(opts.ref_t) >= 2;
if use_ref_grid
    ref_t = opts.ref_t(:);
    phi = ref_t / T;   % 与生成激励的单周期时间一致，相位 [0,1]
    M = numel(phi);
else
    if ~isempty(opts.ref_t) && ~isempty(opts.ref_q) && numel(opts.ref_t) < 2
        warning('cycle_average: ref_t 点数 < 2，忽略参考网格，改用默认 M。');
    end
    phi = [];
end

% 过渡段裁剪：该时段内不参与锚点检测，避免把过渡当周期
trim_ramp = opts.trim_ramp;
idx_valid = true(N, 1);
if trim_ramp > 0
    idx_valid = t >= trim_ramp & t <= (t(end) - trim_ramp);
end

% 1. 周期边界：锚点检测 或 按固定周期 T 切分（与生成激励轨迹的单周期一致）
use_fixed_period = strcmpi(opts.anchor_type, 'fixed_period');
if use_fixed_period
    % 用生成激励时的 T：从「数据起点 + trim_ramp」起按周期切分（避免 t 不从 0 开始时首周期为空）
    t_start = t(1) + trim_ramp;
    anchors = zeros(K+1, 1);
    for c = 0:K
        idx_c = find(t >= t_start + c*T, 1);
        if isempty(idx_c)
            if c == 0
                anchors(1) = 1;
            else
                anchors(c+1) = N;  % 数据不足，最后一锚点到结尾
            end
        else
            anchors(c+1) = idx_c;
        end
    end
    if anchors(K+1) <= anchors(1)
        warning('cycle_average: fixed_period 下有效数据不足 K 个周期，请检查 trim_ramp 与 T。');
    end
else
    % 锚点检测（按指定关节）
    if opts.anchor_joint == 0
        [~, anchor_joint] = max(max(q) - min(q));
        anchor_joint = min(anchor_joint, n);
    else
        anchor_joint = min(max(1, opts.anchor_joint), n);
    end
    q_anchor = q(:, anchor_joint);
    window = max(5, round(0.01 * N));
    window = window + mod(window+1,2);
    q_f = movmean(q_anchor, window);
    dt = median(diff(t));
    qd_f = [diff(q_f)./dt; 0];

switch lower(opts.anchor_type)
    case 'zero_cross'
        % 三个条件都保持 (N-1)×1，避免广播成矩阵导致 find 返回线性下标越界
        idx_raw = find(q_f(1:end-1) <= 0 & q_f(2:end) > 0 & qd_f(2:end) > 0);
        % 只在有效区（去掉过渡段）内取穿越点
        if trim_ramp > 0
            idx_raw = idx_raw(t(idx_raw) >= trim_ramp & t(idx_raw) <= (t(end) - trim_ramp));
        end
        % 施加最小间隔约束（按 [0.8T, 1.2T]）
        min_dt = 0.8 * T;
        max_dt = 1.2 * T;
        anchors = [];
        last_t = -inf;
        for k = 1:numel(idx_raw)
            tk = t(idx_raw(k));
            if isempty(anchors)
                anchors(end+1,1) = idx_raw(k); %#ok<AGROW>
                last_t = tk;
            else
                if (tk - last_t) >= min_dt && (tk - last_t) <= max_dt
                    anchors(end+1,1) = idx_raw(k); %#ok<AGROW>
                    last_t = tk;
                end
            end
        end
    case 'peak'
        % 简单峰值检测：局部最大，间隔约 T，且只在有效区内
        anchors = [];
        last_t = -inf;
        for k = 2:N-1
            if ~idx_valid(k), continue; end
            if q_f(k) > q_f(k-1) && q_f(k) >= q_f(k+1)
                tk = t(k);
                if isempty(anchors) || (tk - last_t) >= 0.8*T
                    anchors(end+1,1) = k; %#ok<AGROW>
                    last_t = tk;
                end
            end
        end
    otherwise
        error('cycle_average: 未知 anchor_type: %s', opts.anchor_type);
end

if ~use_fixed_period && numel(anchors) < K+1
    % 锚点不足：在有效区（去掉过渡段）内按时间等间隔切分，避免把过渡段当周期
    warning('cycle_average: 有效锚点不足 K+1，在有效区内按等间隔切分（对齐可能偏差）。');
    idx_valid_idx = find(idx_valid);
    if numel(idx_valid_idx) < 2
        idx_valid_idx = (1:N)';
    end
    anchors = round(linspace(idx_valid_idx(1), idx_valid_idx(end), K+1));
    anchors = anchors(:);
elseif ~use_fixed_period
    % 只取前 K+1 个
    anchors = anchors(1:(K+1));
end
% fixed_period 时 anchors 已为 K+1 个，无需再裁

% 2. 构造统一相位网格（若已用 ref_t 提供则保持）
if isempty(phi)
    phi = (0:(M-1))' / M;  % [0,1)
end
if M < 1
    error('cycle_average: 相位网格点数 M = 0，请检查：1) 数据时间是否覆盖 t(1)+trim_ramp ～ t(1)+trim_ramp+K*T；2) 是否误传 M=0 或空 ref_t。');
end

% 3. 每周期重采样到统一相位
q_cyc   = cell(K,1);
qd_cyc  = cell(K,1);
tau_cyc = cell(K,1);

has_qd_input = ~isempty(opts.qd) && size(opts.qd,1) == N;
if has_qd_input
    qd = opts.qd;
else
    qd = [];
end

for c = 1:K
    k_start = anchors(c);
    k_end   = anchors(c+1);   % [k_start, k_end)
    idx_seg = k_start:(k_end-1);
    % 单周期(K=1)时允许周期内仅 1 点：复制成 2 点再插值，避免报错
    if numel(idx_seg) < 1
        error('cycle_average: 第 %d 周期无有效点（anchors 重叠），请检查 trim_ramp 与 T。', c);
    end
    if numel(idx_seg) == 1
        q_seg   = q(idx_seg, :);
        tau_seg = tau(idx_seg, :);
        q_seg   = [q_seg; q_seg];
        tau_seg = [tau_seg; tau_seg];
        phi_c   = [0; 1];
    else
        phi_c   = (0:(numel(idx_seg)-1))' / (numel(idx_seg)-1);  % [0,1]
        q_seg   = q(idx_seg, :);
        tau_seg = tau(idx_seg, :);
    end
    % 使用 interp1 插值到 phi
    q_cyc{c}   = interp1(phi_c, q_seg,   phi, 'pchip', 'extrap');
    tau_cyc{c} = interp1(phi_c, tau_seg, phi, 'pchip', 'extrap');
    if has_qd_input
        if numel(idx_seg) == 1
            qd_seg = [qd(idx_seg, :); qd(idx_seg, :)];
        else
            qd_seg = qd(idx_seg, :);
        end
        qd_cyc{c}   = interp1(phi_c, qd_seg, phi, 'pchip', 'extrap');
    else
        qd_cyc{c} = [];
    end
end

% 4. 周期平均与标准差
q_stack   = zeros(M, n, K);
tau_stack = zeros(M, n, K);
qd_stack  = [];
if has_qd_input
    qd_stack = zeros(M, n, K);
end

for c = 1:K
    q_stack(:,:,c)   = q_cyc{c};
    tau_stack(:,:,c) = tau_cyc{c};
    if has_qd_input
        qd_stack(:,:,c) = qd_cyc{c};
    end
end

q_bar   = mean(q_stack, 3);
tau_bar = mean(tau_stack, 3);
tau_std = std(tau_stack, 0, 3);

if has_qd_input
    qd_bar = mean(qd_stack, 3);
else
    qd_bar = zeros(M, n);  % 占位，后续可在相位域上估算
end

% 5. 加速度 qdd_bar：在均值上求导（方案文档推荐）
%    - 有 qd：保留 qd_bar = mean(qd)；qdd = d(qd_bar)/d(phi) * 1/T（对平均速度求导）
%    - 无 qd：qd_bar 与 qdd_bar 均由 q_bar 在相位上求导后乘 1/T、1/T^2
qdd_bar = zeros(M, n);
if use_ref_grid && M > 1
    dphi = mean(diff(phi));
else
    dphi = 1/M;
end
for j = 1:n
    if has_qd_input
        % 有采集速度：用平均速度对 φ 求导得加速度，不覆盖 qd_bar
        qd_b = qd_bar(:, j);
        ddq_phi = zeros(M, 1);
        if M >= 3
            ddq_phi(2:M-1) = (qd_b(3:M) - qd_b(1:M-2)) / (2*dphi);
            ddq_phi(1)    = (qd_b(2) - qd_b(1)) / dphi;
            ddq_phi(M)    = (qd_b(M) - qd_b(M-1)) / dphi;
        end
        qdd_bar(:, j) = ddq_phi / T;   % d(qd)/d(phi) * (1/T) = d(qd)/dt
    else
        % 无 qd：由平均角度求一阶、二阶导
        qb = q_bar(:, j);
        dq_phi = zeros(M,1);
        ddq_phi = zeros(M,1);
        if M >= 3
            dq_phi(2:M-1)  = (qb(3:M) - qb(1:M-2)) / (2*dphi);
            dq_phi(1)      = (qb(2) - qb(1)) / dphi;
            dq_phi(M)      = (qb(M) - qb(M-1)) / dphi;
            ddq_phi(2:M-1) = (qb(3:M) - 2*qb(2:M-1) + qb(1:M-2)) / (dphi^2);
            ddq_phi(1)     = ddq_phi(2);
            ddq_phi(M)     = ddq_phi(M-1);
        end
        qd_bar(:, j)  = dq_phi / T;
        qdd_bar(:, j) = ddq_phi / (T^2);
    end
end

% 输出结构体
avg_data = struct();
avg_data.phi      = phi;
avg_data.q_bar    = q_bar;
avg_data.qd_bar   = qd_bar;
avg_data.qdd_bar  = qdd_bar;
avg_data.tau_bar  = tau_bar;
avg_data.tau_std  = tau_std;
avg_data.per_cycle.q  = q_cyc;
avg_data.per_cycle.qd = qd_cyc;
avg_data.per_cycle.tau= tau_cyc;

end

