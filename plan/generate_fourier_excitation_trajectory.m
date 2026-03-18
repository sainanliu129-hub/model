function generate_fourier_excitation_trajectory(varargin)
% generate_fourier_excitation_trajectory  生成基于傅里叶级数的激励轨迹
%
% 用于动力学参数辨识/模型精度测试（Part B），激励丰富、回归矩阵条件数更好。
% 公式：q_j(t) = sum_{i=1}^{N} (1/sqrt(i)) * [ a_ij*sin(i*omega*t + phi_j) ]
%  - 仅正弦项且整周期时 q(0)=q(T)=0，便于起止0位
%  - 各关节相位 phi_j 错开，提高激励独立性
%  - 幅值按URDF限位缩放，保证安全
%
% 输入参数（可选）:
%   'Ts'              - 采样周期，默认 0.002 s (500 Hz)
%   'urdf_file'       - URDF文件路径
%   'duration'        - 傅里叶段总时长(秒)，默认 20
%   'n_harmonics'     - 谐波数 N，默认 5
%   'n_periods'       - 基频周期数（整周期保证起止为0），默认 1
%   'output_file'     - 输出CSV文件名
%
% 速度限值：从URDF读取 velocity，超限则对该关节轨迹整体缩放。
% 碰撞检测（与往返轨迹一致）：两条腿单独激励。
%   - 左腿激励时：左腿 6 关节做傅里叶，右腿处于安全位（roll=lower，其余 0）。
%   - 右腿激励时：右腿 6 关节做傅里叶，左腿处于安全位（roll=upper，其余 0）。
%   - 当前腿的 roll 为 0，另一条腿的 roll 为安全位。
%
% 输出CSV格式：time, leg_l1_joint, ..., leg_r6_joint，起止为0位（含过渡段）。

p = inputParser;
addParameter(p, 'Ts', 0.002, @isnumeric);
addParameter(p, 'urdf_file', '', @ischar);
addParameter(p, 'duration', 20, @isnumeric);
addParameter(p, 'n_harmonics', 5, @isnumeric);
addParameter(p, 'n_periods', 1, @isnumeric);  % 整周期数，使 q(0)=q(T)=0
addParameter(p, 'transition_time', 1.0, @isnumeric);
addParameter(p, 'output_file', '', @ischar);
parse(p, varargin{:});
opts = p.Results;

fprintf('===== 生成傅里叶级数激励轨迹 =====\n');
fprintf('采样周期: %.4f s (%.0f Hz)\n', opts.Ts, 1/opts.Ts);
fprintf('傅里叶段时长: %.1f s, 谐波数: %d, 周期数: %d\n', ...
    opts.duration, opts.n_harmonics, opts.n_periods);

%% 查找URDF
if isempty(opts.urdf_file)
    current_dir = fileparts(mfilename('fullpath'));
    possible_paths = {
        fullfile('noetix_description', 'urdf', 'E1.urdf');
        fullfile(current_dir, '..', '..', '..', 'noetix_description', 'urdf', 'E1.urdf');
        fullfile(current_dir, '..', '..', 'noetix_description', 'urdf', 'E1.urdf');
        fullfile(pwd, 'noetix_description', 'urdf', 'E1.urdf');
    };
    for i = 1:length(possible_paths)
        if exist(possible_paths{i}, 'file')
            opts.urdf_file = possible_paths{i};
            break;
        end
    end
end
if isempty(opts.urdf_file) || ~exist(opts.urdf_file, 'file')
    error('未找到URDF文件，请指定 urdf_file 或确保 noetix_description/urdf/E1.urdf 存在');
end
fprintf('URDF: %s\n', opts.urdf_file);

%% 关节名称与映射（与 generate_multi_speed_trajectory 一致）
all_joint_names = {
    'l_leg_hip_yaw_joint'; 'l_leg_hip_roll_joint'; 'l_leg_hip_pitch_joint';
    'l_leg_knee_joint'; 'l_leg_ankle_pitch_joint'; 'l_leg_ankle_roll_joint';
    'r_leg_hip_yaw_joint'; 'r_leg_hip_roll_joint'; 'r_leg_hip_pitch_joint';
    'r_leg_knee_joint'; 'r_leg_ankle_pitch_joint'; 'r_leg_ankle_roll_joint'
};
joint_name_mapping = containers.Map(...
    {'l_leg_hip_yaw_joint','l_leg_hip_roll_joint','l_leg_hip_pitch_joint','l_leg_knee_joint','l_leg_ankle_pitch_joint','l_leg_ankle_roll_joint', ...
     'r_leg_hip_yaw_joint','r_leg_hip_roll_joint','r_leg_hip_pitch_joint','r_leg_knee_joint','r_leg_ankle_pitch_joint','r_leg_ankle_roll_joint'}, ...
    {'leg_l1_joint','leg_l2_joint','leg_l3_joint','leg_l4_joint','leg_l5_joint','leg_l6_joint', ...
     'leg_r1_joint','leg_r2_joint','leg_r3_joint','leg_r4_joint','leg_r5_joint','leg_r6_joint'});

%% 从URDF读取关节限位
xmlDoc = xmlread(opts.urdf_file);
jointNodes = xmlDoc.getElementsByTagName('joint');
numJoints = jointNodes.getLength();
joint_params = struct();
for i = 1:length(all_joint_names)
    joint_name = all_joint_names{i};
    if ~joint_name_mapping.isKey(joint_name), continue; end
    urdf_name = joint_name_mapping(joint_name);
    found = false;
    for j = 0:numJoints - 1
        node = jointNodes.item(j);
        if ~strcmp(char(node.getAttribute('name')), urdf_name), continue; end
        limit = node.getElementsByTagName('limit').item(0);
        if isempty(limit), break; end
        lower = str2double(limit.getAttribute('lower'));
        upper = str2double(limit.getAttribute('upper'));
        velocity = str2double(limit.getAttribute('velocity'));
        if isnan(velocity) || velocity <= 0, velocity = 5.0; end
        joint_params.(joint_name).lower = lower * 0.9;
        joint_params.(joint_name).upper = upper * 0.9;
        joint_params.(joint_name).velocity_max = velocity;
        found = true;
        break;
    end
    if ~found
        joint_params.(joint_name).lower = -pi;
        joint_params.(joint_name).upper = pi;
        joint_params.(joint_name).velocity_max = 5.0;
    end
end

% 碰撞检测：roll_joint 安全位置（与 generate_multi_speed_trajectory 一致）
roll_left_idx = 2;   % l_leg_hip_roll_joint
roll_right_idx = 8; % r_leg_hip_roll_joint
safe_roll_left = 0;
safe_roll_right = 0;
if isfield(joint_params, 'l_leg_hip_roll_joint')
    safe_roll_left = joint_params.l_leg_hip_roll_joint.upper;  % 左腿 roll 最大
end
if isfield(joint_params, 'r_leg_hip_roll_joint')
    safe_roll_right = joint_params.r_leg_hip_roll_joint.lower; % 右腿 roll 最小
end
fprintf('碰撞检测：两条腿单独激励，当前腿激励时另一条腿处于安全位（与往返轨迹一致）\n');

T_fourier = opts.duration;
omega = 2*pi*opts.n_periods / T_fourier;
N = opts.n_harmonics;
n_trans = max(round(opts.transition_time / opts.Ts), 10);
zero_pos = zeros(1, 12);

% 左腿关节索引 1..6，右腿 7..12；当前腿 roll=0，另一腿 roll=安全位
left_leg_idx = [1, 2, 3, 4, 5, 6];   % 左腿：1 yaw, 2 roll, 3 pitch, 4 knee, 5 ankle_pitch, 6 ankle_roll
right_leg_idx = [7, 8, 9, 10, 11, 12];
left_excite_idx = [1, 3, 4, 5, 6];    % 左腿参与傅里叶的关节（不含 roll）
right_excite_idx = [7, 9, 10, 11, 12];

t_base = (0:opts.Ts:T_fourier)';
if t_base(end) < T_fourier - opts.Ts*0.5
    t_base = [t_base; T_fourier];
end
n_pts = length(t_base);

% ---------- 左腿激励段 ----------
% 左腿：关节 1,3,4,5,6 傅里叶，关节 2(roll)=0；右腿：关节 8(roll)=safe_roll_right，7,9,10,11,12=0
rng(42);
a_left = 0.5 + 0.5*rand(N, 5);  % 5 个关节
phi_left = 2*pi*(0:4)'/5;
q_left = zeros(n_pts, 12);
for k = 1:5
    j = left_excite_idx(k);
    for i = 1:N
        q_left(:, j) = q_left(:, j) + (1/sqrt(i)) * a_left(i,k) * sin(i*omega*t_base + phi_left(k));
    end
end
q_left(:, roll_left_idx) = 0;
q_left(:, 7) = 0; q_left(:, roll_right_idx) = safe_roll_right; q_left(:, 9:12) = 0;

% 左腿段：位置与速度限值
for j = left_excite_idx
    jname = all_joint_names{j};
    if ~isfield(joint_params, jname), continue; end
    low = joint_params.(jname).lower; upp = joint_params.(jname).upper;
    qj = q_left(:,j); q_max = max(qj); q_min = min(qj);
    if q_max - q_min < 1e-9, continue; end
    margin = 0.9;
    scale_upp = margin*upp/max(q_max,1e-9); if q_max<=1e-9, scale_upp=1e6; end
    scale_low = margin*(-low)/max(-q_min,1e-9); if q_min>=-1e-9, scale_low=1e6; end
    scale = min(scale_upp, scale_low); if scale>1e5, scale=1; end
    q_left(:,j) = q_left(:,j) * scale;
end
qd_left = [zeros(1,12); diff(q_left,1,1)/opts.Ts]; qd_left(1,:) = qd_left(2,:);
for j = left_excite_idx
    jname = all_joint_names{j};
    if ~isfield(joint_params, jname), continue; end
    v_max = joint_params.(jname).velocity_max;
    v_cur = max(abs(qd_left(:,j)));
    if v_cur > v_max
        scale_vel = 0.9 * v_max / v_cur;
        q_left(:,j) = q_left(:,j) * scale_vel;
        fprintf('  左腿关节 %s 速度超限，缩放 %.3f\n', jname, scale_vel);
    end
end

% ---------- 右腿激励段 ----------
rng(43);
a_right = 0.5 + 0.5*rand(N, 5);
phi_right = 2*pi*(0:4)'/5;
q_right = zeros(n_pts, 12);
for k = 1:5
    j = right_excite_idx(k);
    for i = 1:N
        q_right(:, j) = q_right(:, j) + (1/sqrt(i)) * a_right(i,k) * sin(i*omega*t_base + phi_right(k));
    end
end
q_right(:, roll_right_idx) = 0;
q_right(:, [1,3,4,5,6]) = 0;
q_right(:, roll_left_idx) = safe_roll_left;  % 左腿安全位

for j = right_excite_idx
    jname = all_joint_names{j};
    if ~isfield(joint_params, jname), continue; end
    low = joint_params.(jname).lower; upp = joint_params.(jname).upper;
    qj = q_right(:,j); q_max = max(qj); q_min = min(qj);
    if q_max - q_min < 1e-9, continue; end
    margin = 0.9;
    scale_upp = margin*upp/max(q_max,1e-9); if q_max<=1e-9, scale_upp=1e6; end
    scale_low = margin*(-low)/max(-q_min,1e-9); if q_min>=-1e-9, scale_low=1e6; end
    scale = min(scale_upp, scale_low); if scale>1e5, scale=1; end
    q_right(:,j) = q_right(:,j) * scale;
end
qd_right = [zeros(1,12); diff(q_right,1,1)/opts.Ts]; qd_right(1,:) = qd_right(2,:);
for j = right_excite_idx
    jname = all_joint_names{j};
    if ~isfield(joint_params, jname), continue; end
    v_max = joint_params.(jname).velocity_max;
    v_cur = max(abs(qd_right(:,j)));
    if v_cur > v_max
        scale_vel = 0.9 * v_max / v_cur;
        q_right(:,j) = q_right(:,j) * scale_vel;
        fprintf('  右腿关节 %s 速度超限，缩放 %.3f\n', jname, scale_vel);
    end
end
fprintf('速度限值：已按 URDF velocity 检查\n');

%% 拼接：起过渡 + 左腿段 + 左右切换过渡 + 右腿段 + 止过渡
% 段1：0 -> 左腿段首（左=首点、右=安全位）
q_first_left = q_left(1,:);
t_start = linspace(0, opts.transition_time, n_trans)';
q_start = (1 - (1:n_trans)'/n_trans) * zero_pos + (1:n_trans)'/n_trans * q_first_left;

t_current = opts.transition_time;
t_left = t_base + t_current;
t_current = t_left(end) + opts.Ts;

% 段2：左腿段末 -> 右腿段首（左腿从末点到安全位，右腿从安全位到首点）
q_left_end = q_left(end,:);  % 左末（近0），右=安全
q_right_start = q_right(1,:); % 左=安全，右首（近0）
t_mid = t_current + linspace(0, opts.transition_time, n_trans)';
q_mid = zeros(n_trans, 12);
for i = 1:n_trans
    alpha = (i-1) / max(n_trans-1, 1);
    q_mid(i,:) = (1-alpha)*q_left_end + alpha*q_right_start;
end
t_current = t_mid(end) + opts.Ts;

% 段3：右腿傅里叶段
t_right = t_base + t_current;
t_current = t_right(end) + opts.Ts;

% 段4：右腿段末 -> 0
q_right_end = q_right(end,:);
t_end = t_current + linspace(0, opts.transition_time, n_trans)';
if n_trans <= 1
    q_end = zero_pos;
else
    alpha = (0:n_trans-1)' / (n_trans-1);
    q_end = (1-alpha)*q_right_end + alpha*zero_pos;
end

t_all = [t_start; t_left; t_mid; t_right; t_end];
q_all = [q_start; q_left; q_mid; q_right; q_end];

fprintf('轨迹结构：起过渡 -> 左腿激励(%.1fs) -> 左右切换过渡 -> 右腿激励(%.1fs) -> 止过渡\n', T_fourier, T_fourier);
fprintf('总点数: %d, 总时长: %.3f s\n', size(t_all,1), t_all(end));

%% 写CSV（与生成轨迹格式一致，默认写入 build/plan）
if isempty(opts.output_file)
    opts.output_file = fullfile(get_build_dir('plan'), ...
        sprintf('fourier_excitation_trajectory_%s.csv', datestr(now, 'yyyymmdd_HHMMSS')));
end
fid = fopen(opts.output_file, 'w');
if fid == -1
    error('无法创建文件: %s', opts.output_file);
end
fprintf(fid, 'time');
for i = 1:12
    fprintf(fid, ',%s', joint_name_mapping(all_joint_names{i}));
end
fprintf(fid, '\n');
for i = 1:size(t_all,1)
    fprintf(fid, '%.5f', t_all(i));
    for j = 1:12
        fprintf(fid, ',%.6f', q_all(i,j));
    end
    fprintf(fid, '\n');
end
fclose(fid);
fprintf('已保存: %s\n', opts.output_file);

%% 简单预览
figure('Name', '傅里叶激励轨迹');
for j = 1:6
    subplot(2,3,j);
    plot(t_all, q_all(:,j), 'b-', 'LineWidth', 1);
    grid on; xlabel('时间 (s)'); ylabel('位置 (rad)');
    title(joint_name_mapping(all_joint_names{j}), 'Interpreter','none');
end
sgtitle('傅里叶激励轨迹（前6关节）');
fprintf('===== 完成 =====\n');
end
