% run_identify_joint_friction  拆下关节单独辨识摩擦参数示例脚本
%
% 流程（对应文档 5. 流程小结）:
%   1. 多档匀速正转：记录 q、q̇、i → τ_real = K_τ × i
%   2. 多档匀速反转：记录 q、q̇、i → τ_real
%   3. 合并所有速度档的 (qd, i) 数据
%   4. 调用 identify_joint_friction 拟合 τ_c、b（及可选 τ_s）
%
% 本脚本使用仿真数据演示；真机时请替换为从文件/采集卡读取的 q、qd、i。
%
% 轨迹生成方案（与 generate_joint_trajectory 一致）:
%   - 每个速度档的位置都从0开始（不累积）
%   - 位置平滑过渡：速度档切换时先减速回到0，再开始新速度档（无跳变）
%   - 低速档：正负交替（+0.01, -0.01, +0.02, -0.02, ...）
%   - 中速档和高速档：先所有正速度，再所有负速度
%   - 时间连续，位置平滑过渡
%
% 数据格式约定（与文档一致）:
%   - 时间、角度 q、角速度 qd、电流 i，采样率建议 ≥ 100 Hz（本脚本使用500Hz）
%   - 匀速段：速度环控制，每档速度转约 1 圈以上
%   - 真机数据格式：data = [序号, 目标位置, 当前位置, 目标速度, 当前速度, 目标电流, 当前电流, 温度]

clear;
clc;
close all;

%% 1. 参数设置
K_tau = 0.5;   % 力矩系数 N·m/A（需已标定）
Ts = 1/500;    % 采样周期 2 ms (500 Hz，与轨迹生成一致)
v_max = 2.0;   % 关节最大速度 rad/s（根据实际关节模块设置）

%% 2. 多档速度测试设置（参考文档步骤4）
% 2.1 低速档：±0.01～±0.3 rad/s，步长 0.01，每档 10 s（约1000个采样点）
v_low = 0.01:0.01:0.3;
v_low = repelem(v_low, 2);
v_low(2:2:end) = -v_low(2:2:end);  % 正负交替
n_low = round(10 / Ts);  % 每档10秒

% 2.2 零速：20 s
v_zero = 0;
n_zero = round(20 / Ts);

% 2.3 中速档：±0.4～±0.9 rad/s，步长 0.1，每档转 1.75 圈
v_mid = 0.4:0.1:0.9;
v_mid_all = [v_mid, -v_mid];  % 正转和反转
% 每档转1.75圈所需时间 = 1.75 * 2*pi / |v|，采样点数 = 时间 / Ts
n_mid = round(1.75 * 2 * pi ./ abs(v_mid) / Ts);

% 2.4 高速档：±1.0～±vmax，约 10 档等分，每档转 1.75 圈
v_high = linspace(1.0, v_max, 10);
v_high_all = [v_high, -v_high];  % 正转和反转
n_high = round(1.75 * 2 * pi ./ abs(v_high) / Ts);

% 合并所有速度档（按文档顺序：低速正负交替、零速、中速、高速）
speed_list = [v_low, v_zero, v_mid_all, v_high_all];

%% 3. 数据准备（真机时改为从文件加载或实时采集）
% 真机数据格式：data = [序号, 目标位置, 当前位置, 目标速度, 当前速度, 目标电流, 当前电流, 温度]
% 仿真数据生成（用于演示）
fprintf('生成仿真测试数据...\n');
tau_c_true = 0.08;  % 真实库伦摩擦
b_true = 0.02;      % 真实粘滞摩擦系数
tau_s_true = 0.10;  % 真实静摩擦

qd_all = [];
i_all = [];
q_all = [];
t_all = [];
t_curr = 0;

for k = 1:length(speed_list)
    v = speed_list(k);
    
    % 确定该速度档的采样点数
    if abs(v) < 0.01  % 零速
        n_k = n_zero;
    elseif abs(v) <= 0.3  % 低速
        n_k = n_low;
    elseif abs(v) <= 0.9  % 中速
        idx_mid = find(abs(v_mid - abs(v)) < 0.05);
        if ~isempty(idx_mid)
            n_k = n_mid(idx_mid(1));
        else
            n_k = round(1.75 * 2 * pi / abs(v) / Ts);
        end
    else  % 高速
        idx_high = find(abs(v_high - abs(v)) < 0.1);
        if ~isempty(idx_high)
            n_k = n_high(idx_high(1));
        else
            n_k = round(1.75 * 2 * pi / abs(v) / Ts);
        end
    end
    
    % 生成该速度档的数据
    t_k = t_curr + (0:(n_k-1))' * Ts;
    qd_k = v * ones(n_k, 1);
    
    % 位置积分：每个速度档都从0开始（与轨迹生成方案一致）
    % 使用相对时间计算位置，每个速度档独立从0开始
    t_rel = (0:(n_k-1))' * Ts;  % 该速度档内的相对时间（从0开始）
    q_k = v * t_rel;  % 位置从0开始：q = v * t_rel
    
    % 计算摩擦力矩（运动段）
    if abs(v) < 0.01  % 零速：静摩擦
        tau_f_k = tau_s_true * (2*rand(n_k,1) - 1);  % 静摩擦范围内随机
        tau_f_k(abs(tau_f_k) > tau_s_true) = sign(tau_f_k(abs(tau_f_k) > tau_s_true)) * tau_s_true;
    else  % 运动段：库伦+粘滞
        tau_f_k = tau_c_true * sign(v) + b_true * v;
    end
    
    % 电流 = 摩擦力矩 / K_tau + 噪声
    i_k = tau_f_k / K_tau + 0.002 * randn(n_k, 1);
    
    % 合并数据
    qd_all = [qd_all; qd_k];
    i_all = [i_all; i_k];
    q_all = [q_all; q_k];
    t_all = [t_all; t_k];
    
    t_curr = t_all(end) + Ts;
end

%% 4. 数据处理：提取稳定段（适配新轨迹方案：每个速度档从0开始，位置平滑过渡）
% 4.1 找到速度变化点（用于分离不同速度档）
% 注意：位置平滑过渡，不能通过位置跳变检测
fprintf('处理数据：提取稳定段...\n');
speed_change_idx = [];

% 检测速度变化
for i = 2:length(qd_all)
    if abs(qd_all(i) - qd_all(i-1)) > 0.005  % 速度变化超过阈值
        speed_change_idx = [speed_change_idx; i];
    end
end

% 可选：检测位置接近0的点（每个速度档从0开始，但位置平滑过渡）
pos_zero_idx = [];
for i = 2:length(q_all)
    if abs(q_all(i)) < 0.05 && abs(qd_all(i)) > 0.001
        % 检查是否是速度档开始（前一个点速度接近0或不同）
        if i > 1 && (abs(qd_all(i-1)) < 0.001 || sign(qd_all(i)) ~= sign(qd_all(i-1)))
            pos_zero_idx = [pos_zero_idx; i];
        end
    end
end

% 合并检测到的变化点
all_change_idx = unique([speed_change_idx; pos_zero_idx]);
speed_change_idx = sort(all_change_idx);

% 4.2 对每个速度档提取稳定段（去掉启动和结束段）
% 稳定段：去掉前100个点和后50个点（参考代码）
stable_start = 100;
stable_end = 50;

qd_stable = [];
i_stable = [];
q_stable = [];

speed_segments = [1; speed_change_idx; length(qd_all)+1];
for seg = 1:length(speed_segments)-1
    start_idx = speed_segments(seg);
    end_idx = speed_segments(seg+1) - 1;
    
    if end_idx - start_idx > stable_start + stable_end
        stable_idx = (start_idx + stable_start):(end_idx - stable_end);
        qd_stable = [qd_stable; qd_all(stable_idx)];
        i_stable = [i_stable; i_all(stable_idx)];
        q_stable = [q_stable; q_all(stable_idx)];
    end
end

% 如果没有找到速度变化点，使用全部数据（去掉首尾）
if isempty(speed_change_idx)
    if length(qd_all) > stable_start + stable_end
        stable_idx = (stable_start+1):(length(qd_all)-stable_end);
        qd_stable = qd_all(stable_idx);
        i_stable = i_all(stable_idx);
        q_stable = q_all(stable_idx);
    else
        qd_stable = qd_all;
        i_stable = i_all;
        q_stable = q_all;
    end
end

%% 5. 静摩擦测试（可选，参考文档步骤4）
% 在 0°、90°、180°、270° 四个角度、正反向各做多次
fprintf('进行静摩擦测试...\n');
test_angles = [0, pi/2, pi, 3*pi/2];  % 四个角度
tau_s_estimates = [];

for angle = test_angles
    % 正反向测试（仿真中简化处理）
    % 真机时：移动到指定角度，然后缓慢增加电流，记录刚能维持微小恒速时的电流
    for direction = [-1, 1]
        % 仿真：在零速附近测试
        v_test = direction * 0.001;  % 微小速度
        tau_f_test = tau_s_true * direction + 0.01 * randn();
        i_test = tau_f_test / K_tau;
        tau_s_estimates = [tau_s_estimates; abs(i_test * K_tau)];
    end
end

tau_s_estimated = mean(tau_s_estimates);

%% 6. 调用辨识函数（使用稳定段数据）
fprintf('调用辨识函数拟合摩擦参数...\n');
Result = identify_joint_friction(qd_stable, i_stable, K_tau, ...
    'EstimateTauS', true, ...   % 估计静摩擦
    'DoPlot', true, ...
    'JointName', '示例关节');

% 如果静摩擦估计失败，使用步骤5的结果
if isnan(Result.tau_s) || Result.tau_s == 0
    Result.tau_s = tau_s_estimated;
end

%% 7. 加速度试验辨识 I_a（可选）
% 步骤7：做带加速度的试验，辨识 I_a
%   - 生成加速度轨迹（如正弦）
%   - 记录 q、q̇、i，数值微分后滤波得 q̈
%   - 方程：τ_real − τ_friction(q̇) = I_a × q̈
%     * τ_real = K_τ × i（已知）
%     * τ_friction(q̇) 用已辨识的 τ_c、b 计算
%     * q̈ 由 q、q̇ 数值微分后滤波得到
%
do_accel_test = true;  % 是否进行加速度试验
if do_accel_test
    fprintf('\n===== 步骤7：加速度试验辨识 I_a =====\n');
    fprintf('生成加速度测试轨迹...\n');
    fprintf('提示：真机测试时建议使用 generate_joint_trajectory(''acceleration'') 生成恒定加速度轨迹\n');
    
    % 正弦轨迹参数
    freq_list = [0.1, 0.2, 0.5, 1.0, 2.0];  % 频率列表 Hz
    amplitude = 0.5;  % 振幅 rad
    duration_per_freq = 10;  % 每个频率持续时间 s
    
    I_a_true = 0.001;  % 真实电机转子惯量 kg·m²（仿真用）
    
    q_accel = [];
    qd_accel = [];
    qdd_accel = [];
    i_accel = [];
    t_accel = [];
    t_curr_accel = 0;
    
    for freq = freq_list
        omega = 2 * pi * freq;
        n_points = round(duration_per_freq / Ts);
        t_freq = t_curr_accel + (0:(n_points-1))' * Ts;
        
        % 正弦轨迹：q = A*sin(ωt)
        q_freq = amplitude * sin(omega * t_freq);
        qd_freq = amplitude * omega * cos(omega * t_freq);
        qdd_freq = -amplitude * omega^2 * sin(omega * t_freq);
        
        % 计算摩擦力矩（使用已辨识的参数）
        tau_friction = Result.tau_c * sign(qd_freq) + Result.b * qd_freq;
        
        % 计算总力矩：τ_real = I_a × q̈ + τ_friction
        tau_real_accel = I_a_true * qdd_freq + tau_friction;
        
        % 电流 = 总力矩 / K_tau + 噪声
        i_freq = tau_real_accel / K_tau + 0.002 * randn(n_points, 1);
        
        % 合并数据
        q_accel = [q_accel; q_freq];
        qd_accel = [qd_accel; qd_freq];
        qdd_accel = [qdd_accel; qdd_freq];
        i_accel = [i_accel; i_freq];
        t_accel = [t_accel; t_freq];
        
        % 更新当前时间（用于下一段，保持时间连续）
        t_curr_accel = t_accel(end) + Ts;
    end
    
    % 7.2 获取加速度 q̈：数值微分后滤波
    if exist('qdd_accel', 'var') && ~isempty(qdd_accel) && length(qdd_accel) == length(qd_accel)
        qdd_filtered = qdd_accel;  % 仿真数据：直接使用
    else
        % 真机数据：从速度数值微分 q̈ = diff(q̇) / Ts，然后滤波
        qdd_numerical = diff(qd_accel) / Ts;
        qdd_numerical = [qdd_numerical(1); qdd_numerical];
        qdd_filtered = movmean(qdd_numerical, 5);  % 移动平均滤波
    end
    
    % 7.3 计算 τ_real 和 τ_friction
    tau_real_accel = K_tau * i_accel;  % τ_real = K_τ × i
    tau_friction_accel = Result.tau_c * sign(qd_accel) + Result.b * qd_accel;  % 用已辨识的 τ_c、b
    
    % 7.4 线性回归拟合 I_a：τ_real − τ_friction = I_a × q̈
    tau_net = tau_real_accel - tau_friction_accel;  % 净力矩（用于加速）
    
    % 去除异常值（加速度过小的点，信噪比低）
    idx_valid = abs(qdd_filtered) > 0.01;  % 加速度阈值
    
    if sum(idx_valid) > 10
        x_fit = qdd_filtered(idx_valid);
        y_fit = tau_net(idx_valid);
        
        % 线性回归：y = I_a * x
        I_a_estimated = x_fit \ y_fit;  % 最小二乘
        
        % 计算拟合优度
        y_fit_pred = I_a_estimated * x_fit;
        ss_res = sum((y_fit - y_fit_pred).^2);
        ss_tot = sum((y_fit - mean(y_fit)).^2);
        R2_Ia = 1 - ss_res / ss_tot;
        RMSE_Ia = sqrt(mean((y_fit - y_fit_pred).^2));
        
        fprintf('  电机转子惯量 I_a = %.6f kg·m²\n', I_a_estimated);
        fprintf('  拟合 R² = %.4f\n', R2_Ia);
        fprintf('  拟合 RMSE = %.6f N·m\n', RMSE_Ia);
        fprintf('  有效样本数 = %d\n', sum(idx_valid));
        
        % 与真值对比（仅仿真时）
        if exist('I_a_true', 'var')
            fprintf('  （仿真真值: I_a=%.6f, 误差=%.2f%%）\n', ...
                I_a_true, abs(I_a_estimated - I_a_true)/I_a_true*100);
        end
        
        % 7.6 可视化结果
        figure('Name', '加速度试验：I_a 辨识');
        subplot(2,1,1);
        plot(t_accel, qdd_filtered, 'b-', 'LineWidth', 1);
        hold on;
        plot(t_accel(idx_valid), qdd_filtered(idx_valid), 'r.', 'MarkerSize', 8);
        grid on;
        xlabel('时间 (s)');
        ylabel('角加速度 q̈ (rad/s²)');
        title('角加速度轨迹（红色点为用于拟合的点）');
        legend('滤波后加速度', '有效拟合点', 'Location', 'best');
        
        subplot(2,1,2);
        plot(qdd_filtered(idx_valid), tau_net(idx_valid), 'b.', 'MarkerSize', 8);
        hold on;
        qdd_plot = linspace(min(qdd_filtered(idx_valid)), max(qdd_filtered(idx_valid)), 100);
        tau_plot = I_a_estimated * qdd_plot;
        plot(qdd_plot, tau_plot, 'r-', 'LineWidth', 2);
        grid on;
        xlabel('角加速度 q̈ (rad/s²)');
        ylabel('净力矩 τ_{real} - τ_{friction} (N·m)');
        title(sprintf('I_a 拟合: I_a = %.6f kg·m², R² = %.4f', I_a_estimated, R2_Ia));
        legend('实测数据', sprintf('拟合直线 (I_a=%.6f)', I_a_estimated), 'Location', 'best');
        
        % 保存结果
        Result.I_a = I_a_estimated;
        Result.I_a_R2 = R2_Ia;
        Result.I_a_RMSE = RMSE_Ia;
    else
        fprintf('  警告：有效加速度数据点不足，无法辨识 I_a\n');
        fprintf('  建议：增加加速度轨迹的幅值或频率范围\n');
        Result.I_a = NaN;
        Result.I_a_R2 = NaN;
        Result.I_a_RMSE = NaN;
    end
else
    fprintf('跳过加速度试验（do_accel_test = false）\n');
    fprintf('提示：若需辨识 I_a，请设置 do_accel_test = true 并运行加速度轨迹\n');
    Result.I_a = NaN;
    Result.I_a_R2 = NaN;
    Result.I_a_RMSE = NaN;
end

%% 8. 输出结果（对应文档 6. 输出）
fprintf('\n===== 关节摩擦辨识结果 =====\n');
fprintf('  库伦摩擦 τ_c = %.4f N·m\n', Result.tau_c);
fprintf('  粘滞摩擦系数 b = %.4f N·m·s/rad\n', Result.b);
if ~isnan(Result.tau_s) && Result.tau_s > 0
    fprintf('  静摩擦 τ_s = %.4f N·m\n', Result.tau_s);
end
fprintf('  拟合 RMSE = %.4f N·m\n', Result.RMSE);
fprintf('  拟合 R²   = %.4f\n', Result.R2);
fprintf('  拟合用样本数 = %d\n', Result.nUsed);
if isfield(Result, 'I_a') && ~isnan(Result.I_a)
    fprintf('  电机转子惯量 I_a = %.6f kg·m² (R²=%.4f)\n', Result.I_a, Result.I_a_R2);
end
fprintf('============================\n');

% 与仿真真值对比（仅仿真示例时有意义）
fprintf('\n（仿真真值对比）\n');
fprintf('  τ_c: 真实=%.4f, 辨识=%.4f, 误差=%.4f%%\n', ...
    tau_c_true, Result.tau_c, abs(Result.tau_c - tau_c_true)/tau_c_true*100);
fprintf('  b:   真实=%.4f, 辨识=%.4f, 误差=%.4f%%\n', ...
    b_true, Result.b, abs(Result.b - b_true)/b_true*100);
if ~isnan(Result.tau_s) && Result.tau_s > 0
    fprintf('  τ_s: 真实=%.4f, 辨识=%.4f, 误差=%.4f%%\n', ...
        tau_s_true, Result.tau_s, abs(Result.tau_s - tau_s_true)/tau_s_true*100);
end

%% 9. 数据可视化（参考参考代码）
figure('Name', '速度-电流关系');
plot(qd_stable, i_stable, 'b.', 'MarkerSize', 4);
hold on;
grid on;
xlabel('角速度 qd (rad/s)');
ylabel('电流 i (A)');
title('速度-电流关系（稳定段数据）');

% 绘制拟合曲线
qd_plot = linspace(min(qd_stable), max(qd_stable), 200);
tau_f_plot = Result.tau_c * sign(qd_plot) + Result.b * qd_plot;
i_fit_plot = tau_f_plot / K_tau;
plot(qd_plot, i_fit_plot, 'r-', 'LineWidth', 2);
legend('实测数据', '拟合曲线', 'Location', 'best');
