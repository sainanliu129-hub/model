% example_friction_torque_from_traj  用 PD-M1-v0_multi_joint_*_traj_1.csv 第 4 列速度计算摩擦力矩
%
% 第 4 列为 vel_leg_l1_joint（rad/s），按摩擦模型 τ_f = τ_c + b×qd 计算实摩擦力矩。
% 请根据实际辨识结果修改 tau_c_pos、tau_c_neg、b。

clear; clc; close all;

%% 1. 轨迹 CSV 与摩擦参数
% 数据目录：优先 data/dynamics/single_pos，否则当前目录
cur = fileparts(mfilename('fullpath'));
proj = fullfile(cur, '..', '..');
csv_name = 'PD-M1-v0_multi_joint_20260206-213622_traj_1.csv';
csv_candidates = {
    fullfile(proj, 'data', 'dynamics', 'single_pos', csv_name);
    fullfile(cur, csv_name);
    csv_name;
};
csv_file = '';
for k = 1:length(csv_candidates)
    if exist(csv_candidates{k}, 'file')
        csv_file = csv_candidates{k};
        break;
    end
end
if isempty(csv_file)
    error('未找到文件 %s，请指定 csv_file 路径', csv_name);
end
fprintf('轨迹文件: %s\n', csv_file);

% 摩擦参数（请改为该关节辨识结果：τ_c_pos, τ_c_neg, b）
tau_c_pos = 1.8;   % N·m
tau_c_neg = -1.7;  % N·m
b         = 0.05;  % N·m·s/rad

%% 2. 读取第 4 列速度并计算摩擦力矩
[time, qd, tau_f] = compute_friction_torque_from_traj_csv(...
    csv_file, tau_c_pos, tau_c_neg, b, 'VelocityColumn', 16, 'VelocityUnit', 'rad/s');

fprintf('点数: %d, 时长: %.2f s\n', length(time), time(end));
fprintf('速度范围: %.4f ~ %.4f rad/s\n', min(qd), max(qd));
fprintf('摩擦力矩范围: %.4f ~ %.4f N·m\n', min(tau_f), max(tau_f));

%% 3. 绘图
figure('Name', '轨迹速度与摩擦力矩');
subplot(2,1,1);
plot(time, qd, 'b-', 'LineWidth', 0.8);
grid on; xlabel('时间 (s)'); ylabel('角速度 qd (rad/s)');
title('第 4 列速度 vel\_leg\_l1\_joint');
subplot(2,1,2);
plot(time, tau_f, 'r-', 'LineWidth', 0.8);
grid on; xlabel('时间 (s)'); ylabel('摩擦力矩 \tau_f (N\cdotm)');
title(sprintf('模型摩擦力矩 (\\tau_{c+}=%.2f, \\tau_{c-}=%.2f, b=%.4f)', tau_c_pos, tau_c_neg, b));


j = 4;   % 关节4

tau_sim   = data_sim_1.torque_leg_l(:,j);
tau_true  = data_true_1.torque_leg_l(:,j);
tau_comp  = data_sim_1.torque_leg_l(:,j) + data_sim_2(:,j);

figure;
plot(tau_sim,'b','LineWidth',1.5); hold on;
plot(tau_true,'k','LineWidth',1.5);
plot(tau_comp,'r','LineWidth',1.5);
grid on;

legend('sim','true','sim+comp');
title(sprintf('Joint %d Torque Comparison',j));
xlabel('Sample');
ylabel('Torque (Nm)');