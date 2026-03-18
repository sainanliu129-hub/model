% example_identify_inertia  惯量辨识示例脚本
%
% 本脚本演示如何使用 identify_inertia_from_data 从实测数据辨识电机转子惯量 I_a
%
% 使用流程：
%   1. 先运行摩擦力辨识得到 tau_c、b（如 example_identify_friction 或 identify_friction_from_trajectory）
%   2. 使用加速度轨迹在真机上采集数据（时间、位置、速度、力矩等）
%   3. 保存实测数据为 CSV 文件
%   4. 运行本脚本进行惯量辨识
%
% 数据文件格式（CSV，与摩擦力辨识一致）：
%   3 列格式：时间 (s), 力矩 (N·m), 速度 (rpm 或 rad/s，见 VelocityUnit)
%   5+ 列格式：时间, cmd_pos, joint_pos, joint_vel, joint_torque, ...
%   其中 joint_torque 可为电流（需 tau_is_current=true）或力矩

clear;
clc;
close all;

%% 参数设置
dd = get_data_dir('friction_iden');   % 与 run_full_iden_three_files 一致
data_file = fullfile(dd, 'multi_joint_track_20260210-135938_acc.csv');
K_tau = 2.81;                          % 力矩系数 N·m/A（若列为电流需用于转换）

% 已辨识的摩擦参数（需先运行摩擦力辨识；可不对称：正向 τ_c_pos、负向 τ_c_neg）
tau_c_pos = 2.2;                      % 正向库伦 N·m
tau_c_neg = -1.4;                     % 负向库伦 N·m
tau_c = (tau_c_pos - tau_c_neg) / 2;  % 对称等效，供仅用 tau_c 的调用
b = 0.02;                             % 粘滞摩擦系数 N·m·s/rad

%% 三列格式（时间, 力矩, 速度）— 一点运行即自动执行
% 使用上面参数，对 data_file 做惯量辨识（3 列时自动识别，速度单位 rpm，不对称库伦）
fprintf('========== 三列格式惯量辨识（自动运行）==========\n');
if exist(data_file, 'file')
    Result = identify_inertia_from_data(data_file, K_tau, tau_c, b, ...
        'VelocityUnit', 'rpm', ...
        'TauCPos', tau_c_pos, 'TauCNeg', tau_c_neg, ...
        'tau_is_current', false, ...
        'DoPlot', true);
    fprintf('I_a = %.6f kg·m², R² = %.4f\n', Result.I_a, Result.I_a_R2);
else
    fprintf('数据文件不存在: %s\n', data_file);
    fprintf('请将三列数据（时间, 力矩, 速度）放到该路径，或修改 data_file。\n');
    Result = [];
end

% %% 示例2：自定义参数进行辨识
% fprintf('\n========== 示例2：自定义参数进行辨识 ==========\n');
% if exist(data_file, 'file')
%     Result2 = identify_inertia_from_data(data_file, K_tau, tau_c, b, ...
%         'TauCPos', tau_c_pos, 'TauCNeg', tau_c_neg, ...
%         'tau_is_current', false, ...
%         'filter_window', 5, ...
%         'accel_threshold', 0.01, ...
%         'VelocityUnit', 'rpm', ...
%         'DoPlot', true);
%     fprintf('I_a = %.6f kg·m², R² = %.4f\n', Result2.I_a, Result2.I_a_R2);
% else
%     fprintf('数据文件不存在: %s\n', data_file);
% end
% 
% %% 示例3：批量处理多个数据文件
% fprintf('\n========== 示例3：批量处理多个数据文件 ==========\n');
% data_files = {fullfile(dd, 'accel_data1.csv'), ...
%               fullfile(dd, 'accel_data2.csv'), ...
%               fullfile(dd, 'accel_data3.csv')};
% 
% Results = [];
% for i = 1:length(data_files)
%     if exist(data_files{i}, 'file')
%         fprintf('\n处理文件 %d/%d: %s\n', i, length(data_files), data_files{i});
%         Result_i = identify_inertia_from_data(data_files{i}, K_tau, tau_c, b, ...
%             'tau_is_current', false, ...
%             'DoPlot', false);  % 批量处理时不绘图
%         Results = [Results; Result_i];
%     else
%         fprintf('文件不存在: %s\n', data_files{i});
%     end
% end
% 
% % 计算平均结果
% if ~isempty(Results)
%     fprintf('\n批量处理结果统计:\n');
%     fprintf('  平均惯量 I_a = %.6f kg·m²\n', mean([Results.I_a]));
%     fprintf('  平均拟合 R² = %.4f\n', mean([Results.I_a_R2]));
% end
% 
% fprintf('\n========== 完成 ==========\n');
% fprintf('提示：\n');
% fprintf('  1. 确保数据文件格式正确（3 列或 5+ 列），速度与力矩正负一致\n');
% fprintf('  2. 先完成摩擦力辨识得到 tau_c、b，再辨识惯量\n');
% fprintf('  3. 若 joint_torque 列为电流，设置 tau_is_current=true 并填对 K_tau\n');
