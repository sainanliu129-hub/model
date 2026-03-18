% export_torque_from_leg_csv  从多关节 CSV 中提取双腿 12 关节力矩，保存为新的 CSV
%
% 用法（建议在本目录或工程根目录下运行）：
%   run('applications/Body_GravityPara_Iden/export_torque_from_leg_csv.m')
%
% 按需修改下面的 csv_in 即可。

clear; clc; close all;

% 脚本所在目录（与当前工作目录无关）
app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
ensure_body_gravity_para_iden_path();

% 确保依赖路径（utility_function 等）已加入
ensure_body_gravity_para_iden_path();

% ==== 输入 CSV：原始多关节数据 ====
% 可根据需要修改为其它文件名
csv_in = fullfile(app_root, 'data', '跑步', 'PD-M1-v0_multi_joint_20260227-071606_sim.csv');
if ~exist(csv_in, 'file')
    error('输入 CSV 不存在: %s', csv_in);
end

fprintf('读取源文件: %s\n', csv_in);
data = read_leg_joint_csv(csv_in);

% 双腿 12 关节力矩：左 6 + 右 6
tau = [data.cmd_torque_leg_l, data.cmd_torque_leg_l];   % n×12

% 列名：time + tau_leg_l1_joint ... tau_leg_r6_joint
[~, ~, urdf_names] = get_e1_leg_joint_names();  % leg_l1_joint ... leg_r6_joint
varNames = [{'time'}, strcat('tau_', urdf_names')];

T = array2table([data.time, tau], 'VariableNames', varNames);

% 输出文件：在原文件名后加 _torque
[p, name, ext] = fileparts(csv_in);
csv_out = fullfile(p, [name, '_torque', ext]);

writetable(T, csv_out);
fprintf('已将 12 关节力矩导出到: %s\n', csv_out);

