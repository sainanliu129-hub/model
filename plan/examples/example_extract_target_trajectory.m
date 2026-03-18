% example_extract_target_trajectory  从录制 CSV 拆取期望轨迹示例
%
% 输入CSV需包含表头：time, *_target_pos, *_real_pos, ...
% 输出格式与生成轨迹一致（time, leg_l1_joint, ..., leg_r6_joint），起止为0位。
%
% 用法：先执行项目根目录 addpaths，修改 input_csv 为实际路径后运行。

clear; clc;

input_csv = fullfile(get_data_dir(), 'walkrun_m1_seg000_20260203_105915.csv');
if ~exist(input_csv, 'file'), input_csv = 'walkrun_m1_seg000_20260203_105915.csv'; end

output_csv = fullfile(get_build_dir('plan'), 'walkrun_target_trajectory.csv');
Ts = 0.002;
transition_time = 1.0;

if exist(input_csv, 'file')
    extract_target_trajectory_from_csv(input_csv, ...
        'output_file', output_csv, ...
        'Ts', Ts, ...
        'transition_time', transition_time);
    fprintf('已保存: %s\n', output_csv);
else
    fprintf('请将 input_csv 改为实际录制文件路径后重新运行\n');
    fprintf('示例: extract_target_trajectory_from_csv(''recorded.csv'', ''output_file'', ''out.csv'');\n');
end
