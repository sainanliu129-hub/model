% run_read_leg_trajectory_example  示例：从 CSV 读取腿部轨迹并插值到 500Hz，输出到本模块 build/
%
% 用法：将 data/ 下任意 30Hz 腿部 CSV 文件名赋给 input_csv，或使用默认示例后运行。
% 输出：build/ 下生成 *_leg_500Hz.csv 与 *_leg_500Hz.mat

clear; clc;

% 确保路径包含 plan、utility_function（若在仓库根运行 addpaths 则已包含）
base = fileparts(mfilename('fullpath'));
repo_root = fileparts(fileparts(base));
addpath(genpath(fullfile(repo_root, 'plan')));
addpath(genpath(fullfile(repo_root, 'utility_function')));

% 输入：本模块 data 目录下 CSV（无时间列则默认 30Hz）
input_csv = fullfile(base, 'data', '0-260116_run_softly_轻跑步_002.csv');
if ~exist(input_csv, 'file')
    % 若没有示例数据，尝试仓库根 data 或 build/plan
    input_csv = fullfile(repo_root, 'build', 'plan', '0-260116_run_softly_轻跑步_002_leg_500Hz.csv');
    if ~exist(input_csv, 'file')
        error('请在本模块 data/ 下放入 30Hz 腿部 CSV，或修改 input_csv 路径。');
    end
end

% 输出到本模块 build/
out_dir = fullfile(base, 'build');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
[~, name, ~] = fileparts(input_csv);
output_csv = fullfile(out_dir, [name '_leg_500Hz.csv']);
output_mat = fullfile(out_dir, [name '_leg_500Hz.mat']);

read_leg_trajectory_from_csv(input_csv, ...
    'output_file', output_csv, ...
    'fs_in', 30, ...
    'fs_out', 500, ...
    'save_mat', output_mat);

fprintf('已生成: %s\n', output_csv);
fprintf('已生成: %s\n', output_mat);
