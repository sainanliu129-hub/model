% run_generate_urdf_validation_trajectory  生成 URDF 验证轨迹（Part A/B）
%
% 摩擦力辨识等应用需使用与 plan 一致的轨迹生成。本脚本确保路径后调用
% plan 示例脚本，生成文件默认在 build/plan/。
%
% 用法：在项目根目录执行 addpaths 后，可直接运行本脚本，或运行：
%   run( fullfile( fileparts(which('generate_urdf_validation_trajectory')), 'examples', 'example_generate_urdf_validation_trajectory.m' ) );

if isempty(which('generate_urdf_validation_trajectory'))
    addpath(genpath(fullfile(fileparts(mfilename('fullpath')), '..', '..', 'plan')));
    addpath(genpath(fullfile(fileparts(mfilename('fullpath')), '..', '..', 'utility_function')));
end
run(fullfile(fileparts(which('generate_urdf_validation_trajectory')), 'examples', 'example_generate_urdf_validation_trajectory.m'));
