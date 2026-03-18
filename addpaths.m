% addpaths.m  E1 仓库：添加 plan、dynamics、robot_model、utility_function、applications 到路径
% 使用：在 MATLAB 中 cd 到 model_e1 根目录，运行 addpaths

addpath(genpath('plan/'));
addpath(genpath('dynamics/'));
addpath(genpath('robot_model/'));
addpath(genpath('utility_function/'));
% 应用目录按需添加，或直接运行 applications 下脚本时其会 addpath 自身
addpath(genpath('applications/'));
