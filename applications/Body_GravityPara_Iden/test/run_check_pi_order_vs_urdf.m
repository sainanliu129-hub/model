%% run_check_pi_order_vs_urdf  （已化简）转调 run_cad_quick_check
%
% 本脚本保留旧入口，内部统一调用 CAD-only 快速自检脚本 run_cad_quick_check，
% 以避免重复维护「π 顺序 + 列范数」代码。推荐直接使用：
%
%   run_cad_quick_check
%
% 若希望使用 CSV 轨迹，请修改 run_cad_quick_check 中的 trajectory_source / csv_file 配置。

clc; clear; close all;
fprintf('[run_check_pi_order_vs_urdf] 已化简为调用 run_cad_quick_check（推荐直接使用新脚本）。\n');

run_cad_quick_check;

