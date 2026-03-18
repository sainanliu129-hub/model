function g = get_e1_gravity()
% get_e1_gravity  E1 机器人重力加速度向量（世界系，单位 m/s^2）
%
% 用法:
%   g = get_e1_gravity()
%
% 输出:
%   g - 3×1，世界系下重力加速度，默认 [0; 0; -9.81]（z 向下）
%       修改本文件可改变整机/肢体/ReMatrix 使用的重力方向（如侧放 [0; 9.81; 0]）
%
% 被 get_e1_full_robot、get_e1_limb_robot 及 ReMatrix 的 robot_limb.Gravity 使用。
%
% 常用方向示例（世界系）:
%   竖直向下（默认）: g = [0; 0; -9.81]
%   沿 +y（侧放）:   g = [0; 9.81; 0]
%   沿 -x:           g = [-9.81; 0; 0]

% 三向均非零：方向 [1;1;-1]，模长 9.81 m/s^2
% g = 9.81 * [1; 1; -1] / norm([1; 1; -1]);
g = [0; 0; -9.81];
