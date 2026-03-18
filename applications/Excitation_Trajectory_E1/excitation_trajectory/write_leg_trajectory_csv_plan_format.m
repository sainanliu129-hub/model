function write_leg_trajectory_csv_plan_format(csv_path, t, Q12)
% write_leg_trajectory_csv_plan_format  按 plan/run_generate_and_plot 的 CSV 格式写入双腿轨迹
%
% 格式与 generate_multi_speed_trajectory、read_leg_trajectory_from_csv 一致：
%   表头：time, leg_l1_joint, leg_l2_joint, ..., leg_r6_joint
%   数据：每行 time(%.5f) + 12 个关节角 (%.6f)
% 单腿激励时由调用方构建轨迹：起止 0 位、过渡段、激励段内另一腿为安全位（roll），见 build_excitation_leg_trajectory_with_ramps。
%
% 输入:
%   csv_path - 输出 CSV 路径
%   t        - 时间列 (N×1 或 1×N)，单位 s
%   Q12      - 关节位置 (N×12)，列顺序 [左腿 1..6, 右腿 1..6] = leg_l1..leg_l6, leg_r1..leg_r6

t = t(:);
N = numel(t);
if size(Q12, 1) ~= N || size(Q12, 2) ~= 12
    error('write_leg_trajectory_csv_plan_format: Q12 须为 %d×12（与 t 长度一致）', N);
end

header_names = {'time', 'leg_l1_joint', 'leg_l2_joint', 'leg_l3_joint', 'leg_l4_joint', 'leg_l5_joint', 'leg_l6_joint', ...
    'leg_r1_joint', 'leg_r2_joint', 'leg_r3_joint', 'leg_r4_joint', 'leg_r5_joint', 'leg_r6_joint'};

fid = fopen(csv_path, 'w');
if fid == -1
    error('无法创建文件: %s', csv_path);
end
fprintf(fid, '%s', header_names{1});
for c = 2:numel(header_names)
    fprintf(fid, ',%s', header_names{c});
end
fprintf(fid, '\n');

for i = 1:N
    fprintf(fid, '%.5f', t(i));
    for j = 1:12
        fprintf(fid, ',%.6f', Q12(i, j));
    end
    fprintf(fid, '\n');
end
fclose(fid);
end
