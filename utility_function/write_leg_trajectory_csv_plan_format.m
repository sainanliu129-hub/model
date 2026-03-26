function write_leg_trajectory_csv_plan_format(csv_path, t, Q12)
% write_leg_trajectory_csv_plan_format  与激励/plan 生成 CSV 相同格式
%
% 表头：time, leg_l1_joint, ..., leg_r6_joint
% 数据：time 用 %.5f，关节角用 %.6f（与 applications/Excitation_Trajectory_E1/.../ 下同名函数一致）

t = t(:);
N = numel(t);
if size(Q12, 1) ~= N || size(Q12, 2) ~= 12
    error('write_leg_trajectory_csv_plan_format: Q12 须为 N×12 且 N 与 t 一致');
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
