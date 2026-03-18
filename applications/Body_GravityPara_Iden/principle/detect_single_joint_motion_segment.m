function [range_joint, start_idx, end_idx] = ...
    detect_single_joint_motion_segment(qd, joint_idx, joint_name, ...
                                       N_total, N_per_joint, opts)
%DETECT_SINGLE_JOINT_MOTION_SEGMENT
% 仅针对某个关节 qd(:,joint_idx)，在整条数据上检测其“往返运动段” range_joint。
%
% 使用与主脚本一致的逻辑：
%   - use_motion_interval / use_ordered_segments
%   - 强运动种子 + 平滑 + max_gap_merge + min_motion_len
%   - 多段时选“最早出现的一段”（对应生成轨迹的第一档速度）

use_motion_interval  = opts.use_motion_interval;
use_ordered_segments = opts.use_ordered_segments;

% 默认等分段索引（用于回退或 use_motion_interval=false）
default_start = (joint_idx - 1) * N_per_joint + 1;
if joint_idx == 12
    default_end = N_total;
else
    default_end = joint_idx * N_per_joint;
end

if ~use_motion_interval
    start_idx = default_start;
    end_idx   = default_end;
    range_joint = start_idx:end_idx;
    fprintf('  [%s] use_motion_interval=0, 等分段: 行 %d~%d (%d 点)\n', ...
        joint_name, start_idx, end_idx, numel(range_joint));
    return;
end

%% 1) 先确定检测块 block_start:block_end
if use_ordered_segments
    nominal_start = default_start;
    nominal_end   = default_end;

    block_start = max(1, nominal_start - opts.overlap_points);
    block_end   = min(N_total, nominal_end + opts.block_end_extend);
else
    block_start = 1;
    block_end   = N_total;
end

range_block = block_start:block_end;
qd_block    = qd(range_block, joint_idx);
N_block     = numel(range_block);

fprintf('  [%s] 检测块: 全局行 %d~%d (%d 点)\n', ...
    joint_name, block_start, block_end, N_block);

%% 2) 运动判定：强种子 + 平滑 + 阈值
seed = abs(qd_block) > opts.seed_threshold;
moving = false(N_block, 1);

if any(seed)
    seed_start = find(diff([0; seed(:)]) == 1);
    seed_end   = find(diff([seed(:); 0]) == -1);

    ext = opts.max_gap_merge;
    cand = false(N_block, 1);
    for kk = 1:numel(seed_start)
        cs = max(1, seed_start(kk) - ext);
        ce = min(N_block, seed_end(kk) + ext);
        cand(cs:ce) = true;
    end

    if opts.motion_window > 0
        w   = opts.motion_window;
        ker = ones(2*w+1,1)/(2*w+1);
        avg_abs_qd = conv(abs(qd_block), ker, 'same');
        moving = cand & (avg_abs_qd > opts.vel_threshold);
    else
        moving = cand & (abs(qd_block) > opts.vel_threshold);
    end
else
    if opts.motion_window > 0
        w   = opts.motion_window;
        ker = ones(2*w+1,1)/(2*w+1);
        avg_abs_qd = conv(abs(qd_block), ker, 'same');
        moving = avg_abs_qd > opts.vel_threshold;
    else
        moving = abs(qd_block) > opts.vel_threshold;
    end
end

run_start = find(diff([0; moving]) == 1);
run_end   = find(diff([moving; 0]) == -1);

% 合并间隔较小的区间
if ~isempty(run_start) && numel(run_start) > 1
    new_start = [];
    new_end   = [];
    s = run_start(1);
    e = run_end(1);
    for k = 2:numel(run_start)
        gap = run_start(k) - run_end(k-1) - 1;
        if gap <= opts.max_gap_merge
            e = run_end(k);
        else
            new_start(end+1,1) = s;
            new_end(end+1,1)   = e;
            s = run_start(k);
            e = run_end(k);
        end
    end
    new_start(end+1,1) = s;
    new_end(end+1,1)   = e;
    run_start = new_start;
    run_end   = new_end;
end

run_len = run_end - run_start + 1;
valid   = run_len >= opts.min_motion_len;
run_start = run_start(valid);
run_end   = run_end(valid);
run_len   = run_len(valid);

% 打印候选运动段
if ~isempty(run_start)
    base = block_start;
    for kk = 1:numel(run_start)
        seg_s_local = run_start(kk);
        seg_e_local = run_end(kk);
        seg_n       = run_len(kk);
        seg_s_global = base + seg_s_local - 1;
        seg_e_global = base + seg_e_local - 1;
        fprintf('  [%s] 候选运动段 %2d: 块内 %d~%d (%d 点), 全局行 %d~%d\n', ...
            joint_name, kk, seg_s_local, seg_e_local, seg_n, seg_s_global, seg_e_global);
    end
end

if isempty(run_start)
    warning('[%s] 未检测到足够长的运动区间（|qd|>%.3f 且连续>=%d 点），回退到等分段', ...
        joint_name, opts.vel_threshold, opts.min_motion_len);
    start_idx = default_start;
    end_idx   = default_end;
    range_joint = start_idx:end_idx;
    return;
end

% 选择“最早出现的一段”作为主往返段（对应生成轨迹中的第一档速度）
idx_first = 1;
seg_s_local = run_start(idx_first);
seg_e_local = run_end(idx_first);

start_idx = block_start + seg_s_local - 1;
end_idx   = block_start + seg_e_local - 1;
range_joint = start_idx:end_idx;

fprintf('  [%s] 选择最早运动段: 行 %d~%d (%d 点)\n', ...
    joint_name, start_idx, end_idx, numel(range_joint));

end

