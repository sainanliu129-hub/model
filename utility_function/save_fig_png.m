function save_fig_png(opts, default_name)
% save_fig_png  将当前 figure 保存为 PNG（当 opts.save_png 为 true 时）
%
% 供 Body_GravityPara_Iden 下 compare_torque_sim_vs_id、compare_forward_dynamics_vs_measured 等使用。
%
% 输入：
%   opts         - 含 .save_png (bool)、.save_png_dir (可选，默认 pwd)
%   default_name - 当 figure 无 Name 时使用的文件名（不含扩展名）

if ~isfield(opts, 'save_png') || ~opts.save_png
    return;
end
dir_out = pwd;
if isfield(opts, 'save_png_dir') && ~isempty(opts.save_png_dir)
    dir_out = opts.save_png_dir;
end
if ~exist(dir_out, 'dir')
    mkdir(dir_out);
end
name = get(gcf, 'Name');
if isempty(name), name = default_name; end
name = strrep(name, ' ', '_');
name = strrep(name, ':', '_');
name = strrep(name, '（', '_');
name = strrep(name, '）', '_');
name = strrep(name, '/', '_');
name = strrep(name, '\', '_');
name = strrep(name, '·', '_');
name = regexprep(name, '_+', '_');
name = regexprep(name, '^_+|_+$', '');
name = strtrim(name);
if isempty(name), name = default_name; end
fpath = fullfile(dir_out, [name '.png']);
try
    print(gcf, fpath, '-dpng', '-r150');
    fprintf('已保存: %s\n', fpath);
catch
    warning('保存 PNG 失败: %s', fpath);
end
end
