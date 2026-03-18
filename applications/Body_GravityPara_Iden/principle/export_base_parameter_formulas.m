function [S, lines_text, lines_latex] = export_base_parameter_formulas(Y_full, index_base, para_order, opts)
% export_base_parameter_formulas  从 Y 与 index_base 自动生成「最小参数 = 全参线性组合」的显式公式
%
% 原理：p_m = S*θ，其中 S = Y_min \ Y_full（使 Y_min*S ≈ Y_full），
% 故第 i 个最小参数 p_m(i) = sum_j S(i,j)*θ(j)，系数 S 由 regressor 与选列唯一确定。
% 与具体机械臂几何/URDF 一致（无需手推 DH）。
%
% 输入：
%   Y_full     - (M×60) 全参观测矩阵（已堆叠多时刻）
%   index_base - 最小参数列索引（如 QR 选列得到）
%   para_order - 1 或 2，与 get_limb_theta_from_URDF / ReMatrix 一致
%         1: θ 每连杆 [m mx my mz Ixx Ixy Ixz Iyy Iyz Izz]
%         2: [Ixx Ixy Ixz Iyy Iyz Izz mx my mz m]
%   opts       - 可选，结构体或省略：
%         .coeff_threshold - 系数低于此的项不显示（默认 1e-10）
%         .format_num      - 系数格式，如 '%.6g'（默认）
%         .do_latex        - true 时生成 LaTeX 行（默认 false）
%         .n_links        - 连杆数（默认 6）
%
% 输出：
%   S          - (p_min×60) 参数变换矩阵，p_m = S*θ
%   lines_text - p_min×1 cell，每行为 "p_m(i) = ..." 的文本
%   lines_latex- p_min×1 cell，每行为 LaTeX（若 opts.do_latex）

if nargin < 4
    opts = struct();
end
if ~isfield(opts, 'coeff_threshold'), opts.coeff_threshold = 1e-10; end
if ~isfield(opts, 'format_num'),      opts.format_num = '%.6g'; end
if ~isfield(opts, 'do_latex'),        opts.do_latex = false; end
if ~isfield(opts, 'n_links'),         opts.n_links = 6; end

n_links = opts.n_links;
Y_min = Y_full(:, index_base);
% S: p_min×60，满足 Y_min*S ≈ Y_full => S = Y_min \ Y_full
S = Y_min \ Y_full;

p_min = size(S, 1);
param_names_1 = {'m', 'mx', 'my', 'mz', 'Ixx', 'Ixy', 'Ixz', 'Iyy', 'Iyz', 'Izz'};
param_names_2 = {'Ixx', 'Ixy', 'Ixz', 'Iyy', 'Iyz', 'Izz', 'mx', 'my', 'mz', 'm'};
if para_order == 1
    param_names = param_names_1;
else
    param_names = param_names_2;
end

lines_text = cell(p_min, 1);
lines_latex = cell(p_min, 1);
th = opts.coeff_threshold;
fmt = opts.format_num;

for i = 1:p_min
    terms = {};
    terms_latex = {};
    for j = 1:size(S, 2)
        c = S(i, j);
        if abs(c) < th
            continue;
        end
        link_id = ceil(j / 10);
        idx_in_link = mod(j - 1, 10) + 1;
        name = param_names{idx_in_link};
        sym_name = sprintf('%s_%d', name, link_id);
        % 文本：系数*符号，正数前加 +
        c_str = num2str(c, fmt);
        if c > 0 && ~isempty(terms)
            terms{end+1} = [' + ', c_str, '*', sym_name];
        else
            terms{end+1} = [c_str, '*', sym_name];
        end
        if opts.do_latex
            % LaTeX: 系数 \cdot 符号，惯量用 I_{xx,1} 等
            if startsWith(name, 'I')
                sym_latex = sprintf('%s_{%d}', name, link_id);
            else
                sym_latex = sprintf('%s_{%d}', name, link_id);
            end
            c_latex = num2str(c, fmt);
            if c > 0 && ~isempty(terms_latex)
                terms_latex{end+1} = [' + ', c_latex, ' ', sym_latex];
            else
                terms_latex{end+1} = [c_latex, ' ', sym_latex];
            end
        end
    end
    if isempty(terms)
        terms = {'0'};
        terms_latex = {'0'};
    end
    lines_text{i} = sprintf('p_m(%d) = %s', i, strcat(terms{:}));
    if opts.do_latex
        lines_latex{i} = sprintf('p_m(%d) &= %s \\\\', i, strcat(terms_latex{:}));
    else
        lines_latex{i} = '';
    end
end

end
