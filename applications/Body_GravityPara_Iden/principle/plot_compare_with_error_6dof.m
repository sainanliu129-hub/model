function figs = plot_compare_with_error_6dof(x, ref_data, ref_name, pred_data, pred_names, quantity, title_prefix)
% plot_compare_with_error_6dof  统一绘图：对比图 + 误差图（6关节）
%
% 输入：
%   x          - Mx1 横轴
%   ref_data   - Mx6 参考数据
%   ref_name   - 参考名称（legend）
%   pred_data  - Mx(6*K) 预测拼接矩阵
%   pred_names - 1xK 名称 cell
%   quantity   - 'torque' 或 'qdd' 等（传给 plot_compare_6dof）
%   title_prefix - 标题前缀

if nargin < 7, title_prefix = '对比'; end
if nargin < 6 || isempty(quantity), quantity = 'value'; end

M = size(ref_data, 1);
if size(ref_data, 2) ~= 6
    error('plot_compare_with_error_6dof: ref_data 必须是 Mx6。');
end
if mod(size(pred_data, 2), 6) ~= 0
    error('plot_compare_with_error_6dof: pred_data 列数必须是 6 的倍数。');
end
K = size(pred_data, 2) / 6;
if numel(pred_names) ~= K
    error('plot_compare_with_error_6dof: pred_names 数量与 pred_data 不匹配。');
end

all_data = [ref_data, pred_data];
legend_names = [{ref_name}, pred_names(:)'];

figs = struct();
figs.compare = figure('Name', [title_prefix '_对比']);
plot_compare_6dof(x(:), all_data, quantity, legend_names);
sgtitle([title_prefix ' 对比']);

% 误差图：每个预测都减参考
err_data = zeros(M, 6 * K);
err_names = cell(1, K);
for i = 1:K
    cols = (i-1)*6 + (1:6);
    err_data(:, cols) = pred_data(:, cols) - ref_data;
    err_names{i} = [pred_names{i} ' - ' ref_name];
end
figs.error = figure('Name', [title_prefix '_误差']);
plot_compare_6dof(x(:), err_data, quantity, err_names);
sgtitle([title_prefix ' 误差']);
end

