function y = central_diff_point(x, t, k, dt)
% central_diff_point  单点中心差分：第 k 点的导数
%
% 当只需某一点导数时可避免算整段 central_diff。与 central_diff 首尾规则一致。
%
% 输入：x 向量，t 时间，k 下标 (1..n)，dt 可选（缺省用 median(diff(t))）
% 输出：标量，x 在 k 处的导数

n = numel(x);
if nargin < 4 || isempty(dt)
    dt = median(diff(t));
end
if k == 1
    y = (x(2) - x(1)) / dt;
elseif k == n
    y = (x(n) - x(n-1)) / dt;
else
    y = (x(k+1) - x(k-1)) / (2*dt);
end
end
