function dx = forward_diff(x, t)
% forward_diff  前向差分求导，与 Gazebo 半隐式欧拉一致
%
% dx(k) = (x(k+1)-x(k))/dt，末点沿用前一点。用于与仿真一致时。
%
% 输入：x, t 同长向量
% 输出：dx 与 x 同长

x = x(:);
t = t(:);
n = length(x);
dx = zeros(n, 1);
if n < 2
    return;
end
dt_vec = diff(t);
good = isfinite(dt_vec) & dt_vec > 0;
if any(good)
    dt_vec(~good) = median(dt_vec(good));
else
    dt_vec(:) = 0.002;
end
dx(1:n-1) = diff(x) ./ (dt_vec(:) + eps);
dx(n) = dx(n-1);
end
