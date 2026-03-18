function dx = central_diff(x, t)
% central_diff  中心差分求导，保持与 x 同长（首尾用前向/后向差分）
%
% 用于从位置/速度序列得到速度/加速度，与 Body_GravityPara_Iden 等脚本一致。
%
% 输入：x 为 n×1 或 1×n，t 为时间向量（与 x 同长）
% 输出：dx 与 x 同形状，单位与 x 一致、时间单位由 t 决定

x = x(:);
t = t(:);
n = length(x);
dx = zeros(n, 1);
if n < 2
    return;
end
dt_fwd = t(2:n) - t(1:n-1);
dt_central = t(3:n) - t(1:n-2);
dx(1) = (x(2) - x(1)) / (dt_fwd(1) + eps);
if n > 2
    dx(2:n-1) = (x(3:n) - x(1:n-2)) ./ (dt_central + eps);
end
dx(n) = (x(n) - x(n-1)) / (dt_fwd(end) + eps);
end
