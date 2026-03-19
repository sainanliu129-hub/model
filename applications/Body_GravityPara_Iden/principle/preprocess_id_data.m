function [t_out, q_s, qd_s, qdd_s, tau_id, aux] = preprocess_id_data(t, q, qd_raw, tau_raw, opts)
% preprocess_id_data  统一低通预处理入口（仅低通链路）
%
% 当前策略已收敛为：
%   q  : 低通（Butterworth + filtfilt）
%   qd : 由低通后的 q 做中心差分
%   qdd: 由 qd 再中心差分
%   tau: 低通（Butterworth + filtfilt）
%
% 为兼容旧调用，函数签名保持不变，内部转调 preprocess_lowpass_id_data。
% 旧的 SG 参数（sg_order/sg_frame/derivative_method='sg'）已不再使用。

if nargin < 5
    opts = struct();
end

if isfield(opts, 'derivative_method')
    m = opts.derivative_method;
    if isstring(m), m = char(m); end
    if ischar(m) && strcmpi(m, 'sg')
        warning('preprocess_id_data: 已移除 SG 链路，当前固定使用 lowpass_diff。');
    end
end

[t_out, q_s, qd_s, qdd_s, tau_id, aux] = preprocess_lowpass_id_data(t, q, qd_raw, tau_raw, opts);
end

