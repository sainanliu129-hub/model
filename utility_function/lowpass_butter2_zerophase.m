function y = lowpass_butter2_zerophase(x, fc_Hz, fs_Hz)
% lowpass_butter2_zerophase  二阶 Butterworth 低通、零相位（filtfilt）
%
% 用于 q/qd/tau 等信号滤波，避免相位偏移影响辨识与对比。
%
% 输入：x 信号，fc_Hz 截止频率 (Hz)，fs_Hz 采样率 (Hz)
% 输出：与 x 同形的滤波结果。若 fc_Hz<=0 或 fc_Hz>=fs_Hz/2 则返回 x 不变。

if fc_Hz <= 0 || fc_Hz >= fs_Hz/2
    y = x;
    return;
end
[b, a] = butter(2, fc_Hz / (fs_Hz/2), 'low');
y = filtfilt(b, a, x(:));
y = reshape(y, size(x));
end
