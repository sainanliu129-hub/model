function run_compare_single_joint_torque(csv_file)
% run_compare_single_joint_torque  单关节往返轨迹对齐总入口
%
% 用法：
%   run_compare_single_joint_torque()                 % 自动查找 CSV
%   run_compare_single_joint_torque('path/to.csv')    % 指定 CSV
%
% 说明：
%   目前仍然直接调用 compare_single_joint_torque_aligned，
%   后续可逐步把“数据对齐 / 动力学对比 / 绘图”拆成独立函数，
%   由本入口统一调度。

if nargin < 1
    csv_file = '';
end

if isempty(csv_file)
    compare_single_joint_torque_aligned();
else
    compare_single_joint_torque_aligned('csv_file', csv_file);
end

end

