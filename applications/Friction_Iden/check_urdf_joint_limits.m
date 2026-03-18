% check_urdf_joint_limits  检查URDF文件中所有关节的限位定义
%
% 功能：读取E1.urdf文件，检查所有关节是否定义了限位（limit标签）
%
% 使用方法:
%   check_urdf_joint_limits()
%   check_urdf_joint_limits('urdf_file', 'path/to/E1.urdf')

function check_urdf_joint_limits(varargin)

p = inputParser;
addParameter(p, 'urdf_file', '', @ischar);
parse(p, varargin{:});
opts = p.Results;

% 默认URDF路径
if isempty(opts.urdf_file)
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', '..', '..', 'utility_function'));
    try
        opts.urdf_file = get_e1_urdf_path();
    catch
        error('未找到 E1.urdf，请从仓库根目录运行或指定 urdf_file 参数');
    end
end
if ~exist(opts.urdf_file, 'file')
    error('未找到URDF文件: %s', opts.urdf_file);
end

fprintf('========================================\n');
fprintf('检查URDF关节限位\n');
fprintf('========================================\n');
fprintf('URDF文件: %s\n\n', opts.urdf_file);

%% 读取URDF文件
try
    xmlDoc = xmlread(opts.urdf_file);
    jointNodes = xmlDoc.getElementsByTagName('joint');
    numJoints = jointNodes.getLength();
catch ME
    error('读取URDF文件失败: %s', ME.message);
end

fprintf('URDF中总关节数: %d\n\n', numJoints);

%% 检查所有关节的限位
joints_with_limit = {};
joints_without_limit = {};
joint_names_seen = containers.Map();  % 避免重复

fprintf('关节限位检查结果:\n');
fprintf('%-30s %-15s %-15s %-10s\n', '关节名', 'lower (rad)', 'upper (rad)', '类型');
fprintf('%s\n', repmat('-', 1, 70));

for i = 0:numJoints - 1
    jointNode = jointNodes.item(i);
    jointName = char(jointNode.getAttribute('name'));
    jointType = char(jointNode.getAttribute('type'));
    
    % 跳过已检查的关节（避免重复）
    if joint_names_seen.isKey(jointName)
        continue;
    end
    joint_names_seen(jointName) = true;
    
    limitElement = jointNode.getElementsByTagName('limit').item(0);
    
    if ~isempty(limitElement)
        lower_str = char(limitElement.getAttribute('lower'));
        upper_str = char(limitElement.getAttribute('upper'));
        lower = str2double(lower_str);
        upper = str2double(upper_str);
        
        if ~isnan(lower) && ~isnan(upper)
            fprintf('%-30s %-15.4f %-15.4f %-10s\n', jointName, lower, upper, jointType);
            joints_with_limit{end+1} = {jointName, lower, upper, jointType};
        else
            fprintf('%-30s %-15s %-15s %-10s (限位值无效)\n', ...
                jointName, lower_str, upper_str, jointType);
            joints_without_limit{end+1} = {jointName, '限位值无效'};
        end
    else
        fprintf('%-30s %-15s %-15s %-10s (无limit标签)\n', ...
            jointName, '-', '-', jointType);
        joints_without_limit{end+1} = {jointName, '无limit标签'};
    end
end

%% 统计信息
fprintf('\n%s\n', repmat('=', 1, 70));
fprintf('统计信息:\n');
fprintf('  有关节限位: %d 个\n', length(joints_with_limit));
fprintf('  无关节限位: %d 个\n', length(joints_without_limit));

if ~isempty(joints_without_limit)
    fprintf('\n无限位的关节:\n');
    for i = 1:length(joints_without_limit)
        fprintf('  - %s (%s)\n', joints_without_limit{i}{1}, joints_without_limit{i}{2});
    end
end

%% 检查腿部关节（用于轨迹生成）
fprintf('\n%s\n', repmat('=', 1, 70));
fprintf('腿部关节限位检查（用于轨迹生成）:\n');
leg_joints = {'leg_l1_joint', 'leg_l2_joint', 'leg_l3_joint', 'leg_l4_joint', 'leg_l5_joint', 'leg_l6_joint', ...
              'leg_r1_joint', 'leg_r2_joint', 'leg_r3_joint', 'leg_r4_joint', 'leg_r5_joint', 'leg_r6_joint'};

fprintf('%-20s %-15s %-15s %-15s\n', '关节名', 'lower (rad)', 'upper (rad)', '范围 (rad)');
fprintf('%s\n', repmat('-', 1, 65));

all_leg_joints_have_limit = true;
for i = 1:length(leg_joints)
    joint_name = leg_joints{i};
    found = false;
    
    for j = 0:numJoints - 1
        jointNode = jointNodes.item(j);
        jointNameAttr = char(jointNode.getAttribute('name'));
        
        if strcmp(jointNameAttr, joint_name)
            limitElement = jointNode.getElementsByTagName('limit').item(0);
            if ~isempty(limitElement)
                lower = str2double(limitElement.getAttribute('lower'));
                upper = str2double(limitElement.getAttribute('upper'));
                if ~isnan(lower) && ~isnan(upper)
                    range = upper - lower;
                    fprintf('%-20s %-15.4f %-15.4f %-15.4f\n', joint_name, lower, upper, range);
                    found = true;
                end
            end
            break;
        end
    end
    
    if ~found
        fprintf('%-20s %-15s %-15s %-15s (未找到或无限位)\n', joint_name, '-', '-', '-');
        all_leg_joints_have_limit = false;
    end
end

if all_leg_joints_have_limit
    fprintf('\n✓ 所有12个腿部关节都有有效的限位定义\n');
else
    fprintf('\n✗ 部分腿部关节缺少限位定义\n');
end

fprintf('\n========================================\n');

end
