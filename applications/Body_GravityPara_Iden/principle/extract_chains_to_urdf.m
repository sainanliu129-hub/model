% extract_chains_to_urdf  从完整URDF提取4个chain并保存为独立的URDF文件
%
% 功能：
%   1. 加载完整的E1.urdf
%   2. 识别4个chain（左腿、右腿、左臂、右臂）
%   3. 使用subtree提取每个chain
%   4. 保存为独立的URDF文件
%
% 用法：
%   extract_chains_to_urdf()
%   extract_chains_to_urdf('urdf_path', 'path/to/E1.urdf', 'output_dir', 'path/to/output')
%
% 输出：
%   在output_dir目录下生成4个URDF文件：
%   - E1_left_leg.urdf
%   - E1_right_leg.urdf
%   - E1_left_arm.urdf
%   - E1_right_arm.urdf

function extract_chains_to_urdf(varargin)
p = inputParser;
addParameter(p, 'urdf_path', '', @ischar);
addParameter(p, 'output_dir', '', @ischar);
parse(p, varargin{:});
opts = p.Results;

%% 1. 确定URDF文件路径
if isempty(opts.urdf_path)
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', '..', '..', 'utility_function'));
    opts.urdf_path = get_e1_urdf_path();
end
if ~exist(opts.urdf_path, 'file')
    error('找不到URDF文件: %s', opts.urdf_path);
end

%% 2. 确定输出目录
if isempty(opts.output_dir)
    base_dir = fileparts(mfilename('fullpath'));
    opts.output_dir = fullfile(base_dir, '..', '..', '..', 'noetix_description', 'urdf', 'chains');
end

if ~exist(opts.output_dir, 'dir')
    mkdir(opts.output_dir);
end

fprintf('===== 提取chain并保存为独立URDF文件 =====\n');
fprintf('输入URDF: %s\n', opts.urdf_path);
fprintf('输出目录: %s\n', opts.output_dir);

%% 3. 加载完整机器人模型
fprintf('\n加载完整机器人模型...\n');
robot_full = importrobot(opts.urdf_path);
robot_full.DataFormat = 'row';

fprintf('总bodies数: %d\n', robot_full.NumBodies);

%% 4. 识别4个chain的末端link
fprintf('\n识别chain末端link...\n');

% 定义4个chain的末端link名称
chain_configs = {
    struct('name', 'left_leg', 'end_link', 'leg_l6_link', 'joint_prefix', 'leg_l');
    struct('name', 'right_leg', 'end_link', 'leg_r6_link', 'joint_prefix', 'leg_r');
    struct('name', 'left_arm', 'end_link', 'arm_l3_link', 'joint_prefix', 'arm_l');
    struct('name', 'right_arm', 'end_link', 'arm_r3_link', 'joint_prefix', 'arm_r');
};

% 验证link是否存在，如果不存在则尝试查找
for i = 1:length(chain_configs)
    config = chain_configs{i};
    link_exists = false;
    
    % 检查指定的link是否存在
    for j = 1:robot_full.NumBodies
        if strcmp(robot_full.Bodies{j}.Name, config.end_link)
            link_exists = true;
            break;
        end
    end
    
    % 如果不存在，尝试查找包含joint_prefix的最后一个link
    if ~link_exists
        fprintf('警告：未找到 %s，尝试自动查找...\n', config.end_link);
        for j = robot_full.NumBodies:-1:1
            body_name = robot_full.Bodies{j}.Name;
            if contains(body_name, config.joint_prefix) && ...
               (contains(body_name, '6') || contains(body_name, '3') || ...
                contains(body_name, 'ankle') || contains(body_name, 'foot') || ...
                contains(body_name, 'tip') || contains(body_name, 'end'))
                config.end_link = body_name;
                link_exists = true;
                fprintf('  找到替代link: %s\n', body_name);
                break;
            end
        end
    end
    
    if ~link_exists
        warning('无法找到 %s 的末端link，将跳过该chain', config.name);
        chain_configs{i}.end_link = '';
    else
        chain_configs{i}.end_link = config.end_link;
    end
end

%% 5. 提取每个chain并保存为URDF
fprintf('\n提取并保存chain URDF文件...\n');

for i = 1:length(chain_configs)
    config = chain_configs{i};
    
    if isempty(config.end_link)
        fprintf('跳过 %s（未找到末端link）\n', config.name);
        continue;
    end
    
    try
        fprintf('\n处理 %s (末端link: %s)...\n', config.name, config.end_link);
        
        % 提取subtree
        robot_chain = subtree(robot_full, config.end_link);
        robot_chain.DataFormat = 'row';
        
        fprintf('  子链bodies数: %d\n', robot_chain.NumBodies);
        fprintf('  子链关节数: %d\n', robot_chain.NumBodies - 1);
        
        % 保存为URDF文件
        output_file = fullfile(opts.output_dir, sprintf('E1_%s.urdf', config.name));
        
        % 检查MATLAB版本，使用合适的导出方法
        if exist('exportrobot', 'file') == 2
            % R2023b+ 使用 exportrobot
            exportrobot(robot_chain, 'OutputFileName', output_file, 'ExportMesh', false, ...
                'RobotName', sprintf('E1_%s', config.name));
        elseif exist('urdfExporter', 'class') == 8
            % R2023b+ 使用 urdfExporter
            exporter = urdfExporter(robot_chain);
            exporter.OutputFileName = output_file;
            exporter.ExportMesh = false;
            exporter.RobotName = sprintf('E1_%s', config.name);
            writefile(exporter);
        else
            error('当前MATLAB版本不支持导出URDF，需要R2023b或更高版本');
        end
        
        fprintf('  已保存: %s\n', output_file);
        
    catch ME
        warning('提取 %s 失败: %s', config.name, ME.message);
    end
end

fprintf('\n===== 完成 =====\n');
fprintf('生成的URDF文件保存在: %s\n', opts.output_dir);

end
