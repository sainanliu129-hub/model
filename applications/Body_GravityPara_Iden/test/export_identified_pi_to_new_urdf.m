%% export_identified_pi_to_new_urdf
% 从辨识结果（pi_cad / pi_rec / pi_phys / pi_fd）中选择一种，
% 参考原始 URDF，导出一份更新后的新 URDF（仅替换指定 limb 的动力学参数）。
%
% 用法：
%   1) 直接运行本脚本，按下方配置修改
%   2) 运行后会生成一个新的 urdf 文件，不会覆盖原始 urdf

clc; clear; close all;

app_root = fullfile(fileparts(mfilename('fullpath')), '..');
repo_root = fullfile(app_root, '..', '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
addpath(fullfile(repo_root, 'utility_function'));

if exist('ensure_body_gravity_para_iden_path', 'file')
    ensure_body_gravity_para_iden_path();
end

%% 1) 配置
limb = 'left_leg';                 % 'left_leg' | 'right_leg' | 'left_arm' | 'right_arm'
result_type = 'pi_phys';           % 'pi_cad' | 'pi_rec' | 'pi_phys' | 'pi_fd'
para_order = 1;                    % 与辨识保持一致：1 -> [m mx my mz Ixx Ixy Ixz Iyy Iyz Izz]
force_psd = true;                  % 惯量写入前投影到 PSD，建议 true（尤其 pi_rec/pi_fd）

step5_mat = fullfile(app_root, 'build', 'step5_full_params.mat');
source_urdf = get_e1_urdf_path();  % 默认 noetix_description/urdf/E1.urdf

out_dir = fileparts(source_urdf);
out_name = sprintf('E1_%s_%s_updated.urdf', limb, result_type);
out_urdf = fullfile(out_dir, out_name);

%% 2) 读取辨识结果
if ~isfile(step5_mat)
    error('找不到辨识结果文件: %s', step5_mat);
end
S = load(step5_mat);

if ~isfield(S, result_type) || isempty(S.(result_type))
    error('step5 文件中不存在或为空: %s', result_type);
end
pi_vec = S.(result_type)(:);

%% 3) 确定 limb 对应的 body 顺序（与 get_e1_limb_robot 一致）
body_names = get_limb_body_names(limb);
n = numel(body_names);
if numel(pi_vec) ~= 10 * n
    error('参数长度不匹配: %s 有 %d 个 body，期望参数长度 %d，实际 %d。', ...
        limb, n, 10*n, numel(pi_vec));
end

%% 4) 先整理 limb 更新参数
if ~isfile(source_urdf)
    error('原始 URDF 不存在: %s', source_urdf);
end
link_params = repmat(struct('name', '', 'mass', 0, 'com', [0 0 0], ...
    'ixx', 0, 'ixy', 0, 'ixz', 0, 'iyy', 0, 'iyz', 0, 'izz', 0), n, 1);
for i = 1:n
    bname = body_names{i};
    idx = (i - 1) * 10 + (1:10);
    p = pi_vec(idx);
    [m, com_xyz, Ixx, Iyy, Izz, Iyz, Ixz, Ixy] = pi_block_to_body_inertia(p, para_order, force_psd);
    link_params(i).name = bname;
    link_params(i).mass = m;
    link_params(i).com = com_xyz(:).';
    link_params(i).ixx = Ixx;
    link_params(i).ixy = Ixy;
    link_params(i).ixz = Ixz;
    link_params(i).iyy = Iyy;
    link_params(i).iyz = Iyz;
    link_params(i).izz = Izz;
end

%% 5) 导出新 URDF
if ~isfolder(out_dir)
    mkdir(out_dir);
end

try
    if exist('exportrobot', 'file') == 2
        % 新版 MATLAB：直接导出 rigidBodyTree
        robot_full = importrobot(source_urdf);
        robot_full.DataFormat = 'row';
        for i = 1:n
            body = robot_full.getBody(link_params(i).name);
            if isempty(body)
                error('在整机 URDF 中未找到 body: %s', link_params(i).name);
            end
            body.Mass = link_params(i).mass;
            body.CenterOfMass = link_params(i).com(:);
            % rigidBody.Inertia 顺序：[Ixx Iyy Izz Iyz Ixz Ixy]
            body.Inertia = [link_params(i).ixx; link_params(i).iyy; link_params(i).izz; ...
                link_params(i).iyz; link_params(i).ixz; link_params(i).ixy];
        end
        exportrobot(robot_full, 'OutputFileName', out_urdf, 'ExportMesh', true, ...
            'RobotName', 'E1_updated');
    elseif exist('urdfExporter', 'class') == 8
        % 部分版本有 urdfExporter
        robot_full = importrobot(source_urdf);
        robot_full.DataFormat = 'row';
        for i = 1:n
            body = robot_full.getBody(link_params(i).name);
            if isempty(body)
                error('在整机 URDF 中未找到 body: %s', link_params(i).name);
            end
            body.Mass = link_params(i).mass;
            body.CenterOfMass = link_params(i).com(:);
            body.Inertia = [link_params(i).ixx; link_params(i).iyy; link_params(i).izz; ...
                link_params(i).iyz; link_params(i).ixz; link_params(i).ixy];
        end
        exporter = urdfExporter(robot_full);
        exporter.OutputFileName = out_urdf;
        exporter.ExportMesh = true;
        exporter.RobotName = 'E1_updated';
        writefile(exporter);
    else
        % 老版本回退：直接改原 URDF 的 XML（不依赖 exportrobot）
        write_updated_urdf_by_xml(source_urdf, out_urdf, link_params);
    end
catch ME
    error('导出 URDF 失败: %s', ME.message);
end

fprintf('已生成新 URDF: %s\n', out_urdf);
fprintf('参数来源: %s (%s)\n', step5_mat, result_type);
fprintf('更新肢体: %s\n', limb);
