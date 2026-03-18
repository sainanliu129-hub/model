% export_motor_spec_to_excel  按「静摩擦 / 动摩擦 / 转动惯量」三部分导出 Excel
%
% 每张表：该部分的辨识参数/数据；若 EmbedFigures=true 则用 Python 脚本将附图嵌入表内
% （无需 Excel，Linux + LibreOffice Calc 可用；需安装 Python3 与 openpyxl、Pillow）。
%
% 用法:
%   export_motor_spec_to_excel('MotorName', motor_name, 'FigureDir', results_dir, ...
%       'K_tau', K_tau, 'tau_s', tau_s, 'Result_friction', Result_friction, ...
%       'I_a', I_a, 'I_a_R2', I_a_R2, 'RMSE_overall', RMSE_overall, 'R2_overall', R2_overall);
%
% 可选名值对:
%   'OutPath'        - 输出 xlsx 路径
%   'MotorName'      - 电机型号
%   'FigureDir'      - 附图所在目录（build）
%   'K_tau'          - 力矩系数 N·m/A
%   'tau_s'          - 静摩擦 N·m
%   'file_static'    - 静摩擦数据文件路径（可选）
%   'Result_friction'- 动摩擦辨识结果结构体
%   'I_a'            - 转动惯量 kg·m²
%   'I_a_R2'         - 惯量拟合 R²
%   'RMSE_overall'   - 整体拟合 RMSE N·m
%   'R2_overall'     - 整体拟合 R²
%   'EmbedFigures'   - true：用 Python 脚本将附图嵌入表内（默认）；false：不嵌图

function export_motor_spec_to_excel(varargin)
p = inputParser;
addParameter(p, 'OutPath', '', @ischar);
addParameter(p, 'MotorName', '电机型号', @ischar);
addParameter(p, 'FigureDir', '', @ischar);
addParameter(p, 'K_tau', NaN, @isnumeric);
addParameter(p, 'tau_s', NaN, @isnumeric);
addParameter(p, 'file_static', '', @ischar);
addParameter(p, 'Result_friction', [], @(x) isempty(x) || isstruct(x));
addParameter(p, 'I_a', NaN, @isnumeric);
addParameter(p, 'I_a_R2', NaN, @isnumeric);
addParameter(p, 'RMSE_overall', NaN, @isnumeric);
addParameter(p, 'R2_overall', NaN, @isnumeric);
addParameter(p, 'EmbedFigures', true, @islogical);
parse(p, varargin{:});
opts = p.Results;

motor_name = opts.MotorName;
cur = fileparts(mfilename('fullpath'));
if isempty(opts.OutPath)
    safe_name = regexprep(motor_name, '\s+', '_');
    opts.OutPath = fullfile(cur, 'build', [safe_name '_静摩擦_动摩擦_转动惯量.xlsx']);
end
out_dir = fileparts(opts.OutPath);
if ~isempty(out_dir) && ~exist(out_dir, 'dir')
    mkdir(out_dir);
end
fig_dir = opts.FigureDir;
if isempty(fig_dir), fig_dir = fullfile(cur, 'build'); end

%% Sheet1: 静摩擦
rows = {};
rows{end+1} = {'项目', '值', '单位/备注'};
rows{end+1} = {'电机型号', motor_name, ''};
rows{end+1} = {'力矩系数 K_tau', opts.K_tau, 'N·m/A'};
rows{end+1} = {'静摩擦 τ_s', opts.tau_s, 'N·m（电流扫描静止段）'};
if ~isempty(opts.file_static)
    [~, fn, ext] = fileparts(opts.file_static);
    rows{end+1} = {'数据文件', [fn, ext], ''};
end
rows{end+1} = {'附图', '见右侧', ''};
data = vertcat(rows{2:end});
T1 = cell2table(data, 'VariableNames', rows{1});
writetable(T1, opts.OutPath, 'Sheet', '静摩擦', 'WriteVariableNames', true);

%% Sheet2: 动摩擦
rows = {};
rows{end+1} = {'项目', '值', '单位/备注'};
rows{end+1} = {'电机型号', motor_name, ''};
if ~isempty(opts.Result_friction)
    R = opts.Result_friction;
    rows{end+1} = {'摩擦模型', R.FrictionModel, ''};
    if isfield(R, 'tau_s'), rows{end+1} = {'静摩擦 τ_s（代入）', R.tau_s, 'N·m'}; end
    % tanh_viscous 只显示 tanh 参数（τ_c/b 在代码里 = μ_s/μ_d，仅惯量辨识用，表里不重复）
    if strcmpi(R.FrictionModel, 'tanh_viscous')
        if isfield(R, 'mu_s'), rows{end+1} = {'μ_s', R.mu_s, 'N·m'}; end
        if isfield(R, 'v_act'), rows{end+1} = {'v_act', R.v_act, 'rad/s'}; end
        if isfield(R, 'mu_d'), rows{end+1} = {'μ_d', R.mu_d, 'N·m·s/rad'}; end
    elseif strcmpi(R.FrictionModel, 'stribeck_viscous')
        if isfield(R, 'tau_c'), rows{end+1} = {'τ_c', R.tau_c, 'N·m'}; end
        if isfield(R, 'v_s'), rows{end+1} = {'v_s', R.v_s, 'rad/s'}; end
        if isfield(R, 'b'), rows{end+1} = {'b', R.b, 'N·m·s/rad'}; end
        if isfield(R, 'tau0'), rows{end+1} = {'τ_0', R.tau0, 'N·m'}; end
    else
        % coulomb_viscous 或其它
        if isfield(R, 'tau_c_pos'), rows{end+1} = {'τ_c_pos', R.tau_c_pos, 'N·m'}; end
        if isfield(R, 'tau_c_neg'), rows{end+1} = {'τ_c_neg', R.tau_c_neg, 'N·m'}; end
        if isfield(R, 'tau_c'), rows{end+1} = {'库伦 τ_c', R.tau_c, 'N·m'}; end
        if isfield(R, 'b'), rows{end+1} = {'粘滞 b', R.b, 'N·m·s/rad'}; end
    end
    if isfield(R, 'R2'), rows{end+1} = {'摩擦拟合 R²', R.R2, ''}; end
    if isfield(R, 'RMSE'), rows{end+1} = {'摩擦拟合 RMSE', R.RMSE, 'N·m'}; end
    if isfield(R, 'nUsed'), rows{end+1} = {'有效样本数 nUsed', R.nUsed, ''}; end
end
rows{end+1} = {'附图', '见右侧', ''};
data = vertcat(rows{2:end});
T2 = cell2table(data, 'VariableNames', rows{1});
writetable(T2, opts.OutPath, 'Sheet', '动摩擦', 'WriteVariableNames', true);

%% Sheet3: 转动惯量
rows = {};
rows{end+1} = {'项目', '值', '单位/备注'};
rows{end+1} = {'电机型号', motor_name, ''};
rows{end+1} = {'转动惯量 I_a', opts.I_a, 'kg·m²'};
rows{end+1} = {'惯量拟合 R²', opts.I_a_R2, ''};
rows{end+1} = {'整体拟合 RMSE', opts.RMSE_overall, 'N·m'};
rows{end+1} = {'整体拟合 R²', opts.R2_overall, ''};
rows{end+1} = {'附图', '见下方', ''};
data = vertcat(rows{2:end});
T3 = cell2table(data, 'VariableNames', rows{1});
writetable(T3, opts.OutPath, 'Sheet', '转动惯量', 'WriteVariableNames', true);

%% 用 Python 脚本将图嵌入表内（跨平台，无需 Excel）
if opts.EmbedFigures
    ok = embed_figures_with_python(opts.OutPath, fig_dir, motor_name, cur);
    if ok
        fprintf('  附图已嵌入各表（Python openpyxl）。\n');
    else
        fprintf('  附图未嵌入（请安装: pip install openpyxl Pillow，或见 build 目录下 PNG）。\n');
    end
end

fprintf('已生成 Excel: %s\n', opts.OutPath);
fprintf('  表: 静摩擦, 动摩擦, 转动惯量\n');
end

function ok = embed_figures_with_python(xlsx_path, fig_dir, motor_name, script_dir)
% 调用同目录下的 embed_figures_xlsx.py 将 PNG 嵌入 xlsx
py_script = fullfile(script_dir, 'embed_figures_xlsx.py');
if exist(py_script, 'file') ~= 2
    ok = false;
    return;
end
xlsx_path = fullfile(xlsx_path);
fig_dir = fullfile(fig_dir);
% 强制转为绝对路径，避免 Python 侧 cwd 与路径不一致导致找不到图
try
    fig_dir = char(java.io.File(fig_dir).getAbsolutePath());
catch
    [st, res] = system(['realpath "' fig_dir '" 2>/dev/null']);
    if st == 0, fig_dir = strtrim(res); end
end
cmd = sprintf('python3 "%s" "%s" "%s" "%s" 2>&1', py_script, xlsx_path, fig_dir, motor_name);
[st, msg] = system(cmd);
ok = (st == 0);
if ~ok && ~isempty(msg)
    fprintf('  Python 嵌图: %s\n', strrep(msg, newline, [newline '  ']));
end
end
