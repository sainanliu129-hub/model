function plot_compare_6dof(X, Y, CurveType, LegendString, PlotType)


if nargin < 5
    PlotType = {'k', 'r', 'b--', 'g--', 'm', 'y', 'r--', 'b--', 'g--', 'm--'};
    if nargin < 4
        LegendString = [];
        if nargin < 3
            CurveType = 'none';
        end
    end    
end

if isempty(X) == 1
    sampleRate = 200;
    X = [1 : size(Y, 1)]' / sampleRate;
end

if length(X) ~= size(Y, 1)
    disp('ERROR in FUNCTION: set_plot_compare, Error Type: size not match.');
    return;
end

if strcmp(CurveType, 'q')
    label_x = {'time / s', 'time / s', 'time / s', 'time / s', 'time / s', 'time / s'};
    label_y = {'q / rad', 'q / rad', 'q / rad', 'q / rad', 'q / rad', 'q / rad'};
    label_title = 'Joint Angle Curve';
elseif strcmp(CurveType, 'qd')
    label_x = {'time / s', 'time / s', 'time / s', 'time / s', 'time / s', 'time / s'};
    label_y = {'qd / rad', 'qd / rad', 'qd / rad', 'qd / rad', 'qd / rad', 'qd / rad'};
    label_title = 'Joint Velocity Curve';
elseif strcmp(CurveType, 'qdd')
    label_x = {'time / s', 'time / s', 'time / s', 'time / s', 'time / s', 'time / s'};
    label_y = {'qdd / rad', 'qdd / rad', 'qdd / rad', 'qdd / rad', 'qdd / rad', 'qdd / rad'};
    label_title = 'Joint Acceleration Curve';
elseif strcmp(CurveType, 'torque')
    label_x = {'time / s', 'time / s', 'time / s', 'time / s', 'time / s', 'time / s'};
    label_y = {'torque / N*m', 'torque / N*m', 'torque / N*m', 'torque / N*m', 'torque / N*m', 'torque / N*m'};
    label_title = 'Joint Torque Curve';
elseif strcmp(CurveType, 'current')
    label_x = {'time / s', 'time / s', 'time / s', 'time / s', 'time / s', 'time / s'};
    label_y = {'current / A', 'current / A', 'current / A', 'current / A', 'current / A', 'current / A', };
    label_title = 'Joint Current Curve';
elseif strcmp(CurveType, 'sensor') || strcmp(CurveType, 'force')
    label_x = {'time / s', 'time / s', 'time / s', 'time / s', 'time / s', 'time / s'};
    label_y = {'Force / N', 'Force / N', 'Force / N', 'Moment / N*m', 'Moment / N*m', 'Moment / N*m'};
    label_title = 'Sensor 6-DoF Force Curve';
elseif strcmp(CurveType, 'none')
    label_x = {'', '', '', '', '', ''};
    label_y = {'', '', '', '', '', ''};
    label_title = ' ';
else
    label_x = CurveType.label_x;
    label_y = CurveType.label_y;
    label_title = CurveType.label_title;
end

nDof = 6;

yLength = size(Y, 2) / nDof;

for j = 1 : yLength
    y{j} = Y(:, (j - 1) * nDof + 1 : j * nDof);
end


for i = 1 : nDof
    subplot(2, 3, i)
    hold on;
    for j = 1 : yLength
        h_line = plot(X, y{j}(:, i), PlotType{j}, 'LineWidth', 1.5);
        if i == 1
            h_legend_lines(j) = h_line; %#ok<AGROW>
        end
    end
    hold off;
    xlabel(label_x{i});
    ylabel(label_y{i});
end
% Legend 只保留一个，并固定到整张图右上角区域，避免遮挡任一子图
if isempty(LegendString) == 0 && length(LegendString) == yLength
    try
        h_leg = legend(h_legend_lines, LegendString, ...
            'Units', 'normalized', ...
            'Position', [0.86, 0.90, 0.13, 0.08], ...
            'Location', 'none');
        set(h_leg, 'Box', 'on');
    catch
        legend(LegendString);
    end
end
local_apply_super_title(label_title);
function local_apply_super_title(txt)
% 兼容不同 MATLAB 版本：优先 sgtitle，其次 suptitle，最后 annotation
if exist('sgtitle', 'file') == 2 || exist('sgtitle', 'builtin') == 5
    sgtitle(txt);
    return;
end
if exist('suptitle', 'file') == 2 || exist('suptitle', 'builtin') == 5
    suptitle(txt);
    return;
end
annotation('textbox', [0 0.96 1 0.04], ...
    'String', txt, 'EdgeColor', 'none', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold');
end
end
    




