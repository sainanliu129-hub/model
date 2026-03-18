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
        plot(X, y{j}(:, i), PlotType{j}, 'LineWidth', 1.5);
    end
    hold off;
    if isempty(LegendString) == 0 && length(LegendString) == yLength
        legend(LegendString);
    end
    xlabel(label_x{i});
    ylabel(label_y{i});
end
suptitle(label_title);
    




