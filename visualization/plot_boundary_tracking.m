function plot_boundary_tracking(boundary_distances, plume_extent, ...
    rmse_proposed, rmse_cvt, rmse_lawnmower, params)
% plot_boundary_tracking - 绘制溢油扩散范围、边界追踪距离和RMSE指标
%
% 输入:
%   boundary_distances: 1 x steps Proposed方法平均边界距离
%   plume_extent: 1 x steps 羽流有效体积
%   rmse_proposed, rmse_cvt, rmse_lawnmower: 1 x steps 三种方法边界RMSE
%   params: 参数结构体

    figure('Name', 'Dynamic Boundary Tracking Metrics', 'Position', [100 100 1000 600]);
    steps = length(boundary_distances);
    t = params.sim.pre_release_time + (0:steps-1) * params.algorithm.dt;

    % 左Y轴：羽流体积（橙色，明确图例）
    yyaxis left;
    h1 = plot(t, plume_extent / 1e6, 'Color', [0.85 0.33 0.1], 'LineWidth', 2.2, ...
              'DisplayName', 'Effective plume volume');
    ylabel('Effective Plume Volume (×10^6 m^3)', 'FontSize', 12);
    set(gca, 'YColor', [0.85 0.33 0.1]);

    % 右Y轴：距离/RMSE
    yyaxis right;
    hold on;
    h2 = plot(t, boundary_distances, 'b-', 'LineWidth', 2, 'DisplayName', 'Proposed boundary distance');
    h3 = plot(t, rmse_proposed, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Proposed RMSE');
    h4 = plot(t, rmse_cvt, 'g:', 'LineWidth', 1.5, 'DisplayName', 'CVT RMSE');
    h5 = plot(t, rmse_lawnmower, 'r-.', 'LineWidth', 1.5, 'DisplayName', 'Lawnmower RMSE');
    hold off;
    ylabel('Distance (m)', 'FontSize', 12);

    xlabel('Leak Time (s)', 'FontSize', 12);
    title('Plume Expansion and Boundary Tracking Performance', 'FontSize', 14);
    legend([h1, h2, h3, h4, h5], 'Location', 'best', 'FontSize', 10);
    grid on;
    set(gca, 'FontSize', 11);
end
