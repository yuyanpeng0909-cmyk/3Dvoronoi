function plot_boundary_tracking(boundary_distances, plume_extent, params)
% plot_boundary_tracking - 绘制溢油扩散范围和边界追踪距离指标
%
% 输入:
%   boundary_distances: 1 x steps 平均边界距离
%   plume_extent: 1 x steps 羽流有效范围体积
%   params: 参数结构体

    figure('Name', '动态边界追踪指标', 'Position', [100 100 900 550]);
    t = (1:length(boundary_distances)) * params.algorithm.dt;

    yyaxis left;
    plot(t, boundary_distances, 'b-', 'LineWidth', 2);
    ylabel('平均边界距离 (m)', 'FontSize', 12);

    yyaxis right;
    plot(t, plume_extent, 'r--', 'LineWidth', 2);
    ylabel('羽流有效体积估计 (m³)', 'FontSize', 12);

    xlabel('时间 (s)', 'FontSize', 12);
    title('溢油扩散范围与AUV边界追踪性能', 'FontSize', 14);
    legend('AUV到扩散边界平均距离', '羽流有效体积', 'Location', 'best');
    grid on;
    set(gca, 'FontSize', 11);
end
