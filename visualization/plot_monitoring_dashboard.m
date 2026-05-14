function plot_monitoring_dashboard(H_proposed, H_cvt, H_lawnmower, ...
    cr_proposed, cr_cvt, cr_lawnmower, ...
    rmse_proposed, rmse_cvt, rmse_lawnmower, ...
    boundary_distances, plume_extent, monitored_fraction, trajectory, ...
    velocities, params)
% plot_monitoring_dashboard - 智能体监测效果综合展示（2×4面板）
%
% 输入:
%   H_proposed, H_cvt, H_lawnmower: 覆盖质量序列
%   cr_proposed, cr_cvt, cr_lawnmower: 动态覆盖率序列
%   rmse_proposed, rmse_cvt, rmse_lawnmower: 边界追踪RMSE序列
%   boundary_distances: 平均Boundary Distance
%   plume_extent: 羽流有效体积
%   monitored_fraction: 羽流边界被监测比例
%   trajectory: AUV轨迹
%   velocities: steps x n 各AUV实时速度
%   params: 参数结构体

    figure('Name', 'AUV Dynamic Monitoring Dashboard', 'Position', [20 20 1800 850]);
    t = params.sim.pre_release_time + (0:length(H_proposed)-1) * params.algorithm.dt;
    colors = lines(params.agent.num);
    trajectory_vis = smooth_trajectory(trajectory);

    % 面板1: Coverage Quality H(t) 对比
    subplot(2,4,1);
    plot(t, H_lawnmower, 'r--', 'LineWidth', 1.8); hold on;
    plot(t, H_cvt, 'g:', 'LineWidth', 1.8);
    plot(t, H_proposed, 'b-', 'LineWidth', 2.2);
    hold off;
    xlabel('Leak Time (s)'); ylabel('Coverage Quality H(t)');
    title('Coverage Quality Comparison');
    legend('Lawnmower', 'Standard CVT', 'Proposed CVT-DBT', 'Location', 'best');
    grid on;

    % 面板2: 动态覆盖率 CR(t) 对比
    subplot(2,4,2);
    plot(t, cr_lawnmower * 100, 'r--', 'LineWidth', 1.8); hold on;
    plot(t, cr_cvt * 100, 'g:', 'LineWidth', 1.8);
    plot(t, cr_proposed * 100, 'b-', 'LineWidth', 2.2);
    hold off;
    xlabel('Leak Time (s)'); ylabel('Dynamic Coverage Ratio (%)');
    title('Dynamic Coverage Ratio Comparison');
    legend('Lawnmower', 'Standard CVT', 'Proposed CVT-DBT', 'Location', 'best');
    ylim([0 100]);
    grid on;

    % 面板3: 边界追踪 RMSE 对比
    subplot(2,4,3);
    plot(t, rmse_lawnmower, 'r--', 'LineWidth', 1.8); hold on;
    plot(t, rmse_cvt, 'g:', 'LineWidth', 1.8);
    plot(t, rmse_proposed, 'b-', 'LineWidth', 2.2);
    yline(params.agent.sense_radius, 'k-.', 'LineWidth', 1);
    hold off;
    xlabel('Leak Time (s)'); ylabel('Boundary Tracking RMSE (m)');
    title('Boundary Tracking RMSE Comparison');
    legend('Lawnmower', 'Standard CVT', 'Proposed', sprintf('R_s=%dm', params.agent.sense_radius), 'Location', 'best');
    grid on;

    % 面板4: AUV实时速度
    subplot(2,4,4);
    hold on;
    for i = 1:params.agent.num
        plot(t, velocities(:, i), 'Color', colors(i,:), 'LineWidth', 1.2, ...
             'DisplayName', sprintf('AUV %d', i));
    end
    yline(params.agent.max_speed, 'k--', 'LineWidth', 1.2, 'DisplayName', 'v_{max}');
    hold off;
    xlabel('Leak Time (s)'); ylabel('Speed (m/s)');
    title('Real-Time AUV Control Inputs');
    legend('Location', 'eastoutside', 'FontSize', 6);
    grid on;

    % 面板5: Boundary Distance + Monitoring Fraction
    subplot(2,4,5);
    yyaxis left;
    plot(t, boundary_distances, 'b-', 'LineWidth', 2);
    ylabel('Mean Boundary Distance (m)');
    yyaxis right;
    plot(t, monitored_fraction * 100, 'm--', 'LineWidth', 2);
    ylabel('Boundary Monitoring Fraction (%)');
    xlabel('Leak Time (s)');
    title('Proposed Boundary Tracking and Monitoring Fraction');
    legend('Boundary Distance', 'Monitoring Fraction', 'Location', 'best');
    grid on;

    % 面板6: 羽流体积演化
    subplot(2,4,6);
    plot(t, plume_extent, 'Color', [0.9 0.35 0], 'LineWidth', 2);
    xlabel('Leak Time (s)'); ylabel('Effective Plume Volume (m^3)');
    title('Oil Plume Volume Expansion');
    grid on;

    % 面板7: AUV累计巡航距离
    subplot(2,4,7);
    hold on;
    for i = 1:params.agent.num
        traj_i = squeeze(trajectory_vis(:, i, :));
        travel = [0; cumsum(sqrt(sum(diff(traj_i, 1, 1).^2, 2)))];
        plot(t, travel, 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', sprintf('AUV %d', i));
    end
    hold off;
    xlabel('Leak Time (s)'); ylabel('Cumulative Travel Distance (m)');
    title('AUV Cumulative Travel Distance');
    legend('Location', 'eastoutside', 'FontSize', 6);
    grid on;

    % 面板8: Fleet Speed Statistics
    subplot(2,4,8);
    avg_speed = mean(velocities, 2);
    max_speed_t = max(velocities, [], 2);
    plot(t, avg_speed, 'b-', 'LineWidth', 2, 'DisplayName', 'Mean Speed'); hold on;
    plot(t, max_speed_t, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Maximum Speed');
    yline(params.agent.max_speed, 'k--', 'LineWidth', 1.2, 'DisplayName', 'v_{max}');
    hold off;
    xlabel('Leak Time (s)'); ylabel('Speed (m/s)');
    title('Fleet Speed Statistics');
    legend('Location', 'best', 'FontSize', 9);
    grid on;
end
