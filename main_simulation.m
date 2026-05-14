function main_simulation()
% main_simulation - 主仿真脚本：动态扩散溢油的三维Voronoi覆盖控制
%
% 运行方式: cd 到项目根目录, addpath(genpath('.')), main_simulation
%
% 三种方法对比:
%   Proposed CVT-DBT  - 三维自适应Voronoi + 动态边界追踪
%   Standard CVT      - 标准质心Voronoi细分（无边界/前馈/分离）
%   Lawnmower CPP     - 三维割草机式预规划扫描路径

    %% 1. 初始化参数
    params = init_parameters();

    %% 2. 初始化智能体位置（固定随机种子保证可重复性）
    rng(42);
    init_pos_proposed = init_agents(params);

    % Proposed CVT-DBT：环源部署
    pos_proposed = init_pos_proposed;
    % Standard CVT：相同初始位置（公平对比）
    pos_cvt = init_pos_proposed;
    % Lawnmower CPP：预规划起始位置
    pos_lawnmower = lawnmower_coverage([], params.sim.pre_release_time, params);

    %% 3. 记录数组
    steps = params.sim.total_time;
    n_agents = params.agent.num;
    H_proposed   = zeros(1, steps);
    H_cvt        = zeros(1, steps);
    H_lawnmower  = zeros(1, steps);
    trajectory_proposed = zeros(steps, n_agents, 3);
    velocities_proposed = zeros(steps, n_agents);  % 各AUV实时速度
    move_distances = zeros(1, steps);
    boundary_distances = zeros(1, steps);
    monitored_fraction = zeros(1, steps);
    plume_extent = zeros(1, steps);
    coverage_ratio_proposed  = zeros(1, steps);
    coverage_ratio_cvt       = zeros(1, steps);
    coverage_ratio_lawnmower = zeros(1, steps);
    boundary_rmse_proposed  = zeros(1, steps);
    boundary_rmse_cvt       = zeros(1, steps);
    boundary_rmse_lawnmower = zeros(1, steps);

    prev_centroids = [];

    %% 4. 主仿真循环
    fprintf('开始仿真: %d步, dt=%.1fs, %d个智能体, 预泄漏 %.1fs\n', ...
        steps, params.algorithm.dt, n_agents, params.sim.pre_release_time);
    fprintf('对比方法: Proposed CVT-DBT / Standard CVT / Lawnmower CPP\n');
    fprintf('----------------------------------------------\n');

    for step = 1:steps
        t_monitor = (step - 1) * params.algorithm.dt;
        t_leak = params.sim.pre_release_time + t_monitor;

        % 记录Proposed方法上一步位置（用于计算速度）
        prev_pos_proposed = pos_proposed;

        % === Proposed CVT-DBT（Lloyd迭代 + 前馈 + 边界追踪 + 分离） ===
        [pos_proposed, centroids, move_dist, sample_data] = ...
            lloyd_iteration(pos_proposed, t_leak, prev_centroids, params);
        prev_centroids = centroids;
        H_proposed(step) = coverage_quality(pos_proposed, [], t_leak, params);
        trajectory_proposed(step, :, :) = pos_proposed;
        move_distances(step) = move_dist;
        boundary_distances(step) = sample_data.boundary_distance;
        monitored_fraction(step) = compute_monitoring_fraction(pos_proposed, sample_data.boundary_points, params);

        % 计算各AUV实时速度
        if step == 1
            velocities_proposed(step, :) = 0;
        else
            displacement = sqrt(sum((pos_proposed - prev_pos_proposed).^2, 2));
            velocities_proposed(step, :) = displacement' / params.algorithm.dt;
        end

        % === Standard CVT（纯质心反馈，展示集群塌陷） ===
        pos_cvt = standard_cvt(pos_cvt, t_leak, params);
        H_cvt(step) = coverage_quality(pos_cvt, [], t_leak, params);

        % === Lawnmower CPP（预规划锯齿扫描，开环控制） ===
        pos_lawnmower = lawnmower_coverage(pos_lawnmower, t_leak, params);
        H_lawnmower(step) = coverage_quality(pos_lawnmower, [], t_leak, params);

        % === 共享指标（单次update_plume调用） ===
        plume_state = update_plume(t_leak, params);
        plume_extent(step) = estimate_plume_extent(t_leak, params);

        coverage_ratio_proposed(step)  = compute_dynamic_coverage_ratio(pos_proposed, params, plume_state);
        coverage_ratio_cvt(step)       = compute_dynamic_coverage_ratio(pos_cvt, params, plume_state);
        coverage_ratio_lawnmower(step) = compute_dynamic_coverage_ratio(pos_lawnmower, params, plume_state);

        boundary_rmse_proposed(step)  = compute_boundary_rmse(pos_proposed, params, plume_state);
        boundary_rmse_cvt(step)       = compute_boundary_rmse(pos_cvt, params, plume_state);
        boundary_rmse_lawnmower(step) = compute_boundary_rmse(pos_lawnmower, params, plume_state);

        if mod(step, 20) == 0 || step == 1
            fprintf('Step %3d/%d, 泄漏t=%6.1fs | H: Prop=%.2e CVT=%.2e Lawn=%.2e | CR: %.1f%% %.1f%% %.1f%% | RMSE: %.1f %.1f %.1f\n', ...
                step, steps, t_leak, H_proposed(step), H_cvt(step), H_lawnmower(step), ...
                coverage_ratio_proposed(step)*100, coverage_ratio_cvt(step)*100, coverage_ratio_lawnmower(step)*100, ...
                boundary_rmse_proposed(step), boundary_rmse_cvt(step), boundary_rmse_lawnmower(step));
        end
    end

    fprintf('----------------------------------------------\n');
    fprintf('仿真完成!\n');
    fprintf('H_proposed:   %.2e → %.2e (下降 %.1f%%)\n', ...
        H_proposed(1), H_proposed(end), (H_proposed(1)-H_proposed(end))/H_proposed(1)*100);
    fprintf('H_cvt:        %.2e → %.2e\n', H_cvt(1), H_cvt(end));
    fprintf('H_lawnmower:  %.2e → %.2e\n', H_lawnmower(1), H_lawnmower(end));
    fprintf('羽流有效体积: %.2e → %.2e m^3\n', plume_extent(1), plume_extent(end));
    fprintf('最终覆盖率: Proposed=%.1f%%, CVT=%.1f%%, Lawnmower=%.1f%%\n', ...
        coverage_ratio_proposed(end)*100, coverage_ratio_cvt(end)*100, coverage_ratio_lawnmower(end)*100);
    fprintf('Proposed平均速度: %.2f m/s, 最大速度: %.2f m/s\n', ...
        mean(velocities_proposed(end-9:end), 'all'), max(velocities_proposed(end-9:end), [], 'all'));

    %% 5. 结果可视化和保存
    save_dir = fullfile(pwd, '测试结果', 'Step8_动态扩散边界追踪');
    if ~exist(save_dir, 'dir'), mkdir(save_dir); end

    plot_results(H_proposed, H_cvt, H_lawnmower, params);
    exportgraphics(gcf, fullfile(save_dir, 'step8_coverage_quality.png'), 'Resolution', 150);

    t_final_leak = params.sim.pre_release_time + (steps - 1) * params.algorithm.dt;

    plume_state = update_plume(t_final_leak, params);
    plot_plume_agents_3d(pos_proposed, trajectory_proposed, plume_state, params);
    exportgraphics(gcf, fullfile(save_dir, 'step8_plume_agents_tracking.png'), 'Resolution', 150);

    plot_voronoi_3d(pos_proposed, t_final_leak, params);
    exportgraphics(gcf, fullfile(save_dir, 'step8_voronoi_3d.png'), 'Resolution', 150);

    plot_boundary_tracking(boundary_distances, plume_extent, ...
        boundary_rmse_proposed, boundary_rmse_cvt, boundary_rmse_lawnmower, params);
    exportgraphics(gcf, fullfile(save_dir, 'step8_boundary_tracking.png'), 'Resolution', 150);

    plot_coverage_metrics(coverage_ratio_proposed, coverage_ratio_cvt, coverage_ratio_lawnmower, ...
        boundary_rmse_proposed, boundary_rmse_cvt, boundary_rmse_lawnmower, params);
    exportgraphics(gcf, fullfile(save_dir, 'step8_coverage_metrics.png'), 'Resolution', 150);

    plot_control_input(velocities_proposed, params);
    exportgraphics(gcf, fullfile(save_dir, 'step8_control_input.png'), 'Resolution', 150);

    plot_monitoring_dashboard(H_proposed, H_cvt, H_lawnmower, ...
        coverage_ratio_proposed, coverage_ratio_cvt, coverage_ratio_lawnmower, ...
        boundary_rmse_proposed, boundary_rmse_cvt, boundary_rmse_lawnmower, ...
        boundary_distances, plume_extent, monitored_fraction, trajectory_proposed, ...
        velocities_proposed, params);
    exportgraphics(gcf, fullfile(save_dir, 'step8_monitoring_dashboard.png'), 'Resolution', 150);

    create_dynamic_plume_gif(trajectory_proposed, params, fullfile(save_dir, 'step8_dynamic_plume_monitoring.gif'));

    write_step8_analysis(save_dir, H_proposed, H_cvt, H_lawnmower, ...
        boundary_distances, plume_extent, monitored_fraction, ...
        coverage_ratio_proposed, coverage_ratio_cvt, coverage_ratio_lawnmower, ...
        boundary_rmse_proposed, boundary_rmse_cvt, boundary_rmse_lawnmower, ...
        velocities_proposed, params);

    fprintf('\n所有图表和结果分析已保存到: %s\n', save_dir);
end
