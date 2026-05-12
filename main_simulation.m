function main_simulation()
% main_simulation - 主仿真脚本：动态扩散溢油的三维Voronoi覆盖控制
%
% 运行方式: cd 到项目根目录, addpath(genpath('.')), main_simulation

    %% 1. 初始化参数
    params = init_parameters();

    %% 2. 初始化智能体位置（固定随机种子保证可重复性）
    rng(42);
    init_pos_adaptive = init_agents(params);
    init_pos_static   = init_agents_uniform(params);
    init_pos_random   = init_pos_adaptive;

    pos_adaptive = init_pos_adaptive;
    pos_static   = init_pos_static;
    pos_random   = init_pos_random;

    %% 3. 记录数组
    steps = params.sim.total_time;
    H_adaptive = zeros(1, steps);
    H_static   = zeros(1, steps);
    H_random   = zeros(1, steps);
    trajectory_adaptive = zeros(steps, params.agent.num, 3);
    move_distances = zeros(1, steps);
    boundary_distances = zeros(1, steps);
    monitored_fraction = zeros(1, steps);
    plume_extent = zeros(1, steps);

    prev_centroids = [];

    %% 4. 主仿真循环
    fprintf('开始仿真: %d步, dt=%.1fs, %d个智能体\n', ...
        steps, params.algorithm.dt, params.agent.num);
    fprintf('----------------------------------------------\n');

    for step = 1:steps
        t = step * params.algorithm.dt;

        [pos_adaptive, centroids, move_dist, sample_data] = ...
            lloyd_iteration(pos_adaptive, t, prev_centroids, params);
        prev_centroids = centroids;
        H_adaptive(step) = coverage_quality(pos_adaptive, sample_data, t, params);
        trajectory_adaptive(step, :, :) = pos_adaptive;
        move_distances(step) = move_dist;
        boundary_distances(step) = sample_data.boundary_distance;
        monitored_fraction(step) = compute_monitoring_fraction(pos_adaptive, sample_data.boundary_points, params);
        plume_extent(step) = estimate_plume_extent(t, params);

        pos_static = static_coverage(pos_static, t, params);
        H_static(step) = coverage_quality(pos_static, [], t, params);

        pos_random = random_coverage(pos_random, t, params);
        H_random(step) = coverage_quality(pos_random, [], t, params);

        if mod(step, 20) == 0 || step == 1
            fprintf('Step %3d/%d, t=%6.1fs | H_adapt=%.2e, H_static=%.2e, H_random=%.2e | 边界距=%.2fm | 监测率=%.1f%%\n', ...
                step, steps, t, H_adaptive(step), H_static(step), H_random(step), boundary_distances(step), monitored_fraction(step) * 100);
        end
    end

    fprintf('----------------------------------------------\n');
    fprintf('仿真完成!\n');
    fprintf('H_adaptive: %.2e → %.2e (下降 %.1f%%)\n', ...
        H_adaptive(1), H_adaptive(end), (H_adaptive(1)-H_adaptive(end))/H_adaptive(1)*100);
    fprintf('H_static:   %.2e → %.2e\n', H_static(1), H_static(end));
    fprintf('H_random:   %.2e → %.2e\n', H_random(1), H_random(end));
    fprintf('羽流有效体积: %.2e → %.2e m^3\n', plume_extent(1), plume_extent(end));

    %% 5. 结果可视化和保存
    save_dir = fullfile(pwd, '测试结果', 'Step8_动态扩散边界追踪');
    if ~exist(save_dir, 'dir'), mkdir(save_dir); end

    plot_results(H_adaptive, H_static, H_random, params);
    exportgraphics(gcf, fullfile(save_dir, 'step8_coverage_quality.png'), 'Resolution', 150);

    plume_state = update_plume(steps * params.algorithm.dt, params);
    plot_plume_agents_3d(pos_adaptive, trajectory_adaptive, plume_state, params);
    exportgraphics(gcf, fullfile(save_dir, 'step8_plume_agents_tracking.png'), 'Resolution', 150);

    plot_voronoi_3d(pos_adaptive, steps * params.algorithm.dt, params);
    exportgraphics(gcf, fullfile(save_dir, 'step8_voronoi_3d.png'), 'Resolution', 150);

    plot_boundary_tracking(boundary_distances, plume_extent, params);
    exportgraphics(gcf, fullfile(save_dir, 'step8_boundary_tracking.png'), 'Resolution', 150);

    plot_monitoring_dashboard(H_adaptive, H_static, H_random, boundary_distances, plume_extent, monitored_fraction, trajectory_adaptive, params);
    exportgraphics(gcf, fullfile(save_dir, 'step8_monitoring_dashboard.png'), 'Resolution', 150);

    create_dynamic_plume_gif(trajectory_adaptive, params, fullfile(save_dir, 'step8_dynamic_plume_monitoring.gif'));

    write_step8_analysis(save_dir, H_adaptive, H_static, H_random, boundary_distances, plume_extent, monitored_fraction, params);

    fprintf('\n所有图表和结果分析已保存到: %s\n', save_dir);
end
