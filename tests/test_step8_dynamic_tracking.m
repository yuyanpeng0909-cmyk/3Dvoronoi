function test_step8_dynamic_tracking()
% test_step8_dynamic_tracking - Step 8 测试：动态扩散边界追踪
% 运行方式: cd 到项目根目录, addpath(genpath('.')), test_step8_dynamic_tracking

fprintf('========================================\n');
fprintf('  Step 8 测试：动态扩散边界追踪\n');
fprintf('========================================\n\n');

pass_count = 0;
fail_count = 0;

params = init_parameters();
params.sim.total_time = 40;
params.algorithm.sample_num = 8000;
params.algorithm.boundary_sample_num = 2500;

%% ========== 测试1: 羽流范围随时间扩大 ==========
fprintf('--- 测试1: 羽流范围随时间扩大 ---\n');
try
    V0 = estimate_plume_extent(10, params);
    V1 = estimate_plume_extent(120, params);
    assert(V1 > V0, sprintf('羽流体积应增长: V0=%.3e, V1=%.3e', V0, V1));
    fprintf('  [PASS] 羽流有效体积 %.3e → %.3e m^3\n', V0, V1);
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试2: 边界采样有效 ==========
fprintf('\n--- 测试2: 边界采样有效 ---\n');
try
    rng(42);
    pos = init_agents(params);
    [boundary_points, assigned_agent, boundary_distance] = sample_plume_boundary(pos, 60, params);
    assert(~isempty(boundary_points), '边界点不应为空');
    assert(size(boundary_points, 2) == 3, '边界点应为三维坐标');
    assert(numel(assigned_agent) == size(boundary_points, 1), '边界分配数量不一致');
    assert(isfinite(boundary_distance), '边界距离应为有限值');
    fprintf('  [PASS] 边界点 %d 个，平均距离 %.2f m\n', size(boundary_points, 1), boundary_distance);
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试3: 快速仿真 ==========
fprintf('\n--- 测试3: 40步动态追踪仿真 ---\n');
try
    rng(42);
    pos_adaptive = init_agents(params);
    pos_static = init_agents_uniform(params);
    pos_random = pos_adaptive;

    steps = params.sim.total_time;
    H_adaptive = zeros(1, steps);
    H_static = zeros(1, steps);
    H_random = zeros(1, steps);
    trajectory = zeros(steps, params.agent.num, 3);
    boundary_distances = zeros(1, steps);
    plume_extent = zeros(1, steps);
    monitored_fraction = zeros(1, steps);
    prev_cen = [];

    for step = 1:steps
        t = step * params.algorithm.dt;
        [pos_adaptive, centroids, ~, sd] = lloyd_iteration(pos_adaptive, t, prev_cen, params);
        prev_cen = centroids;
        H_adaptive(step) = coverage_quality(pos_adaptive, sd, t, params);
        trajectory(step, :, :) = pos_adaptive;
        boundary_distances(step) = sd.boundary_distance;
        monitored_fraction(step) = compute_monitoring_fraction(pos_adaptive, sd.boundary_points, params);
        plume_extent(step) = estimate_plume_extent(t, params);

        pos_static = static_coverage(pos_static, t, params);
        H_static(step) = coverage_quality(pos_static, [], t, params);
        pos_random = random_coverage(pos_random, t, params);
        H_random(step) = coverage_quality(pos_random, [], t, params);
    end

    domain = params.domain;
    in_domain = all(pos_adaptive(:,1) >= domain.xmin & pos_adaptive(:,1) <= domain.xmax) && ...
                all(pos_adaptive(:,2) >= domain.ymin & pos_adaptive(:,2) <= domain.ymax) && ...
                all(pos_adaptive(:,3) >= domain.zmin & pos_adaptive(:,3) <= domain.zmax);
    assert(in_domain, '智能体终点应在仿真域内');
    assert(all(isfinite(boundary_distances)), '边界距离应为有限值');
    fprintf('  [PASS] 快速仿真完成，平均边界距离 %.2f m\n', mean(boundary_distances));
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试4: 可视化与结果分析 ==========
fprintf('\n--- 测试4: 可视化与结果分析 ---\n');
try
    save_dir = fullfile(fileparts(mfilename('fullpath')), '..', '测试结果', 'Step8_动态扩散边界追踪');
    if ~exist(save_dir, 'dir'), mkdir(save_dir); end

    plot_results(H_adaptive, H_static, H_random, params);
    exportgraphics(gcf, fullfile(save_dir, 'step8_coverage_quality.png'), 'Resolution', 150);
    %close(gcf);

    plume_state = update_plume(steps * params.algorithm.dt, params);
    plot_plume_agents_3d(pos_adaptive, trajectory, plume_state, params);
    exportgraphics(gcf, fullfile(save_dir, 'step8_plume_agents_tracking.png'), 'Resolution', 150);
    %close(gcf);

    plot_voronoi_3d(pos_adaptive, steps * params.algorithm.dt, params);
    exportgraphics(gcf, fullfile(save_dir, 'step8_voronoi_3d.png'), 'Resolution', 150);
    %close(gcf);

    plot_boundary_tracking(boundary_distances, plume_extent, params);
    exportgraphics(gcf, fullfile(save_dir, 'step8_boundary_tracking.png'), 'Resolution', 150);
    %close(gcf);

    write_step8_analysis(save_dir, H_adaptive, H_static, H_random, boundary_distances, plume_extent, monitored_fraction, params);
    assert(exist(fullfile(save_dir, '结果分析.md'), 'file') == 2, '结果分析.md 未生成');
    fprintf('  [PASS] Step8图片和结果分析已保存\n');
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 总结 ==========
fprintf('\n========================================\n');
fprintf('  Step 8 测试结果: %d PASS, %d FAIL\n', pass_count, fail_count);
if fail_count == 0
    fprintf('  >>> Step 8 全部通过！动态扩散边界追踪模块完成！ <<<\n');
else
    fprintf('  >>> 存在失败项，请检查后重试 <<<\n');
end
fprintf('========================================\n');
end
