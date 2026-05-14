function test_step8_dynamic_tracking()
% test_step8_dynamic_tracking - Step 8 测试：动态扩散边界追踪
% 运行方式: cd到项目根目录后显式addpath('.', 'coverage_control', 'comparison_methods', 'density_function', 'plume_model', 'visualization', 'tests')，再运行 test_step8_dynamic_tracking
%
% 对比三种方法: Proposed CVT-DBT / Standard CVT / Lawnmower CPP

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
    V0 = estimate_plume_extent(params.sim.pre_release_time, params);
    V1 = estimate_plume_extent(params.sim.pre_release_time + 120, params);
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
    [boundary_points, assigned_agent, boundary_distance] = sample_plume_boundary(pos, params.sim.pre_release_time + 60, params);
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

%% ========== 测试3: 三种方法对比仿真 ==========
fprintf('\n--- 测试3: 40步三种方法对比仿真 ---\n');
try
    rng(42);
    pos_proposed = init_agents(params);
    pos_cvt = pos_proposed;
    pos_lawnmower = lawnmower_coverage([], params.sim.pre_release_time, params);

    steps = params.sim.total_time;
    H_proposed = zeros(1, steps);
    H_cvt = zeros(1, steps);
    H_lawnmower = zeros(1, steps);
    trajectory = zeros(steps, params.agent.num, 3);
    boundary_distances = zeros(1, steps);
    plume_extent = zeros(1, steps);
    monitored_fraction = zeros(1, steps);
    cr_proposed = zeros(1, steps);
    cr_cvt = zeros(1, steps);
    cr_lawnmower = zeros(1, steps);
    rmse_proposed = zeros(1, steps);
    rmse_cvt = zeros(1, steps);
    rmse_lawnmower = zeros(1, steps);
    velocities = zeros(steps, params.agent.num);
    prev_cen = [];
    prev_pos = [];

    for step = 1:steps
        t_monitor = (step - 1) * params.algorithm.dt;
        t_leak = params.sim.pre_release_time + t_monitor;

        % Proposed CVT-DBT
        prev_pos = pos_proposed;
        [pos_proposed, centroids, ~, sd] = lloyd_iteration(pos_proposed, t_leak, prev_cen, params);
        prev_cen = centroids;
        H_proposed(step) = coverage_quality(pos_proposed, [], t_leak, params);
        trajectory(step, :, :) = pos_proposed;
        boundary_distances(step) = sd.boundary_distance;
        monitored_fraction(step) = compute_monitoring_fraction(pos_proposed, sd.boundary_points, params);

        % 记录速度
        if step == 1 || isempty(prev_pos)
            velocities(step, :) = 0;
        else
            velocities(step, :) = sqrt(sum((pos_proposed - prev_pos).^2, 2))' / params.algorithm.dt;
        end

        % Standard CVT
        pos_cvt = standard_cvt(pos_cvt, t_leak, params);
        H_cvt(step) = coverage_quality(pos_cvt, [], t_leak, params);

        % Lawnmower CPP
        pos_lawnmower = lawnmower_coverage(pos_lawnmower, t_leak, params);
        H_lawnmower(step) = coverage_quality(pos_lawnmower, [], t_leak, params);

        % 共享指标
        plume_state = update_plume(t_leak, params);
        plume_extent(step) = estimate_plume_extent(t_leak, params);

        cr_proposed(step) = compute_dynamic_coverage_ratio(pos_proposed, params, plume_state);
        cr_cvt(step) = compute_dynamic_coverage_ratio(pos_cvt, params, plume_state);
        cr_lawnmower(step) = compute_dynamic_coverage_ratio(pos_lawnmower, params, plume_state);

        rmse_proposed(step) = compute_boundary_rmse(pos_proposed, params, plume_state);
        rmse_cvt(step) = compute_boundary_rmse(pos_cvt, params, plume_state);
        rmse_lawnmower(step) = compute_boundary_rmse(pos_lawnmower, params, plume_state);
    end

    domain = params.domain;
    in_domain = all(pos_proposed(:,1) >= domain.xmin & pos_proposed(:,1) <= domain.xmax) && ...
                all(pos_proposed(:,2) >= domain.ymin & pos_proposed(:,2) <= domain.ymax) && ...
                all(pos_proposed(:,3) >= domain.zmin & pos_proposed(:,3) <= domain.zmax);
    assert(in_domain, '智能体终点应在仿真域内');
    assert(all(isfinite(boundary_distances)), '边界距离应为有限值');

    % 覆盖率范围检查
    assert(all(cr_proposed >= 0 & cr_proposed <= 1), '覆盖率应在[0,1]范围内');
    assert(all(cr_cvt >= 0 & cr_cvt <= 1), '覆盖率应在[0,1]范围内');
    assert(all(cr_lawnmower >= 0 & cr_lawnmower <= 1), '覆盖率应在[0,1]范围内');

    % Proposed方法覆盖率 >= Lawnmower覆盖率（最终步）
    assert(cr_proposed(end) >= cr_lawnmower(end), ...
        sprintf('Proposed覆盖率(%.1f%%)应>=Lawnmower(%.1f%%)', cr_proposed(end)*100, cr_lawnmower(end)*100));

    % Proposed方法RMSE < Standard CVT RMSE
    assert(rmse_proposed(end) < rmse_cvt(end), ...
        sprintf('Proposed RMSE(%.1f)应<CVT RMSE(%.1f)', rmse_proposed(end), rmse_cvt(end)));

    fprintf('  [PASS] 三种方法仿真完成\n');
    fprintf('    H: Proposed=%.2e, CVT=%.2e, Lawnmower=%.2e\n', H_proposed(end), H_cvt(end), H_lawnmower(end));
    fprintf('    CR: Proposed=%.1f%%, CVT=%.1f%%, Lawnmower=%.1f%%\n', cr_proposed(end)*100, cr_cvt(end)*100, cr_lawnmower(end)*100);
    fprintf('    RMSE: Proposed=%.1f, CVT=%.1f, Lawnmower=%.1f\n', rmse_proposed(end), rmse_cvt(end), rmse_lawnmower(end));
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

    plot_results(H_proposed, H_cvt, H_lawnmower, params);
    exportgraphics(gcf, fullfile(save_dir, 'step8_coverage_quality.png'), 'Resolution', 150);

    t_final_leak = params.sim.pre_release_time + (steps - 1) * params.algorithm.dt;

    plume_state = update_plume(t_final_leak, params);
    plot_plume_agents_3d(pos_proposed, trajectory, plume_state, params);
    exportgraphics(gcf, fullfile(save_dir, 'step8_plume_agents_tracking.png'), 'Resolution', 150);

    plot_voronoi_3d(pos_proposed, t_final_leak, params);
    exportgraphics(gcf, fullfile(save_dir, 'step8_voronoi_3d.png'), 'Resolution', 150);

    plot_coverage_regions_3d(pos_proposed, plume_state, params);
    exportgraphics(gcf, fullfile(save_dir, 'step8_coverage_regions_3d.png'), 'Resolution', 150);

    plot_boundary_tracking(boundary_distances, plume_extent, rmse_proposed, rmse_cvt, rmse_lawnmower, params);
    exportgraphics(gcf, fullfile(save_dir, 'step8_boundary_tracking.png'), 'Resolution', 150);

    plot_coverage_metrics(cr_proposed, cr_cvt, cr_lawnmower, rmse_proposed, rmse_cvt, rmse_lawnmower, params);
    exportgraphics(gcf, fullfile(save_dir, 'step8_coverage_metrics.png'), 'Resolution', 150);

    plot_control_input(velocities, params);
    exportgraphics(gcf, fullfile(save_dir, 'step8_control_input.png'), 'Resolution', 150);

    plot_monitoring_dashboard(H_proposed, H_cvt, H_lawnmower, ...
        cr_proposed, cr_cvt, cr_lawnmower, ...
        rmse_proposed, rmse_cvt, rmse_lawnmower, ...
        boundary_distances, plume_extent, monitored_fraction, trajectory, ...
        velocities, params);
    exportgraphics(gcf, fullfile(save_dir, 'step8_monitoring_dashboard.png'), 'Resolution', 150);

    write_step8_analysis(save_dir, H_proposed, H_cvt, H_lawnmower, ...
        boundary_distances, plume_extent, monitored_fraction, ...
        cr_proposed, cr_cvt, cr_lawnmower, ...
        rmse_proposed, rmse_cvt, rmse_lawnmower, velocities, params);
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
