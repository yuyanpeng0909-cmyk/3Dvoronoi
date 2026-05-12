function test_step7_visualization()
% test_step7_visualization - Step 7 测试：可视化模块
% 运行方式: cd 到项目根目录, addpath(genpath('.')), test_step7_visualization

fprintf('========================================\n');
fprintf('  Step 7 测试：可视化模块\n');
fprintf('========================================\n\n');

pass_count = 0;
fail_count = 0;

% 仿真生成测试数据（200步充分收敛，重要性采样加速质心估计）
params = init_parameters();
params.sim.total_time = 200;          % 200步充分收敛
params.algorithm.sample_num = 20000;  % 采样点数（重要性采样下效率更高）

rng(42);
pos_adaptive = init_agents(params);
pos_static = init_agents_uniform(params);
pos_random = pos_adaptive;

steps = params.sim.total_time;
H_adaptive = zeros(1, steps);
H_static = zeros(1, steps);
H_random = zeros(1, steps);
trajectory = zeros(steps, params.agent.num, 3);
prev_cen = [];

for step = 1:steps
    t = step * params.algorithm.dt;
    [pos_adaptive, centroids, ~, sd] = lloyd_iteration(pos_adaptive, t, prev_cen, params);
    prev_cen = centroids;
    H_adaptive(step) = coverage_quality(pos_adaptive, sd, t, params);
    trajectory(step, :, :) = pos_adaptive;

    pos_static = static_coverage(pos_static, t, params);
    H_static(step) = coverage_quality(pos_static, [], t, params);

    pos_random = random_coverage(pos_random, t, params);
    H_random(step) = coverage_quality(pos_random, [], t, params);
end

% 保存目录
save_dir = fullfile(fileparts(mfilename('fullpath')), '..', '测试结果', 'Step7_可视化');
if ~exist(save_dir, 'dir'), mkdir(save_dir); end

%% ========== 测试1: plot_results ==========
fprintf('--- 测试1: plot_results 覆盖质量对比 ---\n');
try
    plot_results(H_adaptive, H_static, H_random, params);
    exportgraphics(gcf, fullfile(save_dir, 'step7_coverage_quality.png'), 'Resolution', 150);
    close(gcf);
    fprintf('  [PASS] 覆盖质量对比图已保存\n');
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试2: plot_plume_agents_3d ==========
fprintf('\n--- 测试2: plot_plume_agents_3d 三维可视化 ---\n');
try
    t_final = steps * params.algorithm.dt;
    plume_state = update_plume(t_final, params);
    plot_plume_agents_3d(pos_adaptive, trajectory, plume_state, params);
    exportgraphics(gcf, fullfile(save_dir, 'step7_plume_agents_3d.png'), 'Resolution', 150);
    %close(gcf);
    fprintf('  [PASS] 三维羽流+智能体图已保存\n');
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试3: plot_voronoi_3d ==========
fprintf('\n--- 测试3: plot_voronoi_3d Voronoi区域 ---\n');
try
    plot_voronoi_3d(pos_adaptive, t_final, params);
    exportgraphics(gcf, fullfile(save_dir, 'step7_voronoi_3d.png'), 'Resolution', 150);
    %close(gcf);
    fprintf('  [PASS] Voronoi区域图已保存\n');
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 总结 ==========
fprintf('\n========================================\n');
fprintf('  Step 7 测试结果: %d PASS, %d FAIL\n', pass_count, fail_count);
if fail_count == 0
    fprintf('  >>> Step 7 全部通过！所有模块完成！ <<<\n');
else
    fprintf('  >>> 存在失败项，请检查后重试 <<<\n');
end
fprintf('========================================\n');
end
