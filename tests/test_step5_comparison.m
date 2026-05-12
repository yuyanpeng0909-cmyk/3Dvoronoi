function test_step5_comparison()
% test_step5_comparison - Step 5 测试：对比方法
% 运行方式: cd 到项目根目录, addpath(genpath('.')), test_step5_comparison

fprintf('========================================\n');
fprintf('  Step 5 测试：对比方法\n');
fprintf('========================================\n\n');

pass_count = 0;
fail_count = 0;

params = init_parameters();
params.algorithm.sample_num = 10000;  % 加速测试

%% ========== 测试1: static_coverage 位置不变 ==========
fprintf('--- 测试1: static_coverage 位置不变 ---\n');
try
    rng(42);
    init_pos = init_agents(params);
    for step = 1:10
        pos = static_coverage(init_pos, step, params);
    end
    assert(isequal(pos, init_pos), '静态覆盖位置应不变');
    fprintf('  [PASS] 10步后位置完全不变\n');
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试2: random_coverage 位置在域内 ==========
fprintf('\n--- 测试2: random_coverage 位置在域内 ---\n');
try
    rng(42);
    pos = init_agents(params);
    domain = params.domain;

    for step = 1:50
        pos = random_coverage(pos, step, params);
    end

    in_domain = all(pos(:,1) >= domain.xmin & pos(:,1) <= domain.xmax) & ...
                all(pos(:,2) >= domain.ymin & pos(:,2) <= domain.ymax) & ...
                all(pos(:,3) >= domain.zmin & pos(:,3) <= domain.zmax);
    assert(in_domain, '随机游走后部分位置超出域');

    fprintf('  [PASS] 50步随机游走后仍在域内\n');
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试3: 随机游走位移合理 ==========
fprintf('\n--- 测试3: 随机游走位移合理 ---\n');
try
    rng(42);
    pos = init_agents(params);
    pos_init = pos;

    n_steps = 100;
    for step = 1:n_steps
        pos = random_coverage(pos, step, params);
    end

    % 平均位移（应远小于 v_max * dt * n_steps = 2*1*100 = 200m）
    displacements = sqrt(sum((pos - pos_init).^2, 2));
    mean_disp = mean(displacements);
    max_possible = params.agent.max_speed * params.algorithm.dt * n_steps;

    assert(mean_disp < max_possible, '平均位移不应超过理论最大值');
    assert(mean_disp > 0, '随机游走应有非零位移');

    fprintf('  [PASS] 平均位移=%.1f m (理论最大=%.0f m)\n', mean_disp, max_possible);
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试4: 三种方法50步对比 ==========
fprintf('\n--- 测试4: 三种方法50步 H(t) 对比 ---\n');
try
    rng(42);
    pos_adaptive = init_agents(params);
    pos_static = init_agents_uniform(params);
    pos_random = pos_adaptive;

    n_steps = 50;
    H_adaptive = zeros(1, n_steps);
    H_static = zeros(1, n_steps);
    H_random = zeros(1, n_steps);

    prev_cen = [];

    for step = 1:n_steps
        t = step * params.algorithm.dt;

        % 自适应覆盖
        [pos_adaptive, centroids, ~, sd] = lloyd_iteration(pos_adaptive, t, prev_cen, params);
        prev_cen = centroids;
        H_adaptive(step) = coverage_quality(pos_adaptive, sd, t, params);

        % 静态覆盖
        pos_static = static_coverage(pos_static, t, params);
        H_static(step) = coverage_quality(pos_static, [], t, params);

        % 随机覆盖
        pos_random = random_coverage(pos_random, t, params);
        H_random(step) = coverage_quality(pos_random, [], t, params);
    end

    % 验证趋势
    H_adapt_end = mean(H_adaptive(end-4:end));
    H_stat_end = mean(H_static(end-4:end));
    H_rand_end = mean(H_random(end-4:end));

    fprintf('  自适应 H(end)=%.4e\n', H_adapt_end);
    fprintf('  静态   H(end)=%.4e\n', H_stat_end);
    fprintf('  随机   H(end)=%.4e\n', H_rand_end);

    % 自适应应优于随机
    assert(H_adapt_end < H_rand_end, ...
        '自适应覆盖应优于随机覆盖');

    fprintf('  [PASS] 自适应 < 随机 (自适应覆盖更优)\n');
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试5: 三种方法对比可视化 ==========
fprintf('\n--- 测试5: 三种方法对比可视化 ---\n');
try
    figure('Name', 'Step 5 - 三种方法对比', 'Position', [50 50 1200 500]);

    % 重新运行50步，记录完整数据
    rng(42);
    pos_a = init_agents(params);
    pos_s = init_agents_uniform(params);
    pos_r = pos_a;
    n_steps = 50;
    H_a = zeros(1, n_steps);
    H_s = zeros(1, n_steps);
    H_r = zeros(1, n_steps);
    traj_a = zeros(n_steps, params.agent.num, 3);
    prev_cen = [];

    for step = 1:n_steps
        t = step * params.algorithm.dt;
        [pos_a, centroids, ~, sd] = lloyd_iteration(pos_a, t, prev_cen, params);
        prev_cen = centroids;
        H_a(step) = coverage_quality(pos_a, sd, t, params);
        traj_a(step, :, :) = pos_a;

        pos_s = static_coverage(pos_s, t, params);
        H_s(step) = coverage_quality(pos_s, [], t, params);

        pos_r = random_coverage(pos_r, t, params);
        H_r(step) = coverage_quality(pos_r, [], t, params);
    end

    t_axis = (1:n_steps) * params.algorithm.dt;

    % 子图1: H(t) 三种方法对比
    subplot(1,3,1);
    plot(t_axis, H_s, 'r--', 'LineWidth', 2); hold on;
    plot(t_axis, H_r, 'g:', 'LineWidth', 2);
    plot(t_axis, H_a, 'b-', 'LineWidth', 2.5);
    % 趋势拟合
    if n_steps > 6
        p = polyfit(t_axis, H_a, min(5, n_steps-1));
        plot(t_axis, polyval(p, t_axis), 'b-.', 'LineWidth', 1, 'Color', [0.5 0.5 1]);
    end
    hold off;
    xlabel('时间 (s)'); ylabel('覆盖质量 H(t)');
    title('三种方法覆盖质量对比');
    legend('静态覆盖', '随机覆盖', '自适应覆盖', '自适应趋势', 'Location', 'best');
    grid on;

    % 子图2: 自适应覆盖的智能体轨迹
    subplot(1,3,2);
    src = params.plume.source_pos;
    hold on;
    colors = lines(params.agent.num);
    for i = 1:params.agent.num
        traj_i = squeeze(traj_a(:, i, :));
        plot3(traj_i(:,1), traj_i(:,2), traj_i(:,3), '-', 'Color', colors(i,:), 'LineWidth', 1.5);
        scatter3(traj_i(1,1), traj_i(1,2), traj_i(1,3), 50, colors(i,:), 'o', 'filled');
        scatter3(traj_i(end,1), traj_i(end,2), traj_i(end,3), 70, colors(i,:), 's', 'filled', ...
                 'MarkerEdgeColor', 'k');
    end
    scatter3(src(1), src(2), src(3), 200, [1 0.5 0], 'p', 'filled', 'MarkerEdgeColor', 'k');
    hold off;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('自适应覆盖 — 智能体轨迹');
    axis equal; view(3); grid on;

    % 子图3: 三种方法最终位置对比
    subplot(1,3,3);
    hold on;
    scatter3(pos_a(:,1), pos_a(:,2), pos_a(:,3), 120, 'b', 's', 'filled', ...
             'MarkerEdgeColor', 'k', 'DisplayName', '自适应');
    scatter3(pos_s(:,1), pos_s(:,2), pos_s(:,3), 100, 'r', '^', 'filled', ...
             'MarkerEdgeColor', 'k', 'DisplayName', '静态');
    scatter3(pos_r(:,1), pos_r(:,2), pos_r(:,3), 100, 'g', 'd', 'filled', ...
             'MarkerEdgeColor', 'k', 'DisplayName', '随机');
    scatter3(src(1), src(2), src(3), 200, [1 0.5 0], 'p', 'filled', ...
             'MarkerEdgeColor', 'k', 'DisplayName', '溢油源');
    hold off;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('三种方法最终智能体位置');
    legend('Location', 'best'); axis equal; view(3); grid on;

    % 保存图片
    save_dir = fullfile(fileparts(mfilename('fullpath')), '..', '测试结果', 'Step5_对比方法');
    if ~exist(save_dir, 'dir'), mkdir(save_dir); end
    exportgraphics(gcf, fullfile(save_dir, 'step5_comparison_visualization.png'), 'Resolution', 150);
    close(gcf);

    fprintf('  [PASS] 三种方法对比可视化已生成并保存\n');
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 总结 ==========
fprintf('\n========================================\n');
fprintf('  Step 5 测试结果: %d PASS, %d FAIL\n', pass_count, fail_count);
if fail_count == 0
    fprintf('  >>> Step 5 全部通过！可以进入 Step 6 <<<\n');
else
    fprintf('  >>> 存在失败项，请检查后重试 <<<\n');
end
fprintf('========================================\n');
end
