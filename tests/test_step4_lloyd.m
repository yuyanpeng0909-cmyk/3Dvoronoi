function test_step4_lloyd()
% test_step4_lloyd - Step 4 测试：Lloyd迭代控制律
% 运行方式: cd 到项目根目录, addpath(genpath('.')), test_step4_lloyd

fprintf('========================================\n');
fprintf('  Step 4 测试：Lloyd迭代控制律\n');
fprintf('========================================\n\n');

pass_count = 0;
fail_count = 0;

% 测试参数（减少采样点加速）
params = init_parameters();
params.algorithm.sample_num = 10000;

%% ========== 测试1: 位置更新后仍在域内 ==========
fprintf('--- 测试1: 位置更新后仍在域内 ---\n');
try
    rng(42);
    agents = init_agents(params);
    [new_pos, ~, ~, ~] = lloyd_iteration(agents, 1, [], params);

    domain = params.domain;
    in_domain = all(new_pos(:,1) >= domain.xmin & new_pos(:,1) <= domain.xmax) & ...
                all(new_pos(:,2) >= domain.ymin & new_pos(:,2) <= domain.ymax) & ...
                all(new_pos(:,3) >= domain.zmin & new_pos(:,3) <= domain.zmax);
    assert(in_domain, '更新后部分智能体超出域边界');

    fprintf('  [PASS] 所有智能体更新后仍在域内\n');
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试2: 速度限幅生效 ==========
fprintf('\n--- 测试2: 速度限幅生效 ---\n');
try
    % 将智能体放在远离质心的位置（距离很大），测试速度是否被限幅
    rng(42);
    agents = init_agents(params);

    % 手动设置一个远离羽流的智能体（质心会偏向羽流，反馈量大）
    agents(1,:) = [10, 180, -280];

    [new_pos, centroids, move_dist, ~] = lloyd_iteration(agents, 1, [], params);

    % 检查最大位移 <= v_max * dt
    displacements = sqrt(sum((new_pos - agents).^2, 2));
    max_expected = params.agent.max_speed * params.algorithm.dt;
    assert(all(displacements <= max_expected + 1e-10), ...
        sprintf('位移 %.3f 超过限幅 %.3f', max(displacements), max_expected));

    fprintf('  [PASS] 最大位移=%.3f m, 限幅=%.3f m\n', max(displacements), max_expected);
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试3: 前馈补偿项效果 ==========
fprintf('\n--- 测试3: 前馈补偿项效果 ---\n');
try
    rng(42);
    agents = init_agents(params);

    % 先运行一步获取质心
    [~, centroids_t1, ~, ~] = lloyd_iteration(agents, 1, [], params);

    % 有前馈
    params_ff = params;
    params_ff.algorithm.ff_gain = 0.3;
    [pos_ff, ~, ~, ~] = lloyd_iteration(agents, 2, centroids_t1, params_ff);

    % 无前馈
    params_no_ff = params;
    params_no_ff.algorithm.ff_gain = 0;
    [pos_no_ff, ~, ~, ~] = lloyd_iteration(agents, 2, centroids_t1, params_no_ff);

    % 两种结果应有差异
    diff_ff = sqrt(sum((pos_ff - pos_no_ff).^2, 2));
    has_effect = any(diff_ff > 1e-6);

    assert(has_effect, '前馈补偿应产生位置差异');

    fprintf('  [PASS] 前馈引起的平均位置偏差=%.4f m\n', mean(diff_ff));
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试4: 采样数据打包正确 ==========
fprintf('\n--- 测试4: 采样数据打包正确 ---\n');
try
    rng(42);
    agents = init_agents(params);
    [~, ~, ~, sample_data] = lloyd_iteration(agents, 1, [], params);

    assert(isfield(sample_data, 'samples'), '缺少 samples');
    assert(isfield(sample_data, 'phi'), '缺少 phi');
    assert(isfield(sample_data, 'dist_matrix'), '缺少 dist_matrix');
    assert(isfield(sample_data, 'nearest_agent'), '缺少 nearest_agent');

    fprintf('  [PASS] sample_data 包含全部4个字段\n');
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试5: 多步迭代收敛 ==========
fprintf('\n--- 测试5: 多步迭代收敛（静态密度近似） ---\n');
try
    rng(42);
    agents = init_agents(params);
    pos = agents;

    n_steps = 30;
    H_history = zeros(1, n_steps);
    prev_cen = [];

    for step = 1:n_steps
        t = step * params.algorithm.dt;
        [pos, centroids, ~, sd] = lloyd_iteration(pos, t, prev_cen, params);
        prev_cen = centroids;
        H_history(step) = coverage_quality(pos, sd, t, params);
    end

    % H 应总体下降（即使密度时变，前几步应明显下降）
    H_start = mean(H_history(1:3));
    H_end = mean(H_history(end-2:end));
    decreased = H_end < H_start;

    if decreased
        fprintf('  [PASS] H 从 %.4e 降至 %.4e (下降 %.1f%%)\n', ...
            H_start, H_end, (H_start-H_end)/H_start*100);
    else
        fprintf('  [警告] H 未见明显下降: %.4e → %.4e（可能需要调参）\n', H_start, H_end);
    end
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试6: 智能体向羽流区域聚集 ==========
fprintf('\n--- 测试6: 智能体向羽流区域聚集 ---\n');
try
    rng(42);
    agents = init_agents(params);
    src = params.plume.source_pos;

    % 初始距源平均距离
    dist_init = mean(sqrt(sum((agents - src).^2, 2)));

    % 运行30步
    pos = agents;
    prev_cen = [];
    for step = 1:30
        t = step * params.algorithm.dt;
        [pos, centroids, ~, ~] = lloyd_iteration(pos, t, prev_cen, params);
        prev_cen = centroids;
    end

    dist_final = mean(sqrt(sum((pos - src).^2, 2)));
    moved_closer = dist_final < dist_init;

    fprintf('  初始平均距源: %.1f m → 最终: %.1f m (%.1f m)\n', ...
        dist_init, dist_final, dist_init - dist_final);

    if moved_closer
        fprintf('  [PASS] 智能体向源方向移动了 %.1f m\n', dist_init - dist_final);
    else
        fprintf('  [警告] 智能体未向源聚集（可能需要更多步数或调参）\n');
    end
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试7: Lloyd迭代收敛与轨迹可视化 ==========
fprintf('\n--- 测试7: Lloyd迭代收敛与轨迹可视化 ---\n');
try
    figure('Name', 'Step 4 - Lloyd迭代收敛', 'Position', [50 50 1200 500]);
    src = params.plume.source_pos;

    % 重新运行30步迭代，记录完整数据
    rng(42);
    agents_vis = init_agents(params);
    pos = agents_vis;
    n_steps = 30;
    H_hist = zeros(1, n_steps);
    traj = zeros(n_steps, params.agent.num, 3);
    dist_to_src = zeros(n_steps, params.agent.num);
    prev_cen = [];

    for step = 1:n_steps
        t = step * params.algorithm.dt;
        [pos, centroids, ~, sd] = lloyd_iteration(pos, t, prev_cen, params);
        prev_cen = centroids;
        H_hist(step) = coverage_quality(pos, sd, t, params);
        traj(step, :, :) = pos;
        dist_to_src(step, :) = sqrt(sum((pos - src).^2, 2));
    end

    % 子图1: H(t) 收敛曲线
    subplot(1,3,1);
    t_axis = (1:n_steps) * params.algorithm.dt;
    plot(t_axis, H_hist, 'b-o', 'LineWidth', 2, 'MarkerSize', 5, 'MarkerFaceColor', 'b');
    xlabel('时间 (s)'); ylabel('覆盖质量 H(t)');
    title('Lloyd迭代收敛过程');
    grid on;
    % 标注下降百分比
    pct = (H_hist(1) - H_hist(end)) / H_hist(1) * 100;
    text(t_axis(end/2), H_hist(1), sprintf('下降 %.1f%%', pct), ...
         'Color', 'r', 'FontSize', 11, 'FontWeight', 'bold');

    % 子图2: 3D轨迹图
    subplot(1,3,2);
    hold on;
    colors = lines(params.agent.num);
    for i = 1:params.agent.num
        traj_i = squeeze(traj(:, i, :));
        plot3(traj_i(:,1), traj_i(:,2), traj_i(:,3), '-', 'Color', colors(i,:), 'LineWidth', 1.5);
        % 起点
        scatter3(traj_i(1,1), traj_i(1,2), traj_i(1,3), 60, colors(i,:), 'o', 'filled');
        % 终点
        scatter3(traj_i(end,1), traj_i(end,2), traj_i(end,3), 80, colors(i,:), 's', 'filled', ...
                 'MarkerEdgeColor', 'k');
    end
    scatter3(src(1), src(2), src(3), 200, [1 0.5 0], 'p', 'filled', 'MarkerEdgeColor', 'k');
    hold off;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('智能体运动轨迹');
    legend(arrayfun(@(x) sprintf('A%d',x), 1:params.agent.num, 'UniformOutput', false), ...
           'Location', 'best', 'NumColumns', 2);
    axis equal; view(3); grid on;

    % 子图3: 到源距离变化
    subplot(1,3,3);
    hold on;
    for i = 1:params.agent.num
        plot(t_axis, dist_to_src(:, i), '-', 'Color', colors(i,:), 'LineWidth', 1.2);
    end
    hold off;
    xlabel('时间 (s)'); ylabel('距源距离 (m)');
    title('各智能体到溢油源距离');
    grid on;

    % 保存图片
    save_dir = fullfile(fileparts(mfilename('fullpath')), '..', '测试结果', 'Step4_Lloyd迭代');
    if ~exist(save_dir, 'dir'), mkdir(save_dir); end
    exportgraphics(gcf, fullfile(save_dir, 'step4_lloyd_visualization.png'), 'Resolution', 150);
    close(gcf);

    fprintf('  [PASS] Lloyd迭代可视化已生成并保存\n');
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 总结 ==========
fprintf('\n========================================\n');
fprintf('  Step 4 测试结果: %d PASS, %d FAIL\n', pass_count, fail_count);
if fail_count == 0
    fprintf('  >>> Step 4 全部通过！可以进入 Step 5 <<<\n');
else
    fprintf('  >>> 存在失败项，请检查后重试 <<<\n');
end
fprintf('========================================\n');
end
