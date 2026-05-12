function test_step6_main()
% test_step6_main - Step 6 测试：主仿真脚本集成（快速验证版）
% 运行方式: cd 到项目根目录, addpath(genpath('.')), test_step6_main

fprintf('========================================\n');
fprintf('  Step 6 测试：主仿真脚本集成\n');
fprintf('========================================\n\n');

pass_count = 0;
fail_count = 0;

% 缩短仿真参数
params = init_parameters();
params.sim.total_time = 20;           % 仅20步
params.algorithm.sample_num = 5000;   % 较少采样点
params.algorithm.ff_gain = 0.3;

%% ========== 运行完整仿真流程 ==========
fprintf('--- 运行20步快速仿真 ---\n');
try
    rng(42);
    init_pos_adaptive = init_agents(params);
    init_pos_static   = init_agents_uniform(params);
    pos_adaptive = init_pos_adaptive;
    pos_static   = init_pos_static;
    pos_random   = init_pos_adaptive;

    steps = params.sim.total_time;
    H_adaptive = zeros(1, steps);
    H_static   = zeros(1, steps);
    H_random   = zeros(1, steps);
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

    fprintf('  仿真完成，无报错\n');
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] 仿真报错: %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试2: 输出维度正确 ==========
fprintf('\n--- 测试2: 输出维度正确 ---\n');
try
    assert(length(H_adaptive) == steps, 'H_adaptive 长度错误');
    assert(length(H_static) == steps, 'H_static 长度错误');
    assert(length(H_random) == steps, 'H_random 长度错误');
    assert(isequal(size(trajectory), [steps, params.agent.num, 3]), 'trajectory 维度错误');

    fprintf('  [PASS] H维度 [%d], trajectory维度 [%dx%dx3]\n', ...
        steps, steps, params.agent.num);
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试3: H_adaptive 总体下降 ==========
fprintf('\n--- 测试3: H_adaptive 总体下降 ---\n');
try
    H_start = mean(H_adaptive(1:3));
    H_end = mean(H_adaptive(end-2:end));

    assert(H_end < H_start, ...
        sprintf('H应下降: start=%.2e, end=%.2e', H_start, H_end));

    decrease_pct = (H_start - H_end) / H_start * 100;
    fprintf('  [PASS] H: %.2e → %.2e (下降 %.1f%%)\n', H_start, H_end, decrease_pct);
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试4: 轨迹记录完整 ==========
fprintf('\n--- 测试4: 轨迹记录完整 ---\n');
try
    % 检查第1步和最后一步轨迹非零
    assert(any(trajectory(1,:,:,:) ~= 0, 'all'), '第1步轨迹应为非零');
    assert(any(trajectory(end,:,:,:) ~= 0, 'all'), '最后一步轨迹应为非零');

    % 检查位置在域内
    final_pos = reshape(trajectory(end,:,:), [params.agent.num, 3]);
    domain = params.domain;
    in_domain = all(final_pos(:,1) >= domain.xmin & final_pos(:,1) <= domain.xmax) & ...
                all(final_pos(:,2) >= domain.ymin & final_pos(:,2) <= domain.ymax) & ...
                all(final_pos(:,3) >= domain.zmin & final_pos(:,3) <= domain.zmax);
    assert(in_domain, '轨迹终点超出域边界');

    fprintf('  [PASS] 轨迹完整，终点在域内\n');
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试5: 三种方法H排序合理 ==========
fprintf('\n--- 测试5: 三种方法 H(end) 排序 ---\n');
try
    H_a = mean(H_adaptive(end-4:end));
    H_s = mean(H_static(end-4:end));
    H_r = mean(H_random(end-4:end));

    fprintf('  H_adaptive=%.2e\n', H_a);
    fprintf('  H_static  =%.2e\n', H_s);
    fprintf('  H_random  =%.2e\n', H_r);

    assert(H_a < H_r, '自适应应优于随机');
    fprintf('  [PASS] 自适应 < 随机\n');
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试6: 主仿真结果可视化 ==========
fprintf('\n--- 测试6: 主仿真结果可视化 ---\n');
try
    figure('Name', 'Step 6 - 主仿真结果', 'Position', [50 50 1200 500]);
    src = params.plume.source_pos;
    t_axis = (1:steps) * params.algorithm.dt;

    % 子图1: 三种方法 H(t) 对比
    subplot(1,3,1);
    plot(t_axis, H_static, 'r--', 'LineWidth', 2); hold on;
    plot(t_axis, H_random, 'g:', 'LineWidth', 2);
    plot(t_axis, H_adaptive, 'b-', 'LineWidth', 2.5);
    if steps > 6
        p = polyfit(t_axis, H_adaptive, min(5, steps-1));
        plot(t_axis, polyval(p, t_axis), 'b-.', 'LineWidth', 1, 'Color', [0.5 0.5 1]);
    end
    hold off;
    xlabel('时间 (s)'); ylabel('覆盖质量 H(t)');
    title('主仿真 — 覆盖质量对比');
    legend('静态', '随机', '自适应', '趋势拟合', 'Location', 'best');
    grid on;

    % 子图2: 自适应轨迹 3D
    subplot(1,3,2);
    hold on;
    colors = lines(params.agent.num);
    for i = 1:params.agent.num
        traj_i = squeeze(trajectory(:, i, :));
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

    % 子图3: H 下降百分比柱状图
    subplot(1,3,3);
    H_a_drop = (mean(H_adaptive(1:min(3,steps))) - mean(H_adaptive(end-min(2,steps-1):end))) ...
               / mean(H_adaptive(1:min(3,steps))) * 100;
    H_s_change = abs(mean(H_static(end-min(2,steps-1):end)) - mean(H_static(1:min(3,steps)))) ...
                 / mean(H_static(1:min(3,steps))) * 100;
    H_r_change = abs(mean(H_random(end-min(2,steps-1):end)) - mean(H_random(1:min(3,steps)))) ...
                 / mean(H_random(1:min(3,steps))) * 100;
    bar([H_a_drop, H_s_change, H_r_change]);
    set(gca, 'XTickLabel', {'自适应', '静态', '随机'});
    ylabel('H 变化率 (%)');
    title(sprintf('覆盖质量变化 (%d步)', steps));
    grid on;

    % 保存图片
    save_dir = fullfile(fileparts(mfilename('fullpath')), '..', '测试结果', 'Step6_主仿真集成');
    if ~exist(save_dir, 'dir'), mkdir(save_dir); end
    exportgraphics(gcf, fullfile(save_dir, 'step6_main_visualization.png'), 'Resolution', 150);
    close(gcf);

    fprintf('  [PASS] 主仿真可视化已生成并保存\n');
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 总结 ==========
fprintf('\n========================================\n');
fprintf('  Step 6 测试结果: %d PASS, %d FAIL\n', pass_count, fail_count);
if fail_count == 0
    fprintf('  >>> Step 6 全部通过！可以进入 Step 7 <<<\n');
else
    fprintf('  >>> 存在失败项，请检查后重试 <<<\n');
end
fprintf('========================================\n');
end
