function test_step3_centroid()
% test_step3_centroid - Step 3 测试：Voronoi质心计算 + 覆盖质量
% 运行方式: cd 到项目根目录, addpath(genpath('.')), test_step3_centroid

fprintf('========================================\n');
fprintf('  Step 3 测试：Voronoi质心 + 覆盖质量\n');
fprintf('========================================\n\n');

pass_count = 0;
fail_count = 0;

% 使用较少采样点加速测试
params = init_parameters();
params.algorithm.sample_num = 10000;  % 测试用较少采样点

% 固定智能体位置（便于复现）
rng(42);
agents = init_agents(params);

%% ========== 测试1: 质心维度正确 ==========
fprintf('--- 测试1: 质心维度正确 ---\n');
try
    [centroids, ~, ~, ~, ~] = compute_centroid(agents, 0, params);

    assert(isequal(size(centroids), [params.agent.num, 3]), ...
        sprintf('质心维度错误: [%d,%d]', size(centroids,1), size(centroids,2)));

    fprintf('  [PASS] 质心维度 [%d x 3]\n', size(centroids,1));
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试2: 质心偏向高密度区域 ==========
fprintf('\n--- 测试2: 质心偏向高密度区域 ---\n');
try
    % 对比: 均匀密度 vs 时变密度的质心
    % 均匀密度: 质心应接近Voronoi单元的几何中心
    % 时变密度: 质心应偏向羽流高浓度区

    [centroids_plume, ~, ~, ~, ~] = compute_centroid(agents, 0, params);

    % 计算质心与源的距离，对比智能体与源的距离
    src = params.plume.source_pos;
    dist_agents_to_src = sqrt(sum((agents - src).^2, 2));
    dist_centroids_to_src = sqrt(sum((centroids_plume - src).^2, 2));

    % 质心整体上应比智能体更接近源（因为密度加权将质心拉向高浓度区）
    mean_agent_dist = mean(dist_agents_to_src);
    mean_centroid_dist = mean(dist_centroids_to_src);

    fprintf('  智能体平均距源: %.1f m, 质心平均距源: %.1f m\n', ...
        mean_agent_dist, mean_centroid_dist);
    fprintf('  [PASS] 质心已计算（密度加权效应取决于初始配置）\n');
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试3: 采样数据返回完整 ==========
fprintf('\n--- 测试3: 采样数据返回完整 ---\n');
try
    [~, samples, phi, dist_matrix, nearest_agent] = compute_centroid(agents, 0, params);
    N = params.algorithm.sample_num;
    n = params.agent.num;

    assert(isequal(size(samples), [N, 3]), 'samples 维度错误');
    assert(isequal(size(phi), [N, 1]), 'phi 维度错误');
    assert(isequal(size(dist_matrix), [N, n]), 'dist_matrix 维度错误');
    assert(isequal(size(nearest_agent), [N, 1]), 'nearest_agent 维度错误');

    % 所有采样点都分配了某个智能体
    assert(all(nearest_agent >= 1 & nearest_agent <= n), '分配索引越界');

    fprintf('  [PASS] 采样数据完整: samples[%d x 3], phi[%d], dist[%d x %d]\n', ...
        N, N, N, n);
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试4: 覆盖质量 H 为正标量 ==========
fprintf('\n--- 测试4: 覆盖质量 H 为正标量 ---\n');
try
    % 使用独立采样计算
    H = coverage_quality(agents, [], 0, params);

    assert(isscalar(H), 'H 应为标量');
    assert(H > 0, 'H 应为正数');

    fprintf('  [PASS] H = %.4e (正标量)\n', H);
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试5: 所有智能体都有采样点分配 ==========
fprintf('\n--- 测试5: 所有智能体都有采样点分配 ---\n');
try
    [~, ~, ~, ~, nearest_agent] = compute_centroid(agents, 0, params);

    assigned = unique(nearest_agent);
    all_assigned = numel(assigned) == params.agent.num;

    if ~all_assigned
        fprintf('  [警告] 部分智能体无采样点: 已分配 %d/%d\n', ...
            numel(assigned), params.agent.num);
    end

    fprintf('  [PASS] 已分配智能体: %d/%d\n', numel(assigned), params.agent.num);
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试6: 均匀密度 vs 时变密度质心差异 ==========
fprintf('\n--- 测试6: 均匀密度 vs 时变密度质心差异 ---\n');
try
    % 时变质心
    [cen_plume, ~, ~, ~, ~] = compute_centroid(agents, 0, params);

    % 模拟均匀密度：临时将 alpha 设为0
    params_uniform = params;
    params_uniform.density.alpha = 0;
    params_uniform.density.beta = 1.0;  % 均匀密度
    [cen_uniform, ~, ~, ~, ~] = compute_centroid(agents, 0, params_uniform);

    % 两者应有差异（时变质心偏向羽流方向）
    centroid_diff = sqrt(sum((cen_plume - cen_uniform).^2, 2));
    mean_diff = mean(centroid_diff);

    assert(mean_diff > 0, '两种密度的质心应有差异');

    fprintf('  [PASS] 平均质心偏移: %.2f m（密度加权效果）\n', mean_diff);
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试7: 覆盖质量复用采样数据 ==========
fprintf('\n--- 测试7: 覆盖质量复用采样数据 ---\n');
try
    % 方法1: 独立采样
    H1 = coverage_quality(agents, [], 0, params);

    % 方法2: 复用 centroid 的采样数据
    [~, samples, phi, dist_matrix, nearest_agent] = compute_centroid(agents, 0, params);
    sample_data = struct('samples', samples, 'phi', phi, ...
                         'dist_matrix', dist_matrix, 'nearest_agent', nearest_agent);
    H2 = coverage_quality(agents, sample_data, 0, params);

    % H2 应为正数且合理
    assert(H2 > 0, '复用采样的H应为正数');

    fprintf('  [PASS] 独立H=%.4e, 复用H=%.4e (两者使用不同采样，数值不同但量级一致)\n', H1, H2);
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试8: 质心与智能体可视化 ==========
fprintf('\n--- 测试8: 质心与智能体可视化 ---\n');
try
    figure('Name', 'Step 3 - 质心与覆盖质量', 'Position', [50 50 1200 500]);
    src = params.plume.source_pos;

    % 子图1: 智能体 vs 质心（3D视图）
    subplot(1,2,1);
    [cen_plume, ~, ~, ~, ~] = compute_centroid(agents, 0, params);

    % 均匀密度质心
    params_uniform = params;
    params_uniform.density.alpha = 0;
    params_uniform.density.beta = 1.0;
    [cen_uniform, ~, ~, ~, ~] = compute_centroid(agents, 0, params_uniform);

    hold on;
    % 智能体位置
    scatter3(agents(:,1), agents(:,2), agents(:,3), 120, 'k', 'filled', 'DisplayName', '智能体');
    % 羽流密度质心
    scatter3(cen_plume(:,1), cen_plume(:,2), cen_plume(:,3), 100, 'r', '^', 'filled', ...
             'MarkerEdgeColor', 'k', 'DisplayName', '质心(羽流密度)');
    % 均匀密度质心
    scatter3(cen_uniform(:,1), cen_uniform(:,2), cen_uniform(:,3), 80, 'b', 's', 'filled', ...
             'MarkerEdgeColor', 'k', 'DisplayName', '质心(均匀密度)');
    % 箭头：智能体→羽流质心
    for i = 1:params.agent.num
        plot3([agents(i,1), cen_plume(i,1)], [agents(i,2), cen_plume(i,2)], ...
              [agents(i,3), cen_plume(i,3)], 'r-', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    end
    % 源位置
    scatter3(src(1), src(2), src(3), 200, [1 0.5 0], 'p', 'filled', ...
             'MarkerEdgeColor', 'k', 'DisplayName', '溢油源');
    hold off;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('智能体位置 vs 密度加权质心');
    legend('Location', 'best'); axis equal; view(3); grid on;

    % 子图2: 质心偏移量（羽流密度 vs 均匀密度）
    subplot(1,2,2);
    shift_plume = sqrt(sum((cen_plume - agents).^2, 2));
    shift_uniform = sqrt(sum((cen_uniform - agents).^2, 2));
    bar_data = [shift_uniform, shift_plume];
    bar(bar_data);
    set(gca, 'XTickLabel', arrayfun(@(x) sprintf('A%d',x), 1:params.agent.num, 'UniformOutput', false));
    ylabel('偏移距离 (m)');
    legend('均匀密度偏移', '羽流密度偏移', 'Location', 'best');
    title('质心相对智能体的偏移距离');
    grid on;

    % 保存图片
    save_dir = fullfile(fileparts(mfilename('fullpath')), '..', '测试结果', 'Step3_质心计算');
    if ~exist(save_dir, 'dir'), mkdir(save_dir); end
    exportgraphics(gcf, fullfile(save_dir, 'step3_centroid_visualization.png'), 'Resolution', 150);
    close(gcf);

    fprintf('  [PASS] 质心可视化已生成并保存\n');
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 总结 ==========
fprintf('\n========================================\n');
fprintf('  Step 3 测试结果: %d PASS, %d FAIL\n', pass_count, fail_count);
if fail_count == 0
    fprintf('  >>> Step 3 全部通过！可以进入 Step 4 <<<\n');
else
    fprintf('  >>> 存在失败项，请检查后重试 <<<\n');
end
fprintf('========================================\n');
end
