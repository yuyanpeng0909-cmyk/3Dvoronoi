function test_step2_density()
% test_step2_density - Step 2 测试：密度函数 + 智能体初始化
% 运行方式: cd 到项目根目录, addpath(genpath('.')), test_step2_density

fprintf('========================================\n');
fprintf('  Step 2 测试：密度函数 + 智能体初始化\n');
fprintf('========================================\n\n');

pass_count = 0;
fail_count = 0;

% 先获取参数（Step 1 已验证通过）
params = init_parameters();

%% ========== 测试1: 密度函数处处为正 ==========
fprintf('--- 测试1: 密度函数处处为正 ---\n');
try
    % 在域内随机采样100个点
    rng(123);
    N_test = 100;
    x_test = params.domain.xmin + (params.domain.xmax - params.domain.xmin) * rand(N_test, 1);
    y_test = params.domain.ymin + (params.domain.ymax - params.domain.ymin) * rand(N_test, 1);
    z_test = params.domain.zmin + (params.domain.zmax - params.domain.zmin) * rand(N_test, 1);

    phi = compute_density(x_test, y_test, z_test, 0, params);

    assert(all(phi > 0), '密度函数应处处为正');
    assert(all(phi >= params.density.beta), ...
        sprintf('密度函数应 >= beta=%.4f, 最小值=%.6f', params.density.beta, min(phi)));

    fprintf('  [PASS] 密度最小值=%.4f (beta=%.4f), 密度最大值=%.4f\n', ...
        min(phi), params.density.beta, max(phi));
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试2: 密度函数在源附近最高 ==========
fprintf('\n--- 测试2: 密度函数在源附近最高 ---\n');
try
    src = params.plume.source_pos;

    % 源下游100m处
    phi_near = compute_density(src(1)+50, src(2), src(3), 0, params);
    % 远离源处
    phi_far = compute_density(400, 150, -250, 0, params);
    % 上游（浓度为零，只有基底密度）
    phi_upstream = compute_density(10, 0, -50, 0, params);

    assert(phi_near > phi_far, '源附近密度应高于远处');
    assert(phi_near > phi_upstream, '源附近密度应高于上游');

    fprintf('  [PASS] 源附近=%.4f, 远处=%.4f, 上游=%.4f\n', ...
        phi_near, phi_far, phi_upstream);
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试3: α·C_norm + β 公式正确性 ==========
fprintf('\n--- 测试3: 密度公式正确性 ---\n');
try
    src = params.plume.source_pos;
    x_t = src(1) + 100;

    % 手动计算
    C_raw = gaussian_plume_3d(x_t, 0, src(3), 0, params);
    C_norm_manual = min(C_raw / params.plume.C_max_estimate, 1);
    phi_manual = params.density.alpha * C_norm_manual + params.density.beta;

    % 函数计算
    phi_func = compute_density(x_t, 0, src(3), 0, params);

    assert(abs(phi_manual - phi_func) < 1e-12, '密度公式计算不一致');

    fprintf('  [PASS] 手动=%.6f, 函数=%.6f, 差值=%.2e\n', ...
        phi_manual, phi_func, abs(phi_manual - phi_func));
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试4: init_agents 随机位置在域内 ==========
fprintf('\n--- 测试4: init_agents 随机位置在域内 ---\n');
try
    rng(42);
    positions = init_agents(params);
    n = params.agent.num;

    assert(isequal(size(positions), [n, 3]), ...
        sprintf('位置矩阵维度错误: [%d,%d], 预期 [%d,3]', size(positions,1), size(positions,2), n));

    % 检查所有位置在域内
    in_domain = all(positions(:,1) >= params.domain.xmin) && ...
                all(positions(:,1) <= params.domain.xmax) && ...
                all(positions(:,2) >= params.domain.ymin) && ...
                all(positions(:,2) <= params.domain.ymax) && ...
                all(positions(:,3) >= params.domain.zmin) && ...
                all(positions(:,3) <= params.domain.zmax);
    assert(in_domain, '部分智能体位置超出域边界');

    fprintf('  [PASS] %d个智能体, 全部在域内, x范围=[%.1f,%.1f]\n', ...
        n, min(positions(:,1)), max(positions(:,1)));
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试5: init_agents_uniform 位置合理 ==========
fprintf('\n--- 测试5: init_agents_uniform 位置合理 ---\n');
try
    rng(42);
    positions_u = init_agents_uniform(params);
    n = params.agent.num;

    assert(isequal(size(positions_u), [n, 3]), ...
        sprintf('均匀位置维度错误: [%d,%d]', size(positions_u,1), size(positions_u,2)));

    % 检查在域的80%范围内
    in_domain = all(positions_u(:,1) >= params.domain.xmin) && ...
                all(positions_u(:,1) <= params.domain.xmax) && ...
                all(positions_u(:,2) >= params.domain.ymin) && ...
                all(positions_u(:,2) <= params.domain.ymax) && ...
                all(positions_u(:,3) >= params.domain.zmin) && ...
                all(positions_u(:,3) <= params.domain.zmax);
    assert(in_domain, '均匀分布部分位置超出域边界');

    fprintf('  [PASS] %d个智能体均匀分布, 全部在域内\n', n);
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试6: 密度函数时变性 ==========
fprintf('\n--- 测试6: 密度函数时变性 ---\n');
try
    src = params.plume.source_pos;
    x_t = src(1) + 100;

    phi_t0  = compute_density(x_t, 0, src(3), 0,  params);
    phi_t25 = compute_density(x_t, 0, src(3), 25, params);

    assert(phi_t0 ~= phi_t25, '不同时刻密度应不同');

    fprintf('  [PASS] t=0: phi=%.4f, t=25: phi=%.4f\n', phi_t0, phi_t25);
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试7: 密度场切片可视化 ==========
fprintf('\n--- 测试7: 密度场切片可视化 ---\n');
try
    figure('Name', 'Step 2 - 密度函数验证', 'Position', [50 50 1200 400]);
    src = params.plume.source_pos;

    % 公共参数：仅下游区域
    nx_vis = 200; ny_vis = 150;
    x_down = linspace(src(1)+5, 500, nx_vis);
    y_vis = linspace(-200, 200, ny_vis);

    % 子图1: x-y 切片（z=源深度）— 密度增量，线性尺度
    subplot(1,3,1);
    [Xv, Yv] = meshgrid(x_down, y_vis);
    Zv = ones(size(Xv)) * src(3);
    phi_xy = compute_density(Xv, Yv, Zv, 0, params);
    phi_delta = phi_xy - params.density.beta;  % 减去基底密度
    pcolor(Xv, Yv, phi_delta); shading flat;
    caxis([0, max(phi_delta(:))]);
    colorbar; xlabel('X (m)'); ylabel('Y (m)');
    title(sprintf('\\phi - \\beta (X-Y, z=%.0fm)', src(3)));
    hold on; plot(src(1), src(2), 'r*', 'MarkerSize', 15); hold off;

    % 诊断输出
    fprintf('  phi_delta: min=%.6f, max=%.6f\n', min(phi_delta(:)), max(phi_delta(:)));
    n_nonzero = sum(phi_delta(:) > 0.01);
    fprintf('  phi_delta>0.01 的点数: %d / %d (%.1f%%)\n', ...
        n_nonzero, numel(phi_delta), 100*n_nonzero/numel(phi_delta));

    % 子图2: x-y 切片 — 对数尺度，突出羽流锥形结构
    subplot(1,3,2);
    pcolor(Xv, Yv, log10(phi_delta + 1e-6)); shading flat;
    caxis([log10(0.01), log10(max(phi_delta(:)))]);
    colorbar; xlabel('X (m)'); ylabel('Y (m)');
    title(sprintf('log_{10}(\\phi-\\beta+\\epsilon) (X-Y, z=%.0fm)', src(3)));
    hold on; plot(src(1), src(2), 'r*', 'MarkerSize', 15); hold off;

    % 子图3: 密度值分布直方图
    subplot(1,3,3);
    N_hist = 5000;
    rng(0);
    x_h = params.domain.xmin + (params.domain.xmax - params.domain.xmin) * rand(N_hist, 1);
    y_h = params.domain.ymin + (params.domain.ymax - params.domain.ymin) * rand(N_hist, 1);
    z_h = params.domain.zmin + (params.domain.zmax - params.domain.zmin) * rand(N_hist, 1);
    phi_h = compute_density(x_h, y_h, z_h, 0, params);
    histogram(phi_h, 50);
    xlabel('\phi (密度)'); ylabel('采样点数');
    title(sprintf('密度分布 (\\alpha=%.0f, \\beta=%.2f)', params.density.alpha, params.density.beta));

    % 保存图片
    save_dir = fullfile(fileparts(mfilename('fullpath')), '..', '测试结果', 'Step2_密度函数');
    if ~exist(save_dir, 'dir'), mkdir(save_dir); end
    exportgraphics(gcf, fullfile(save_dir, 'step2_density_visualization.png'), 'Resolution', 150);

    fprintf('  [PASS] 可视化图表已生成并保存\n');
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 总结 ==========
fprintf('\n========================================\n');
fprintf('  Step 2 测试结果: %d PASS, %d FAIL\n', pass_count, fail_count);
if fail_count == 0
    fprintf('  >>> Step 2 全部通过！可以进入 Step 3 <<<\n');
else
    fprintf('  >>> 存在失败项，请检查后重试 <<<\n');
end
fprintf('========================================\n');
end
