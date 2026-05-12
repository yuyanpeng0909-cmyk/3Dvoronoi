function test_step1_plume()
% test_step1_plume - Step 1 测试：参数初始化 + 羽流模型
% 运行方式: cd 到项目根目录, addpath('plume_model'), 然后 test_step1_plume

fprintf('========================================\n');
fprintf('  Step 1 测试：参数初始化 + 羽流模型\n');
fprintf('========================================\n\n');

pass_count = 0;
fail_count = 0;

%% ========== 测试1: 参数结构体完整性 ==========
fprintf('--- 测试1: 参数结构体完整性 ---\n');
try
    params = init_parameters();

    % 检查顶级字段
    fields = {'domain', 'agent', 'plume', 'density', 'algorithm', 'sim'};
    for i = 1:length(fields)
        assert(isfield(params, fields{i}), ...
            sprintf('缺少字段: params.%s', fields{i}));
    end

    % 检查域参数
    assert(params.domain.xmin == 0 && params.domain.xmax == 500);
    assert(params.domain.ymin == -200 && params.domain.ymax == 200);
    assert(params.domain.zmin == -300 && params.domain.zmax == 0);

    % 检查智能体参数
    assert(params.agent.num == 8);
    assert(params.agent.max_speed > 0);
    assert(params.agent.control_gain > 0);

    % 检查C_max_estimate已计算
    assert(isfield(params.plume, 'C_max_estimate'), 'C_max_estimate 未计算');
    assert(params.plume.C_max_estimate > 0, 'C_max_estimate 应为正数');

    fprintf('  [PASS] 参数结构体完整，C_max_estimate = %.6f\n', params.plume.C_max_estimate);
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试2: 浓度场下游非零、上游为零 ==========
fprintf('\n--- 测试2: 浓度场上下游特性 ---\n');
try
    src = params.plume.source_pos;

    % 下游点（应有非零浓度）
    C_downstream = gaussian_plume_3d(src(1)+100, src(2), src(3), 0, params);
    assert(C_downstream > 0, '下游浓度应为正');

    % 上游点（应为零）
    C_upstream = gaussian_plume_3d(src(1)-10, src(2), src(3), 0, params);
    assert(C_upstream == 0, '上游浓度应为零');

    fprintf('  [PASS] 下游 C=%.6f, 上游 C=%.6f\n', C_downstream, C_upstream);
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试3: 浓度场关于y轴对称 ==========
fprintf('\n--- 测试3: y轴对称性 ---\n');
try
    src = params.plume.source_pos;
    x_t = src(1) + 100;
    y_offset = 50;
    z_t = src(3);

    C_pos = gaussian_plume_3d(x_t, src(2)+y_offset, z_t, 0, params);
    C_neg = gaussian_plume_3d(x_t, src(2)-y_offset, z_t, 0, params);

    assert(abs(C_pos - C_neg) < 1e-10, '浓度场关于y轴应对称');
    fprintf('  [PASS] y=+50: C=%.6f, y=-50: C=%.6f, 差值=%.2e\n', ...
        C_pos, C_neg, abs(C_pos-C_neg));
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试4: 镜像源反射项 ==========
fprintf('\n--- 测试4: 镜像源反射项 ---\n');
try
    src = params.plume.source_pos;
    x_t = src(1) + 100;

    % z=0 附近（海面反射）应有非零浓度
    C_surface = gaussian_plume_3d(x_t, 0, -5, 0, params);
    assert(C_surface > 0, '海面附近应有浓度（镜像反射）');

    % 在源深度处浓度应最高
    C_at_source_depth = gaussian_plume_3d(x_t, 0, src(3), 0, params);
    assert(C_at_source_depth >= C_surface, '源深度处浓度应 >= 海面附近');

    fprintf('  [PASS] 源深度 C=%.6f, 海面 C=%.6f\n', C_at_source_depth, C_surface);
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试5: 时变特性 ==========
fprintf('\n--- 测试5: 时变特性 ---\n');
try
    src = params.plume.source_pos;
    x_t = src(1) + 100;

    C_t0  = gaussian_plume_3d(x_t, 0, src(3), 0,  params);
    C_t25 = gaussian_plume_3d(x_t, 0, src(3), 25, params);

    assert(C_t0 ~= C_t25, '不同时刻浓度应不同（时变特性）');
    fprintf('  [PASS] t=0: C=%.6f, t=25: C=%.6f, 变化=%.2f%%\n', ...
        C_t0, C_t25, abs(C_t25-C_t0)/C_t0*100);
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试6: update_plume 网格维度 ==========
fprintf('\n--- 测试6: update_plume 网格维度 ---\n');
try
    plume_state = update_plume(0, params);

    assert(isfield(plume_state, 'X'), '缺少 X 字段');
    assert(isfield(plume_state, 'C'), '缺少 C 字段');

    [sx, sy, sz] = size(plume_state.X);
    assert(sx == 80 && sy == 60 && sz == 50, ...
        sprintf('网格维度错误: [%d,%d,%d], 预期 [80,60,50]', sx, sy, sz));
    assert(isequal(size(plume_state.C), size(plume_state.X)), ...
        'C 与 X 维度不一致');

    C_max = max(plume_state.C(:));
    C_min = min(plume_state.C(:));
    assert(C_max > 0, '最大浓度应为正');
    assert(C_min >= 0, '最小浓度应非负');

    fprintf('  [PASS] 网格 [%d,%d,%d], C_max=%.4e, C_min=%.4e\n', ...
        sx, sy, sz, C_max, C_min);
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试7: 浓度场沿距离衰减 ==========
fprintf('\n--- 测试7: 浓度场空间衰减 ---\n');
try
    src = params.plume.source_pos;

    C_near = gaussian_plume_3d(src(1)+50,  0, src(3), 0, params);
    C_mid  = gaussian_plume_3d(src(1)+150, 0, src(3), 0, params);
    C_far  = gaussian_plume_3d(src(1)+350, 0, src(3), 0, params);

    assert(C_near > C_mid && C_mid > C_far, ...
        '浓度应随距离衰减: near > mid > far');

    fprintf('  [PASS] x+50: %.4e, x+150: %.4e, x+350: %.4e\n', ...
        C_near, C_mid, C_far);
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 测试8: 浓度场切片可视化 ==========
fprintf('\n--- 测试8: 浓度场切片可视化 ---\n');
try
    figure('Name', 'Step 1 - 浓度场验证', 'Position', [50 50 1200 400]);
    src = params.plume.source_pos;

    % 仅绘制下游区域（x > 源位置），避免上游零值干扰颜色映射
    nx_vis = 150; ny_vis = 100;
    x_down = linspace(src(1)+5, 500, nx_vis);  % 下游区域
    y_vis = linspace(-200, 200, ny_vis);

    % 子图1: x-y 平面切片 (z = 源深度)
    subplot(1,3,1);
    [Xv, Yv] = meshgrid(x_down, y_vis);
    Zv = ones(size(Xv)) * src(3);
    C_xy = gaussian_plume_3d(Xv, Yv, Zv, 0, params);
    pcolor(Xv, Yv, C_xy); shading flat;
    caxis([0, max(C_xy(:))]);
    colorbar; xlabel('X (m)'); ylabel('Y (m)');
    title(sprintf('X-Y 切片 (z=%.0fm)', src(3)));
    hold on; plot(src(1), src(2), 'rp', 'MarkerSize', 15, 'MarkerFaceColor', 'r'); hold off;

    % 子图2: x-z 平面切片 (y = 0)
    subplot(1,3,2);
    nz_vis = 80;
    z_vis = linspace(-300, 0, nz_vis);
    [Xv2, Zv2] = meshgrid(x_down, z_vis);
    Yv2 = zeros(size(Xv2));
    C_xz = gaussian_plume_3d(Xv2, Yv2, Zv2, 0, params);
    pcolor(Xv2, Zv2, C_xz); shading flat;
    caxis([0, max(C_xz(:))]);
    colorbar; xlabel('X (m)'); ylabel('Z (m)');
    title('X-Z 切片 (y=0)');
    hold on; plot(src(1), src(3), 'rp', 'MarkerSize', 15, 'MarkerFaceColor', 'r'); hold off;

    % 子图3: 时变对比
    subplot(1,3,3);
    src = params.plume.source_pos;
    t_list = [0, 12.5, 25, 37.5, 50];
    C_vs_t = zeros(1, length(t_list));
    for ti = 1:length(t_list)
        C_vs_t(ti) = gaussian_plume_3d(src(1)+100, 0, src(3), t_list(ti), params);
    end
    plot(t_list, C_vs_t, 'bo-', 'LineWidth', 1.5, 'MarkerSize', 8);
    xlabel('时间 (s)'); ylabel('浓度 (kg/m^3)');
    title('源下游100m处浓度随时间变化');
    grid on;

    % 保存图片
    save_dir = fullfile(fileparts(mfilename('fullpath')), '..', '测试结果', 'Step1_羽流模型');
    if ~exist(save_dir, 'dir'), mkdir(save_dir); end
    exportgraphics(gcf, fullfile(save_dir, 'step1_concentration_slices.png'), 'Resolution', 150);

    fprintf('  [PASS] 可视化图表已生成并保存\n');
    pass_count = pass_count + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% ========== 总结 ==========
fprintf('\n========================================\n');
fprintf('  Step 1 测试结果: %d PASS, %d FAIL\n', pass_count, fail_count);
if fail_count == 0
    fprintf('  >>> Step 1 全部通过！可以进入 Step 2 <<<\n');
else
    fprintf('  >>> 存在失败项，请检查后重试 <<<\n');
end
fprintf('========================================\n');
end
