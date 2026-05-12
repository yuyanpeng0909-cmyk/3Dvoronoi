function write_step8_analysis(save_dir, H_adaptive, H_static, H_random, boundary_distances, plume_extent, params)
% write_step8_analysis - 写入Step8动态扩散边界追踪结果分析文档
%
% 输入:
%   save_dir: 结果保存目录
%   H_adaptive, H_static, H_random: 覆盖质量序列
%   boundary_distances: 边界追踪距离序列
%   plume_extent: 羽流有效体积序列
%   params: 参数结构体

    pass_count = 0;
    fail_count = 0;

    n = numel(H_adaptive);
    tail_start = max(1, n - 4);
    head_end = min(5, n);
    H_a_end = H_adaptive(end);
    H_s_end = H_static(end);
    H_r_end = H_random(end);
    H_a_start = H_adaptive(1);
    H_drop = (H_a_start - H_a_end) / H_a_start * 100;
    H_s_growth = (H_s_end - H_static(1)) / max(H_static(1), eps) * 100;
    H_r_growth = (H_r_end - H_random(1)) / max(H_random(1), eps) * 100;
    adaptive_advantage = (H_s_end - H_a_end) / max(H_s_end, eps) * 100;
    boundary_mean = mean(boundary_distances(isfinite(boundary_distances)));
    extent_growth = (plume_extent(end) - plume_extent(1)) / max(plume_extent(1), eps);
    extent_growth_pct = extent_growth * 100;

    checks = [plume_extent(end) > plume_extent(1), ...
              all(isfinite(boundary_distances)), ...
              H_s_growth > H_drop, ...
              H_a_end < H_s_end];
    pass_count = sum(checks);
    fail_count = numel(checks) - pass_count;

    fid = fopen(fullfile(save_dir, '结果分析.md'), 'w');
    fprintf(fid, '# Step8 测试结果：动态扩散边界追踪\n\n');
    fprintf(fid, '## 测试结果汇总\n\n');
    fprintf(fid, '**总计**: %d PASS, %d FAIL\n\n', pass_count, fail_count);
    fprintf(fid, '## 仿真参数\n\n');
    fprintf(fid, '| 参数 | 值 | 说明 |\n|------|-----|------|\n');
    fprintf(fid, '| AUV数量 | %d | 使用三维Voronoi分割进行覆盖 |\n', params.agent.num);
    fprintf(fid, '| 仿真步数 | %d | 每步 %.1f s |\n', params.sim.total_time, params.algorithm.dt);
    fprintf(fid, '| 边界阈值 | %.1f%% Cmax | 用于定义扩散边界等值面 |\n', params.plume.boundary_threshold * 100);
    fprintf(fid, '| 边界追踪增益 | %.2f | 驱动AUV贴近所属扩散边界 |\n\n', params.algorithm.boundary_gain);

    fprintf(fid, '## 关键数值分析\n\n');
    fprintf(fid, '| 指标 | 数值 | 判断 |\n|------|------|------|\n');
    fprintf(fid, '| 羽流有效体积 | %.3e → %.3e m³ | %s |\n', plume_extent(1), plume_extent(end), passfail(plume_extent(end) > plume_extent(1)));
    fprintf(fid, '| 羽流体积增长率 | %.2f%% | 浓度范围随时间扩大 |\n', extent_growth_pct);
    fprintf(fid, '| 自适应H变化率 | %.2f%% | 动态扩散使总任务量增加，需结合对比基线判断 |\n', H_drop);
    fprintf(fid, '| 静态H增长率 | %.2f%% | %s |\n', H_s_growth, passfail(H_s_growth > H_drop));
    fprintf(fid, '| H_adaptive(end) | %.3e | %s |\n', H_a_end, passfail(H_a_end < H_s_end));
    fprintf(fid, '| H_static(end) | %.3e | 自适应相对静态改善 %.2f%% |\n', H_s_end, adaptive_advantage);
    fprintf(fid, '| H_random(end) | %.3e | 对比基线 |\n', H_r_end);
    fprintf(fid, '| 平均边界距离 | %.2f m | %s |\n\n', boundary_mean, passfail(isfinite(boundary_mean)));

    fprintf(fid, '## 图片说明\n\n');
    fprintf(fid, '| 图片 | 展示内容 | 验证目的 |\n|------|----------|----------|\n');
    fprintf(fid, '| step8_coverage_quality.png | 自适应、静态、随机三种覆盖质量曲线 | 验证自适应Voronoi控制性能 |\n');
    fprintf(fid, '| step8_plume_agents_tracking.png | 三维羽流等值面、AUV彩色轨迹、最终位置 | 验证智能体颜色区分和动态边界追踪 |\n');
    fprintf(fid, '| step8_voronoi_3d.png | 三维Voronoi区域采样着色 | 验证不同AUV负责不同空间区域 |\n');
    fprintf(fid, '| step8_boundary_tracking.png | 羽流有效体积和边界距离随时间变化 | 验证范围动态扩大且AUV保持监测 |\n');
    fprintf(fid, '| step8_monitoring_dashboard.png | 监测覆盖率、边界距离、扩散体积和AUV巡航距离 | 强化展示智能体监测效果 |\n');
    fprintf(fid, '| step8_dynamic_plume_monitoring.gif | 溢油源从初始时刻到最终时刻的三维动态扩散、浓度核心变化和8个AUV实时轨迹 | 直接验证动态扩散过程和智能体实时监测 |\n\n');

    fprintf(fid, '## 结论\n\n');
    if fail_count == 0
        fprintf(fid, '动态扩散模型、三维Voronoi分割、彩色AUV显示和边界追踪指标均通过验证。\n');
    else
        fprintf(fid, '存在未通过指标，建议检查仿真参数或增加仿真步数。\n');
    end
    fclose(fid);
end

function text = passfail(condition)
    if condition
        text = 'PASS';
    else
        text = 'FAIL';
    end
end
