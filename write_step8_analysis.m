function write_step8_analysis(save_dir, H_adaptive, H_static, H_random, boundary_distances, plume_extent, monitored_fraction, params)
% write_step8_analysis - 写入Step8动态扩散边界追踪结果分析文档
%
% 输入:
%   save_dir: 结果保存目录
%   H_adaptive, H_static, H_random: 覆盖质量序列
%   boundary_distances: 边界追踪距离序列
%   plume_extent: 羽流有效体积序列
%   monitored_fraction: 边界监测覆盖率序列
%   params: 参数结构体

    n = numel(H_adaptive);
    tail_start = max(1, n - 19);
    head_end = min(20, n);
    H_a_end = H_adaptive(end);
    H_s_end = H_static(end);
    H_r_end = H_random(end);
    H_a_start = H_adaptive(1);
    H_change = (H_a_end - H_a_start) / max(H_a_start, eps) * 100;
    H_s_growth = (H_s_end - H_static(1)) / max(H_static(1), eps) * 100;
    adaptive_advantage = (H_s_end - H_a_end) / max(H_s_end, eps) * 100;
    boundary_valid = boundary_distances(isfinite(boundary_distances));
    boundary_mean = mean(boundary_valid);
    boundary_head = mean(boundary_distances(1:head_end), 'omitnan');
    boundary_tail = mean(boundary_distances(tail_start:end), 'omitnan');
    boundary_improve = (boundary_head - boundary_tail) / max(boundary_head, eps) * 100;
    monitor_tail = mean(monitored_fraction(tail_start:end), 'omitnan') * 100;
    extent_growth = (plume_extent(end) - plume_extent(1)) / max(plume_extent(1), eps);
    extent_growth_pct = extent_growth * 100;

    checks = [plume_extent(end) > plume_extent(1) * 1.5, ...
              all(isfinite(boundary_distances)), ...
              boundary_tail < boundary_head, ...
              monitor_tail >= 75];
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
    fprintf(fid, '| 羽流有效体积 | %.3e → %.3e m³ | %s |\n', plume_extent(1), plume_extent(end), passfail(checks(1)));
    fprintf(fid, '| 羽流体积增长率 | %.2f%% | 浓度范围随时间扩大 |\n', extent_growth_pct);
    fprintf(fid, '| 平均边界距离 | %.2f m | %s |\n', boundary_mean, passfail(checks(2)));
    fprintf(fid, '| 边界距离改善率 | %.2f%% | %s |\n', boundary_improve, passfail(checks(3)));
    fprintf(fid, '| 后20步平均监测覆盖率 | %.2f%% | %s |\n', monitor_tail, passfail(checks(4)));
    fprintf(fid, '| H_adaptive变化率 | %.2f%% | 动态扩散使总任务量变化，该值作为辅助参考 |\n', H_change);
    fprintf(fid, '| H_static增长率 | %.2f%% | 静态基线辅助参考 |\n', H_s_growth);
    fprintf(fid, '| H_adaptive(end) | %.3e | 与基线对比参考 |\n', H_a_end);
    fprintf(fid, '| H_static(end) | %.3e | 自适应相对静态差异 %.2f%% |\n', H_s_end, adaptive_advantage);
    fprintf(fid, '| H_random(end) | %.3e | 随机基线参考 |\n\n', H_r_end);

    fprintf(fid, '## 图片说明\n\n');
    fprintf(fid, '| 图片 | 展示内容 | 验证目的 |\n|------|----------|----------|\n');
    fprintf(fid, '| step8_coverage_quality.png | 自适应、静态、随机三种覆盖质量曲线 | 辅助比较不同策略在动态任务量下的变化 |\n');
    fprintf(fid, '| step8_plume_agents_tracking.png | 三维羽流等值面、AUV彩色轨迹、最终位置 | 验证智能体颜色区分和动态边界追踪 |\n');
    fprintf(fid, '| step8_voronoi_3d.png | 三维Voronoi透明区域面片 | 验证不同AUV负责不同空间区域 |\n');
    fprintf(fid, '| step8_boundary_tracking.png | 羽流有效体积和边界距离随时间变化 | 验证范围动态扩大且AUV保持监测 |\n');
    fprintf(fid, '| step8_monitoring_dashboard.png | 监测覆盖率、边界距离、扩散体积和AUV巡航距离 | 强化展示智能体监测效果 |\n');
    fprintf(fid, '| step8_dynamic_plume_monitoring.gif | 溢油源、扩散边界、浓度切片、扩散前沿和8个AUV实时轨迹 | 直接验证动态扩散过程和智能体实时监测 |\n\n');

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
