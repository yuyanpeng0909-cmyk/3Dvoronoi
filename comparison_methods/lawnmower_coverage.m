function new_positions = lawnmower_coverage(~, t, params)
% lawnmower_coverage - 三维割草机式预定扫描路径覆盖
% 基于条带分解的预规划开环控制，不响应羽流动态
%
% 输入:
%   ~ (positions): 未使用（开环控制）
%   t: 当前泄漏时间 (s)
%   params: 参数结构体
%
% 输出:
%   new_positions: n x 3，按预规划路径计算的位置

    n = params.agent.num;
    domain = params.domain;
    v_scan = params.agent.max_speed;
    t_monitor = max(0, t - params.sim.pre_release_time);

    % Y方向：将域等分为n条平行扫描道
    lane_width = (domain.ymax - domain.ymin) / n;
    y_centers = domain.ymin + lane_width * ((1:n) - 0.5);

    % X方向：匀速前进扫描，到达边界后折返
    x_range = domain.xmax - domain.xmin;
    v_x = v_scan * 0.75;
    x_travel = v_x * t_monitor;
    % 三角波折返
    x_period = 2 * x_range / v_x;
    x_phase = mod(t_monitor, x_period) / x_period;
    if x_phase < 0.5
        x_pos = domain.xmin + x_range * (x_phase * 2);
    else
        x_pos = domain.xmax - x_range * ((x_phase - 0.5) * 2);
    end

    % Z方向：三角波振荡扫描
    z_range = domain.zmax - domain.zmin;
    v_z = v_scan * 0.66;
    z_period = 2 * z_range / v_z;

    new_positions = zeros(n, 3);
    new_positions(:, 1) = x_pos;
    new_positions(:, 2) = y_centers';

    for i = 1:n
        % 各AUV在Z方向有相位偏移，避免同步聚集
        phase_i = mod(t_monitor / z_period + (i - 1) * 0.12, 1.0);
        if phase_i < 0.5
            new_positions(i, 3) = domain.zmin + z_range * (phase_i * 2);
        else
            new_positions(i, 3) = domain.zmax - z_range * ((phase_i - 0.5) * 2);
        end
    end
end
