function new_positions = standard_cvt(agent_positions, t, params)
% standard_cvt - 标准CVT覆盖控制：纯密度加权质心反馈
% 无边界追踪、无前馈、无分离控制
% 展示智能体向密度峰聚集（集群塌陷）现象
%
% 控制律: v_i = kappa * (cen_i - p_i)
%
% 输入:
%   agent_positions: n x 3 当前位置
%   t: 当前时间 (s)
%   params: 参数结构体
%
% 输出:
%   new_positions: n x 3 更新后位置

    kappa = params.agent.control_gain;
    v_max = params.agent.max_speed;
    dt = params.algorithm.dt;

    % 复用密度加权质心计算
    [centroids, ~, ~, ~, ~, ~] = compute_centroid(agent_positions, t, params);

    % 纯反馈控制律
    velocity = kappa * (centroids - agent_positions);

    % 速度限幅
    speed = sqrt(sum(velocity.^2, 2));
    too_fast = speed > v_max;
    if any(too_fast)
        velocity(too_fast, :) = v_max * velocity(too_fast, :) ./ speed(too_fast);
    end

    % 位置更新
    new_positions = agent_positions + velocity * dt;

    % 域边界约束
    domain = params.domain;
    new_positions(:,1) = max(domain.xmin, min(domain.xmax, new_positions(:,1)));
    new_positions(:,2) = max(domain.ymin, min(domain.ymax, new_positions(:,2)));
    new_positions(:,3) = max(domain.zmin, min(domain.zmax, new_positions(:,3)));
end
