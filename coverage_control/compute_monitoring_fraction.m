function fraction = compute_monitoring_fraction(agent_positions, boundary_points, params)
% compute_monitoring_fraction - 计算扩散边界被AUV有效监测的比例
%
% 输入:
%   agent_positions: n x 3 AUV位置
%   boundary_points: Nb x 3 扩散边界采样点
%   params: 参数结构体
%
% 输出:
%   fraction: 距任一AUV小于监测半径的边界点比例

    if isempty(boundary_points)
        fraction = 0;
        return;
    end

    if isfield(params.agent, 'sense_radius')
        sense_radius = params.agent.sense_radius;
    else
        sense_radius = 90;
    end

    min_dist = min(pdist2(boundary_points, agent_positions), [], 2);
    fraction = mean(min_dist <= sense_radius);
end
