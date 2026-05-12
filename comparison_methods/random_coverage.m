function new_positions = random_coverage(positions, ~, params)
% random_coverage - 随机覆盖：各向同性随机游走
%
% 输入:
%   positions: n x 3 当前位置
%   ~ (t): 未使用
%   params: 参数结构体
%
% 输出:
%   new_positions: n x 3 更新后位置（在域内）

    n = size(positions, 1);
    v_max = params.agent.max_speed;
    dt = params.algorithm.dt;

    % 随机方向（三维球面均匀分布）
    random_dir = randn(n, 3);
    random_dir = random_dir ./ vecnorm(random_dir, 2, 2);

    % 随机速度（0到v_max之间均匀分布）
    random_speed = v_max * rand(n, 1);

    % 位置更新
    new_positions = positions + random_dir .* random_speed * dt;

    % 域边界约束
    domain = params.domain;
    new_positions(:,1) = max(domain.xmin, min(domain.xmax, new_positions(:,1)));
    new_positions(:,2) = max(domain.ymin, min(domain.ymax, new_positions(:,2)));
    new_positions(:,3) = max(domain.zmin, min(domain.zmax, new_positions(:,3)));
end
