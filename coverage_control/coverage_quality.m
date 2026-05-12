function H = coverage_quality(agent_positions, sample_data, t, params)
% coverage_quality - 计算覆盖质量函数 H(t)
%
% H = ∫ φ(x,t) · min_i‖x - p_i‖² dx  （蒙特卡罗估计）
%
% 自适应方法：复用重要性采样数据 → H = (1/N)·Σ(w_j·d_j²)
% 对比方法：  独立均匀采样   → H = (V/N)·Σ(φ_j·d_j²)
% 两者均为同一积分的无偏估计，可直接比较
%
% 输入:
%   agent_positions: n x 3 矩阵（当 sample_data 为空时使用）
%   sample_data: struct（来自 lloyd_iteration 的采样数据），可选
%                若为空则独立采样计算
%   t: 当前时间 (s)
%   params: 参数结构体
%
% 输出:
%   H: 标量，覆盖质量值（越小越好）

    if nargin >= 2 && ~isempty(sample_data) && isfield(sample_data, 'w')
        % 复用 Lloyd 迭代中的重要性采样数据
        w = sample_data.w;  % 重要性权重 w = φ/g
        min_dist = min(sample_data.dist_matrix, [], 2);
        H = sum(w .* min_dist.^2) / numel(w);
    else
        % 独立均匀采样（用于对比方法）
        N = params.algorithm.sample_num;
        domain = params.domain;
        samples = zeros(N, 3);
        samples(:,1) = domain.xmin + (domain.xmax - domain.xmin) * rand(N, 1);
        samples(:,2) = domain.ymin + (domain.ymax - domain.ymin) * rand(N, 1);
        samples(:,3) = domain.zmin + (domain.zmax - domain.zmin) * rand(N, 1);
        phi = compute_density(samples(:,1), samples(:,2), samples(:,3), t, params);
        min_dist = min(pdist2(samples, agent_positions), [], 2);

        % 域体积
        domain_vol = (domain.xmax - domain.xmin) * ...
                     (domain.ymax - domain.ymin) * ...
                     (domain.zmax - domain.zmin);

        % 均匀采样蒙特卡罗估计
        H = domain_vol / numel(phi) * sum(phi .* min_dist.^2);
    end
end
