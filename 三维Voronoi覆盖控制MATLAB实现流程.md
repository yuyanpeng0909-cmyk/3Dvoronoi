# 题目一：基于时变密度函数的三维Voronoi水下溢油羽流自适应覆盖控制 — MATLAB实现流程

## 研究背景

- **研究方向**：三维Voronoi覆盖控制算法
- **研究对象**：水下多智能体（8个AUV）对溢油羽流的自适应覆盖监测
- **目标会议**：中国自动化大会（CAC）
- **仿真工具**：MATLAB 2024b
- **安全约束**：本期不含CBF，纯覆盖控制（后期扩展）
- **对比实验**：自适应覆盖 vs 静态均匀覆盖 vs 随机覆盖

### 问题建模

考虑 $n$ 个AUV在三维有界域 $\mathcal{D} \subset \mathbb{R}^3$ 中对时变溢油羽流进行覆盖监测。域内存在时变密度函数 $\phi(x,t)$，由水下溢油羽流浓度场驱动。控制目标是最小化**位置感知代价函数**（locational cost）：

$$\mathcal{H}(p_1,\ldots,p_n,t) = \sum_{i=1}^{n} \int_{V_i} \phi(x,t) \cdot \|x - p_i\|^2 \, dx$$

其中 $V_i$ 为第 $i$ 个智能体的Voronoi单元，$p_i$ 为其位置。该代价函数衡量了覆盖配置与密度分布的匹配程度。

---

## 一、整体代码架构

```
main_simulation.m              % 主仿真脚本（入口）
│
├── init_parameters.m          % 全局参数初始化
├── init_agents.m              % 智能体随机初始位置
├── init_agents_uniform.m      % 智能体均匀网格初始位置
│
├── plume_model/               % 溢油羽流模型模块
│   ├── gaussian_plume_3d.m    % 三维高斯羽流浓度场计算（解析模型）
│   └── update_plume.m         % 羽流浓度场网格采样与更新
│
├── density_function/          % 密度函数模块
│   └── compute_density.m      % 基于羽流浓度的时变密度函数 φ(x,t)
│
├── coverage_control/          % 覆盖控制算法模块
│   ├── compute_centroid.m     % Voronoi单元密度加权质心计算（蒙特卡罗采样）
│   ├── lloyd_iteration.m      % Lloyd算法单步迭代（含前馈补偿）
│   └── coverage_quality.m     % 覆盖质量函数 H(t) 计算
│
├── comparison_methods/        % 对比方法模块
│   ├── static_coverage.m      % 静态均匀覆盖（固定位置）
│   └── random_coverage.m      % 随机覆盖（随机游走）
│
└── visualization/             % 可视化模块
    ├── plot_results.m         % 覆盖质量对比曲线 + 多项式拟合
    ├── plot_plume_3d.m        % 三维羽流等值面可视化
    ├── plot_agents_3d.m       % 三维智能体轨迹可视化
    └── plot_voronoi_3d.m      % 三维Voronoi剖分可视化（基于采样着色）
```

### 架构设计原则

1. **模块化**：每个功能独立成函数，通过 `params` 结构体传递参数
2. **数据流清晰**：`plume_model → density_function → coverage_control` 单向依赖
3. **采样复用**：Lloyd迭代与覆盖质量计算共享采样点和距离矩阵，避免重复计算
4. **可扩展**：预留前馈补偿和CBF安全约束接口

---

## 二、模块一：参数初始化（init_parameters.m）

### 2.1 功能说明

定义仿真所需的所有全局参数，统一用 `struct` 组织。所有物理量采用国际单位制（SI）。

### 2.2 关键参数

```matlab
function params = init_parameters()
%% === 场景参数 ===
params.domain = struct(...
    'xmin', 0, 'xmax', 500, ...      % x方向范围 (m)，沿洋流方向
    'ymin', -200, 'ymax', 200, ...   % y方向范围 (m)，横向
    'zmin', -300, 'zmax', 0 ...      % z方向范围 (m)，深度（负值表示水下）
);

%% === 智能体参数 ===
params.agent = struct(...
    'num', 8, ...                     % 智能体数量
    'max_speed', 2.0, ...            % 最大速度 (m/s)，典型AUV：1-3 m/s
    'control_gain', 0.5 ...          % 控制增益 k
);

%% === 溢油羽流参数（三维高斯羽流模型） ===
params.plume = struct(...
    'source_pos', [50, 0, -50], ...  % 源位置 [x, y, z] (m)
    'Q', 100, ...                    % 源强度 (kg/s)
    'u_current', 0.5, ...            % 洋流速度 (m/s)，沿x正方向
    'sigma_y0', 5, ...               % 初始横向扩散系数 (m)
    'sigma_z0', 3, ...               % 初始垂向扩散系数 (m)
    'ky', 0.1, ...                   % 横向扩散增长率
    'kz', 0.05 ...                   % 垂向扩散增长率
);

%% === 密度函数参数 ===
params.density = struct(...
    'alpha', 10.0, ...               % 浓度缩放系数（控制聚集强度）
    'beta', 0.1 ...                  % 基底密度（保证全域正密度）
);

%% === 算法参数 ===
params.algorithm = struct(...
    'sample_num', 50000, ...         % 蒙特卡罗采样点数
    'dt', 1.0, ...                   % 仿真时间步长 (s)
    'ff_gain', 0.3 ...              % 前馈补偿增益 γ_ff（质心速度前馈）
);

%% === 仿真参数 ===
params.sim = struct(...
    'total_time', 200, ...           % 总仿真时间步数
    'plume_update_interval', 5 ...   % 羽流采样网格更新间隔（步）
);

%% === 预计算最大浓度（用于归一化） ===
x_test = params.plume.source_pos(1) + 10;
y_test = params.plume.source_pos(2);
z_test = params.plume.source_pos(3);
C_test = gaussian_plume_3d(x_test, y_test, z_test, 0, params);
params.plume.C_max_estimate = C_test * 1.2;  % 留20%余量
end
```

### 2.3 参数物理依据

| 参数 | 值 | 物理依据 |
|------|-----|---------|
| 仿真域 500×400×300 m | 典型近海溢油影响范围 100m~1km |
| AUV最大速度 2.0 m/s | 典型AUV（如Bluefin-21）巡航速度 1-3 m/s |
| 源强度 100 kg/s | 中等规模海底管道泄漏（DeepWater Horizon约 200 kg/s） |
| 洋流速度 0.5 m/s | 近海典型表层/中层洋流 0.1-1.0 m/s |
| 扩散系数 ky=0.1, kz=0.05 | 水下横向扩散 > 垂向扩散（密度分层抑制垂向混合） |
| 控制增益 k=0.5 | 0.1-1.0范围内，兼顾收敛速度与稳定性 |

---

## 三、模块二：三维时变溢油羽流模型（plume_model/）

### 3.1 数学模型

采用**三维高斯羽流模型**描述水下溢油扩散。该模型源于大气高斯烟羽模型，经适配水下环境后广泛应用于水下污染物传输建模 [6,7]。

**浓度场公式**：

$$C(x,y,z,t) = \frac{Q}{2\pi \cdot u \cdot \sigma_y(x,t) \cdot \sigma_z(x,t)} \cdot \exp\!\left(-\frac{(y-y_s)^2}{2\sigma_y^2}\right) \cdot \left[\exp\!\left(-\frac{(z-z_s)^2}{2\sigma_z^2}\right) + \exp\!\left(-\frac{(z+z_s)^2}{2\sigma_z^2}\right)\right]$$

其中：
- $Q$：源强度 (kg/s)
- $u$：洋流速度 (m/s)，沿 $x$ 正方向
- $(x_s, y_s, z_s)$：源位置，$z_s$ 为负值（水下）
- 第一指数项：**直接源**，$z = z_s$ 时取最大值 1
- 第二指数项：**镜像源**（海面 $z=0$ 反射），镜像源位于 $z = -z_s > 0$
- $\sigma_y(x,t)$、$\sigma_z(x,t)$：时变扩散参数

**扩散参数的时变模型**：

$$\sigma_y(x,t) = \sigma_{y0} + k_y \cdot x' \cdot \left(1 + A \cdot \sin\!\left(\frac{2\pi t}{T}\right)\right)$$

$$\sigma_z(x,t) = \sigma_{z0} + k_z \cdot x' \cdot \left(1 + A \cdot \sin\!\left(\frac{2\pi t}{T} + \varphi\right)\right)$$

其中 $x' = \max(x - x_s, 0)$，$A = 0.1$ 为脉动幅度，$T = 50$ s 为脉动周期，$\varphi$ 为相位偏移（模拟横向与垂向扩散的非同步变化）。

### 3.2 gaussian_plume_3d.m

```matlab
function C = gaussian_plume_3d(x, y, z, t, params)
% 三维高斯羽流浓度场计算
% 输入:
%   x, y, z: 标量或同维数组，空间坐标
%   t: 当前时间 (s)
%   params: 参数结构体
% 输出:
%   C: 浓度值（与输入同维度，kg/m^3）

    % 提取参数
    src = params.plume.source_pos;
    Q = params.plume.Q;
    u = params.plume.u_current;
    sigma_y0 = params.plume.sigma_y0;
    sigma_z0 = params.plume.sigma_z0;
    ky = params.plume.ky;
    kz = params.plume.kz;

    % 相对坐标
    dx = x - src(1);    % 下游距离
    dy = y - src(2);    % 横向偏移
    dz = z - src(3);    % 垂向偏移（z_s为负值）
    H = abs(src(3));    % 源深度（正值）

    C = zeros(size(x));

    % 时变扩散参数
    T_period = 50;       % 脉动周期 (s)
    A = 0.1;             % 脉动幅度
    phi_y = 0;           % 横向相位
    phi_z = pi/4;        % 垂向相位偏移（非同步变化）
    perturbation_y = 1 + A * sin(2*pi*t/T_period + phi_y);
    perturbation_z = 1 + A * sin(2*pi*t/T_period + phi_z);

    sigma_y = sigma_y0 + ky * max(dx, 0) * perturbation_y;
    sigma_z = sigma_z0 + kz * max(dx, 0) * perturbation_z;

    % 仅在下游方向有非零浓度
    mask = dx > 0;
    if any(mask)
        sy = sigma_y(mask);
        sz = sigma_z(mask);

        % 防止除零（扩散参数下界）
        sy = max(sy, sigma_y0);
        sz = max(sz, sigma_z0);

        % 高斯羽流公式（含镜像源反射项）
        amplitude = Q ./ (2*pi * u * sy .* sz);
        lateral = exp(-dy(mask).^2 ./ (2*sy.^2));
        vertical = exp(-(dz(mask) - H).^2 ./ (2*sz.^2)) ...
                 + exp(-(dz(mask) + H).^2 ./ (2*sz.^2));

        C(mask) = amplitude .* lateral .* vertical;
    end
end
```

### 3.3 update_plume.m

```matlab
function plume_state = update_plume(t, params)
% 生成当前时刻的羽流浓度场三维采样网格
% 输出: plume_state 结构体，包含网格坐标和浓度场

    nx = 50; ny = 40; nz = 30;
    x = linspace(params.domain.xmin, params.domain.xmax, nx);
    y = linspace(params.domain.ymin, params.domain.ymax, ny);
    z = linspace(params.domain.zmin, params.domain.zmax, nz);
    [X, Y, Z] = ndgrid(x, y, z);

    C = gaussian_plume_3d(X, Y, Z, t, params);

    plume_state = struct(...
        'X', X, 'Y', Y, 'Z', Z, ...
        'C', C, ...
        'x', x, 'y', y, 'z', z ...
    );
end
```

### 3.4 模型说明

- **镜像源反射**：公式中第二指数项 $\exp(-(z'+H)^2/(2\sigma_z^2))$ 模拟海面（z=0）对羽流的反射效应
- **时变机制**：正弦脉动模拟洋流湍流强度变化，相位偏移 $\varphi_z$ 反映横向/垂向扩散的非同步性
- **仅在下游有效**：$dx > 0$ 表示浓度仅在源的洋流下游方向非零（稳态对流扩散近似）
- **局限性**：本模型为解析近似，未含衰减项（生物降解、溶解等），后期可扩展为对流-扩散-衰减PDE模型

---

## 四、模块三：时变密度函数（density_function/）

### 4.1 数学模型

将溢油浓度场映射为覆盖控制的时变密度函数 [2,4]：

$$\phi(x, t) = \alpha \cdot C_{\text{norm}}(x, t) + \beta$$

其中：
- $C_{\text{norm}}(x,t) = \min\!\left(\frac{C(x,t)}{C_{\max}^{\text{est}}},\, 1\right)$：归一化浓度 ∈ [0,1]
- $\alpha$：浓度权重系数，控制高浓度区域的聚集强度
- $\beta > 0$：基底密度，保证低浓度区域也有覆盖，避免所有智能体聚集在源附近

**密度函数性质**：
- 处处为正：$\phi(x,t) \geq \beta > 0$
- 时变性：随羽流演化而变化，驱动智能体持续自适应调整
- 与控制目标的联系：密度越高的区域，对应Voronoi单元越小（更多智能体覆盖该区域）

### 4.2 compute_density.m

```matlab
function phi = compute_density(x, y, z, t, params)
% 计算给定位置和时间的密度函数值
% 输入: x, y, z (标量或数组), t (标量), params
% 输出: phi (密度值，与输入同维度，处处 > 0)

    % 计算浓度
    C = gaussian_plume_3d(x, y, z, t, params);

    % 归一化
    C_max = params.plume.C_max_estimate;
    C_norm = min(C / C_max, 1);  % 截断到 [0, 1]

    % 密度函数
    alpha = params.density.alpha;
    beta = params.density.beta;
    phi = alpha * C_norm + beta;
end
```

### 4.3 参数选取建议

| α/β 比值 | 效果 | 适用场景 |
|-----------|------|----------|
| 10-20 | 温和聚集，均匀覆盖为主 | 大范围环境监测 |
| 20-50 | 中等聚集，兼顾热点与全局 | 溢油追踪（推荐） |
| 50-100 | 强聚集，智能体集中于热点 | 精确采样热点区域 |

---

## 五、模块四：三维Voronoi剖分与质心计算（coverage_control/）

### 5.1 方法选择：基于采样的Lloyd算法

在有限三维域中，精确Voronoi剖分会产生**无界单元**（包含 ∞ 顶点），需要额外的边界截断处理。本实现采用**基于蒙特卡罗采样的Lloyd算法** [3,5]：

- 不依赖精确Voronoi几何，通过采样点近似
- 在域内生成均匀随机采样点，按最近智能体分组
- 每组的密度加权均值即为近似质心
- **优势**：自动处理边界问题、天然支持密度加权、实现简单

### 5.2 compute_centroid.m

```matlab
function [centroids, samples, phi, dist_matrix, nearest_agent] = compute_centroid(agent_positions, t, params)
% 基于蒙特卡罗采样计算密度加权质心（附带返回采样数据供复用）
% 输入:
%   agent_positions: n x 3 矩阵
%   t: 当前时间
%   params: 参数结构体
% 输出:
%   centroids: n x 3 质心矩阵
%   samples: N x 3 采样点（用于复用）
%   phi: N x 1 密度值（用于复用）
%   dist_matrix: N x n 距离矩阵（用于复用）
%   nearest_agent: N x 1 最近智能体索引（用于复用）

    n = size(agent_positions, 1);
    N = params.algorithm.sample_num;
    domain = params.domain;

    % 在域内均匀生成采样点
    samples = zeros(N, 3);
    samples(:,1) = domain.xmin + (domain.xmax - domain.xmin) * rand(N, 1);
    samples(:,2) = domain.ymin + (domain.ymax - domain.ymin) * rand(N, 1);
    samples(:,3) = domain.zmin + (domain.zmax - domain.zmin) * rand(N, 1);

    % 计算密度
    phi = compute_density(samples(:,1), samples(:,2), samples(:,3), t, params);

    % 最近智能体分配（Voronoi划分）
    dist_matrix = pdist2(samples, agent_positions);
    [~, nearest_agent] = min(dist_matrix, [], 2);

    % 按智能体分组，计算密度加权质心
    centroids = zeros(n, 3);
    for i = 1:n
        mask = (nearest_agent == i);
        if any(mask)
            w = phi(mask);
            w_total = sum(w);
            centroids(i, :) = sum(w .* samples(mask, :), 1) / w_total;
        else
            centroids(i, :) = agent_positions(i, :);
        end
    end
end
```

### 5.3 关键实现要点

1. **采样点数**：50000-100000，平衡精度与速度
2. **数据复用**：函数同时返回采样点和距离矩阵，供 `coverage_quality` 复用，避免重复计算
3. **pdist2 性能**：8×50000 的距离矩阵约 400000 次距离计算，MATLAB底层C实现，每步 < 0.1s
4. **空单元处理**：若某智能体无采样点分配，质心退化为自身位置

---

## 六、模块五：Lloyd迭代与覆盖控制律（coverage_control/）

### 6.1 控制律设计

**基础控制律**（Lloyd梯度下降 [4]）：

$$\dot{p}_i = k \cdot (c_{V_i} - p_i)$$

其中 $c_{V_i}$ 为第 $i$ 个Voronoi单元的密度加权质心，$k$ 为控制增益。

**含前馈补偿的增强控制律**（参考 Bao et al. [2] 的时变密度跟踪思路）：

$$\dot{p}_i = k \cdot (c_{V_i} - p_i) + \gamma_{\text{ff}} \cdot \dot{c}_{V_i}$$

其中：
- 第一项：**反馈项**，驱动智能体向当前质心移动
- 第二项：**前馈项**，预测质心的运动趋势，补偿密度场的时变效应
- $\gamma_{\text{ff}} \in [0, 1]$：前馈增益，$\gamma_{\text{ff}}=0$ 退化为纯反馈

**质心速度近似**：

$$\dot{c}_{V_i}(t) \approx \frac{c_{V_i}(t) - c_{V_i}(t - \Delta t)}{\Delta t}$$

### 6.2 离散化实现（含速度限幅与边界约束）

```
v_i = k·(c_Vi - p_i) + γ_ff · (c_Vi(t) - c_Vi(t-dt)) / dt
if ‖v_i‖ > v_max:
    v_i = v_max · v_i / ‖v_i‖        % 速度限幅
p_i(t+dt) = p_i(t) + v_i · dt
p_i = clip(p_i, domain_min, domain_max)  % 域边界约束
```

### 6.3 lloyd_iteration.m

```matlab
function [new_positions, centroids, move_distance, sample_data] = lloyd_iteration(agent_positions, t, prev_centroids, params)
% Lloyd算法单步迭代（含前馈补偿）
% 输入:
%   agent_positions: n x 3 当前位置
%   t: 当前时间
%   prev_centroids: n x 3 上一时刻质心（用于前馈计算）
%   params: 参数结构体
% 输出:
%   new_positions: n x 3 更新后位置
%   centroids: n x 3 当前质心
%   move_distance: 平均移动距离
%   sample_data: struct，包含采样数据供复用

    n = size(agent_positions, 1);
    k = params.agent.control_gain;
    v_max = params.agent.max_speed;
    dt = params.algorithm.dt;
    gamma_ff = params.algorithm.ff_gain;

    % Step 1: 计算密度加权质心（同时获取采样数据）
    [centroids, samples, phi, dist_matrix, nearest_agent] = compute_centroid(agent_positions, t, params);

    % Step 2: 反馈控制律
    v_feedback = k * (centroids - agent_positions);

    % Step 3: 前馈补偿（质心速度）
    if ~isempty(prev_centroids)
        dcen_dt = (centroids - prev_centroids) / dt;
        v_feedforward = gamma_ff * dcen_dt;
    else
        v_feedforward = zeros(n, 3);
    end

    velocity = v_feedback + v_feedforward;

    % Step 4: 速度限幅
    speed = sqrt(sum(velocity.^2, 2));
    too_fast = speed > v_max;
    if any(too_fast)
        velocity(too_fast, :) = v_max * velocity(too_fast, :) ./ speed(too_fast);
    end

    % Step 5: 位置更新
    new_positions = agent_positions + velocity * dt;

    % Step 6: 域边界约束
    domain = params.domain;
    new_positions(:,1) = max(domain.xmin, min(domain.xmax, new_positions(:,1)));
    new_positions(:,2) = max(domain.ymin, min(domain.ymax, new_positions(:,2)));
    new_positions(:,3) = max(domain.zmin, min(domain.zmax, new_positions(:,3)));

    % Step 7: 平均移动距离
    move_distance = mean(sqrt(sum((new_positions - agent_positions).^2, 2)));

    % 打包采样数据供复用
    sample_data = struct('samples', samples, 'phi', phi, ...
                         'dist_matrix', dist_matrix, 'nearest_agent', nearest_agent);
end
```

### 6.4 coverage_quality.m

```matlab
function H = coverage_quality(agent_positions, sample_data, t, params)
% 计算覆盖质量函数 H = E[φ(x,t)·min_i‖x-p_i‖²]
% 复用Lloyd迭代中的采样数据，避免重复计算

    if nargin >= 2 && ~isempty(sample_data)
        % 复用采样数据
        phi = sample_data.phi;
        min_dist = min(sample_data.dist_matrix, [], 2);
    else
        % 独立计算（用于对比方法）
        N = params.algorithm.sample_num;
        domain = params.domain;
        samples = zeros(N, 3);
        samples(:,1) = domain.xmin + (domain.xmax - domain.xmin) * rand(N, 1);
        samples(:,2) = domain.ymin + (domain.ymax - domain.ymin) * rand(N, 1);
        samples(:,3) = domain.zmin + (domain.zmax - domain.zmin) * rand(N, 1);
        phi = compute_density(samples(:,1), samples(:,2), samples(:,3), t, params);
        min_dist = min(pdist2(samples, agent_positions), [], 2);
    end

    % 蒙特卡罗估计
    domain_vol = (params.domain.xmax-params.domain.xmin) * ...
                 (params.domain.ymax-params.domain.ymin) * ...
                 (params.domain.zmax-params.domain.zmin);
    H = domain_vol / numel(phi) * sum(phi .* min_dist.^2);
end
```

### 6.5 控制增益选取

| 增益 | 范围 | 效果 |
|------|------|------|
| k（反馈） | 0.1-1.0 | 过大→振荡超调；过小→收敛慢 |
| γ_ff（前馈） | 0.1-0.5 | 增大→更好地跟踪时变质心；过大→对采样噪声敏感 |
| 建议组合 | k=0.5, γ_ff=0.3 | 兼顾收敛速度与鲁棒性 |

---

## 七、模块六：对比方法（comparison_methods/）

### 7.1 静态均匀覆盖（static_coverage.m）

```matlab
function positions = static_coverage(initial_positions, ~, ~)
% 静态覆盖：位置不随时间变化
    positions = initial_positions;
end
```

### 7.2 随机覆盖（random_coverage.m）

```matlab
function new_positions = random_coverage(positions, ~, params)
% 随机覆盖：各向同性随机游走
    n = size(positions, 1);
    v_max = params.agent.max_speed;
    dt = params.algorithm.dt;

    % 随机方向（三维球面均匀分布）
    random_dir = randn(n, 3);
    random_dir = random_dir ./ vecnorm(random_dir, 2, 2);

    % 随机速度
    random_speed = v_max * rand(n, 1);

    new_positions = positions + random_dir .* random_speed * dt;

    % 域边界反弹
    domain = params.domain;
    new_positions(:,1) = max(domain.xmin, min(domain.xmax, new_positions(:,1)));
    new_positions(:,2) = max(domain.ymin, min(domain.ymax, new_positions(:,2)));
    new_positions(:,3) = max(domain.zmin, min(domain.zmax, new_positions(:,3)));
end
```

### 7.3 对比实验设计

| 方法 | 初始位置 | 控制策略 | 预期 H(t) 行为 |
|------|----------|----------|----------------|
| **自适应覆盖** | 随机 | Lloyd + 时变密度 + 前馈 | 快速下降→收敛至低值 |
| **静态均匀覆盖** | 均匀网格 | 固定不动 | 随羽流扩散上升 |
| **随机覆盖** | 随机 | 各向同性随机游走 | 高值波动，无改善趋势 |

**验证指标**：
1. 覆盖质量 H(t) 的收敛速度和稳态值
2. 智能体最终分布与羽流高浓度区域的空间匹配度
3. 各智能体的负载均衡程度（工作量的标准差）

---

## 八、模块七：主仿真脚本（main_simulation.m）

### 8.1 完整仿真流程

```matlab
function main_simulation()
    %% 1. 初始化参数
    params = init_parameters();

    %% 2. 初始化智能体位置（固定随机种子保证可重复性）
    rng(42);
    init_pos_adaptive = init_agents(params);
    init_pos_static   = init_agents_uniform(params);
    init_pos_random   = init_pos_adaptive;  % 同一随机初始

    pos_adaptive = init_pos_adaptive;
    pos_static   = init_pos_static;
    pos_random   = init_pos_random;

    %% 3. 记录数组
    steps = params.sim.total_time;
    H_adaptive = zeros(1, steps);
    H_static   = zeros(1, steps);
    H_random   = zeros(1, steps);
    trajectory_adaptive = zeros(steps, params.agent.num, 3);

    prev_centroids = [];  % 前馈用

    %% 4. 主仿真循环
    for step = 1:steps
        t = step * params.algorithm.dt;

        % --- 自适应覆盖（含前馈） ---
        [pos_adaptive, centroids, move_dist, sample_data] = ...
            lloyd_iteration(pos_adaptive, t, prev_centroids, params);
        prev_centroids = centroids;
        H_adaptive(step) = coverage_quality(pos_adaptive, sample_data, t, params);
        trajectory_adaptive(step, :, :) = pos_adaptive;

        % --- 静态覆盖 ---
        H_static(step) = coverage_quality(pos_static, [], t, params);

        % --- 随机覆盖 ---
        pos_random = random_coverage(pos_random, t, params);
        H_random(step) = coverage_quality(pos_random, [], t, params);

        % --- 进度输出 ---
        if mod(step, 20) == 0
            fprintf('Step %d/%d, t=%.1fs\n', step, steps, t);
            fprintf('  Adaptive H=%.2f, Static H=%.2f, Random H=%.2f, Move=%.3fm\n', ...
                H_adaptive(step), H_static(step), H_random(step), move_dist);
        end
    end

    %% 5. 结果可视化
    plot_results(H_adaptive, H_static, H_random, params);

    % 最终状态三维可视化
    plume_state = update_plume(t, params);
    plot_plume_agents_3d(pos_adaptive, trajectory_adaptive, plume_state, params);
    plot_voronoi_3d(pos_adaptive, t, params);
end
```

### 8.2 init_agents.m

```matlab
function positions = init_agents(params)
% 在域内随机生成智能体初始位置
    n = params.agent.num;
    domain = params.domain;
    positions = zeros(n, 3);
    positions(:,1) = domain.xmin + (domain.xmax - domain.xmin) * rand(n, 1);
    positions(:,2) = domain.ymin + (domain.ymax - domain.ymin) * rand(n, 1);
    positions(:,3) = domain.zmin + (domain.zmax - domain.zmin) * rand(n, 1);
end
```

### 8.3 init_agents_uniform.m

```matlab
function positions = init_agents_uniform(params)
% 在域内均匀网格分布智能体
    n = params.agent.num;
    domain = params.domain;
    nx = ceil(n^(1/3));
    ny = nx;
    nz = ceil(n / (nx*ny));
    x = linspace(domain.xmin*0.8, domain.xmax*0.8, nx);
    y = linspace(domain.ymin*0.8, domain.ymax*0.8, ny);
    z = linspace(domain.zmin*0.8, domain.zmax*0.8, nz);
    [X, Y, Z] = ndgrid(x, y, z);
    grid_points = [X(:), Y(:), Z(:)];
    idx = randperm(size(grid_points,1), n);
    positions = grid_points(idx, :);
end
```

---

## 九、模块八：可视化（visualization/）

### 9.1 plot_results.m — 覆盖质量对比曲线

```matlab
function plot_results(H_adaptive, H_static, H_random, params)
    figure('Name', '覆盖质量对比', 'Position', [100 100 800 500]);

    t = (1:params.sim.total_time) * params.algorithm.dt;

    % 原始曲线
    plot(t, H_adaptive, 'b-', 'LineWidth', 2); hold on;
    plot(t, H_static, 'r--', 'LineWidth', 1.5);
    plot(t, H_random, 'g:', 'LineWidth', 1.5);

    % 趋势拟合（自适应覆盖）
    p = polyfit(t, H_adaptive, 5);
    plot(t, polyval(p, t), 'b-.', 'LineWidth', 1, 'Color', [0.5 0.5 1]);

    xlabel('时间 (s)'); ylabel('覆盖质量 H(t)');
    legend('自适应覆盖', '静态均匀覆盖', '随机覆盖', '自适应趋势拟合', 'Location', 'best');
    title('三种覆盖方法的覆盖质量对比');
    grid on;
end
```

### 9.2 plot_plume_agents_3d.m — 三维羽流+智能体+轨迹

```matlab
function plot_plume_agents_3d(positions, trajectory, plume_state, params)
    figure('Name', '自适应覆盖最终状态', 'Position', [100 100 900 700]);

    % 羽流等值面
    threshold = max(plume_state.C(:)) * 0.1;
    [faces, verts] = isosurface(plume_state.X, plume_state.Y, plume_state.Z, plume_state.C, threshold);
    if ~isempty(faces)
        patch('Faces', faces, 'Vertices', verts, ...
              'FaceAlpha', 0.2, 'FaceColor', [1 0.5 0], 'EdgeColor', 'none');
    end
    hold on;

    % 智能体轨迹（淡色线）
    n_agents = size(trajectory, 2);
    colors = lines(n_agents);
    for i = 1:n_agents
        traj = squeeze(trajectory(:, i, :));
        plot3(traj(:,1), traj(:,2), traj(:,3), '-', 'Color', [colors(i,:) 0.3], 'LineWidth', 0.5);
    end

    % 最终位置
    scatter3(positions(:,1), positions(:,2), positions(:,3), 120, 'b', 'filled', ...
             'MarkerEdgeColor', 'k', 'LineWidth', 1.5);

    % 源位置
    scatter3(params.plume.source_pos(1), params.plume.source_pos(2), params.plume.source_pos(3), ...
             200, 'r', 'p', 'filled', 'MarkerEdgeColor', 'k');

    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('自适应覆盖 — 最终状态与轨迹');
    axis equal; view(3); grid on;
    legend('羽流等值面 (10%峰值)', '', 'AUV最终位置', '溢油源');
end
```

### 9.3 plot_voronoi_3d.m — 三维Voronoi区域可视化（采样着色法）

```matlab
function plot_voronoi_3d(agent_positions, t, params)
% 基于采样点的Voronoi区域着色可视化
    N_vis = 20000;
    domain = params.domain;
    samples = zeros(N_vis, 3);
    samples(:,1) = domain.xmin + (domain.xmax - domain.xmin) * rand(N_vis, 1);
    samples(:,2) = domain.ymin + (domain.ymax - domain.ymin) * rand(N_vis, 1);
    samples(:,3) = domain.zmin + (domain.zmax - domain.zmin) * rand(N_vis, 1);

    [~, nearest] = min(pdist2(samples, agent_positions), [], 2);

    figure('Name', '三维Voronoi剖分', 'Position', [100 100 900 700]);
    hold on;
    colors = lines(size(agent_positions, 1));
    for i = 1:size(agent_positions, 1)
        mask = (nearest == i);
        scatter3(samples(mask,1), samples(mask,2), samples(mask,3), 5, colors(i,:), '.');
    end
    scatter3(agent_positions(:,1), agent_positions(:,2), agent_positions(:,3), 120, 'k', 'filled');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('三维Voronoi区域划分');
    axis equal; view(3); grid on;
end
```

---

## 十、关键实现注意事项

### 10.1 性能优化策略

| 问题 | 解决方案 |
|------|----------|
| pdist2 重复计算 | Lloyd迭代返回采样数据，coverage_quality 直接复用 |
| 采样点数过多 | 自适应采样：高密度区域加密采样（可选） |
| 仿真速度慢 | 先用 sample_num=10000 快速调试，确认后增至 50000+ |
| 羽流网格更新开销 | 仅每隔 plume_update_interval 步更新采样网格 |

### 10.2 数值稳定性

| 问题 | 防护措施 |
|------|----------|
| 密度函数含零值 | β > 0 保证全域正密度 |
| 扩散参数趋零 | `sy = max(sy, sigma_y0)` 下界保护 |
| 质心计算除零 | 空单元退化为智能体自身位置 |
| 智能体重叠 | 初始化时加最小间距检查（可选） |

### 10.3 参数调优指南

| 参数 | 建议范围 | 调优方向 |
|------|----------|----------|
| control_gain k | 0.1-1.0 | 先0.3，观察收敛后增大 |
| ff_gain γ_ff | 0.1-0.5 | 从0开始（纯反馈），逐步增大观察改善 |
| density.alpha | 5-50 | 越大越聚集，推荐20 |
| density.beta | 0.05-0.5 | 越大越均匀，推荐0.1 |
| sample_num | 10000-100000 | 调试用10000，最终用50000+ |
| dt | 0.5-2.0 | 需满足 dt < d_min/v_max |

### 10.4 预期结果

1. **自适应覆盖**：H(t) 快速下降→平稳跟踪时变最优（前馈补偿减小跟踪误差）
2. **静态均匀覆盖**：H(t) 随羽流扩散而上升（固定位置不再匹配动态密度）
3. **随机覆盖**：H(t) 在较高值波动，无明显改善趋势
4. **空间分布**：自适应智能体最终聚集在源下游的高浓度区域

---

## 十一、Step8：动态扩散边界追踪增强

### 11.1 增强目标

在原有三维Voronoi覆盖控制基础上，加入更明确的时变扩散边界监测能力：

1. 溢油浓度场随时间平流、扩散、衰减，扩散范围动态扩大
2. AUV继续按照三维Voronoi单元的密度加权质心移动
3. 每个AUV额外追踪分配给自己的扩散边界采样点
4. 可视化中每个AUV使用独立颜色，并在图例中标注

### 11.2 动态扩散模型

当前 `gaussian_plume_3d.m` 使用三维时变高斯羽流：

$$C(x,y,z,t)=\frac{Q(1-e^{-t/8})e^{-\lambda t}}{(2\pi)^{3/2}\sigma_x\sigma_y\sigma_z}
\exp\left(-\frac{(x-x_c(t))^2}{2\sigma_x^2}\right)
\exp\left(-\frac{(y-y_s)^2}{2\sigma_y^2}\right)
\left[\exp\left(-\frac{(z-z_s)^2}{2\sigma_z^2}\right)+\exp\left(-\frac{(z+z_s)^2}{2\sigma_z^2}\right)\right]g_f(x,t)$$

其中：

- $x_c(t)=x_s+0.55ut$ 表示羽流主体随洋流向下游平移
- $\sigma_x,\sigma_y,\sigma_z$ 随下游距离和时间增大，表示范围动态扩大
- $e^{-\lambda t}$ 表示自然衰减
- $g_f(x,t)$ 为平滑前缘函数，避免扩散前缘出现硬截断

### 11.3 边界追踪控制项

`sample_plume_boundary.m` 根据阈值 $C \approx \eta C_{max}$ 采样扩散边界，并将边界点分配给最近AUV。`lloyd_iteration.m` 的控制律更新为：

$$v_i = k(c_i-p_i)+\gamma_{ff}\dot c_i+k_b(b_i-p_i)$$

其中：

- $c_i$ 是第 $i$ 个Voronoi单元的密度加权质心
- $\dot c_i$ 是质心速度前馈
- $b_i$ 是分配给第 $i$ 个AUV的边界采样点平均位置
- $k_b$ 是边界追踪增益

该控制律使AUV同时兼顾高浓度覆盖和扩散边界实时监测。

### 11.4 Step8 输出

`main_simulation.m` 和 `tests/test_step8_dynamic_tracking.m` 会生成：

| 输出文件 | 说明 |
|----------|------|
| `测试结果/Step8_动态扩散边界追踪/step8_coverage_quality.png` | 三种覆盖方法质量曲线 |
| `测试结果/Step8_动态扩散边界追踪/step8_plume_agents_tracking.png` | 动态羽流、彩色AUV轨迹和最终位置 |
| `测试结果/Step8_动态扩散边界追踪/step8_voronoi_3d.png` | 三维Voronoi分割结果 |
| `测试结果/Step8_动态扩散边界追踪/step8_boundary_tracking.png` | 羽流有效体积和边界距离指标 |
| `测试结果/Step8_动态扩散边界追踪/结果分析.md` | PASS/FAIL、关键数值分析和图片说明 |

---

## 十二、后期扩展方向（CBF安全约束）

当基础版本验证通过后，可参考 Liu et al. [1] 和 Collision-Aware Density-Driven [8] 加入控制障碍函数（CBF）安全约束：

### 12.1 安全约束定义

**避碰CBF**（智能体间）：

$$h_{ij}(\mathbf{p}) = \|p_i - p_j\|^2 - d_{\min}^2, \quad \forall i < j$$

**深度CBF**（边界约束）：

$$h_{i}^{z}(\mathbf{p}) = \min(z_i - z_{\min},\; z_{\max} - z_i)$$

### 12.2 CBF-QP安全滤波器

将Lloyd控制律 $u_0$ 作为名义输入，通过二次规划求解安全修正控制：

$$\min_{u} \|u - u_0\|^2 \quad \text{s.t.} \quad \dot{h}_k + \alpha_k h_k \geq 0, \quad \forall k$$

使用 MATLAB `quadprog` 求解。

### 12.3 扩展步骤

1. 在 `lloyd_iteration.m` 中将Lloyd输出作为名义控制 $u_0$
2. 新增 `cbf_filter.m` 模块，构建 CBF 约束矩阵
3. 每步调用 `quadprog` 求解安全控制输入
4. 对比有/无CBF时的覆盖质量与安全指标

---

## 十三、参考文献

[1] Liu W, et al. Safe 3D Coverage Control for Multi-Agent Systems[J]. Actuators, 2025, 14(4): 186. DOI: 10.3390/act14040186

[2] Bao B, Cortes J, Martinez S. Distributed Time-Varying Coverage Control via Singular Perturbations[J]. arXiv:2512.02163, 2025.

[3] Du Q, Emelianenko M, Ju L. Convergence of the Lloyd Algorithm for Computing Centroidal Voronoi Tessellations[J]. SIAM Journal on Numerical Analysis, 2006, 44(1): 102-119.

[4] Cortes J, Martinez S, Karatas T, Bullo F. Coverage Control for Mobile Sensing Networks[J]. IEEE Transactions on Robotics and Automation, 2004, 20(2): 243-255.

[5] John Burkardt. cvt_3d_sampling: CVT in 3D using Sampling. https://people.sc.fsu.edu/~jburkardt/m_src/cvt_3d_sampling/cvt_3d_sampling.m

[6] An Overview of Oil Spill Modeling and Simulation for Surface and Subsurface Applications[J]. Eng., 2022, 3(4): 29. DOI: 10.3390/eng3040029

[7] The Influence of Oil Leaking Rate and Ocean Current Velocity on the Migration and Diffusion of Underwater Oil Spill[J]. Scientific Reports, 2020, 10: 7983. DOI: 10.1038/s41598-020-66046-1

[8] Collision-Aware Density-Driven Control of Multi-Agent Systems via Control Barrier Functions[J]. arXiv:2512.10392, 2025.

[9] Optimal Transport for Time-Varying Multi-Agent Coverage Control[J]. arXiv:2601.21753, 2026.

[10] Cooperative Coverage Control for Heterogeneous AUVs Based on Control Barrier Functions and Consensus Theory[J]. Sensors, 2026, 26(3): 822.
