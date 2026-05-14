# 题目一：基于时变密度函数的三维Voronoi水下溢油羽流自适应覆盖控制 — MATLAB实现流程

## 研究背景

- **研究方向**：三维Voronoi覆盖控制算法
- **研究对象**：水下多智能体（8个AUV）对溢油羽流的自适应覆盖监测
- **目标会议**：中国自动化大会（CAC）
- **仿真工具**：MATLAB 2024b
- **安全约束**：本期不含CBF，纯覆盖控制（后期扩展）
- **对比实验**：Proposed CVT-DBT vs Standard CVT vs Lawnmower CPP

### 问题建模

考虑 $n$ 个AUV在三维有界域 $\mathcal{D} \subset \mathbb{R}^3$ 中对时变溢油羽流进行覆盖监测。域内存在时变密度函数 $\phi(x,t)$，由水下溢油羽流浓度场驱动。控制目标是最小化**位置感知代价函数**（locational cost）：

$$\mathcal{H}(p_1,\ldots,p_n,t) = \sum_{i=1}^{n} \int_{V_i} \phi(x,t) \cdot \|x - p_i\|^2 \, dx$$

其中 $V_i$ 为第 $i$ 个智能体的三维Voronoi单元，$p_i$ 为其三维位置。该代价函数衡量了覆盖配置与密度分布的匹配程度。

### 系统本质说明

**本系统为三维体积覆盖控制**，而非二维表面覆盖。具体而言：

- AUV位置为三维坐标 $p_i = [x_i, y_i, z_i]^T$，可在整个三维水域体积 $\Omega = [x_{min}, x_{max}] \times [y_{min}, y_{max}] \times [z_{min}, z_{max}]$ 内自由移动
- Voronoi划分基于三维欧氏距离：$V_i(t) = \{ q \in \Omega \mid \|q - p_i(t)\| \leq \|q - p_j(t)\|, \forall j \neq i \}$
- 每个Voronoi单元 $V_i(t)$ 是三维体积区域，而非二维面片或曲面
- 负载定义为三维体积分：$M_i(t) = \int\!\!\int\!\!\int_{V_i(t)} \phi(x,y,z,t) \, dx\,dy\,dz$
- 密度加权质心为三维向量：$c_i(t) = \frac{\int_{V_i(t)} q\,\phi(q,t)\,dq}{\int_{V_i(t)} \phi(q,t)\,dq}$，其中 $q = [x,y,z]^T$

**关于负载均衡**：当前方法具备基于密度加权Voronoi的隐式任务分配能力，但控制律以覆盖质心和边界追踪为主导目标，没有显式引入负载差反馈项（如 $M_i - \bar{M}$），因此属于覆盖驱动的任务分配，而非严格负载均衡控制。

---

## 一、整体代码架构

```
main_simulation.m              % 主仿真脚本（入口）
│
├── init_parameters.m          % 全局参数初始化（含预泄漏时间）
├── init_agents.m              % 溢油源附近监测环初始化
├── init_agents_uniform.m      % 均匀网格初始位置（静态覆盖用）
│
├── plume_model/               % 溢油羽流模型模块
│   ├── gaussian_plume_3d.m    % 持续泄漏三维对流-扩散-衰减浓度场
│   ├── update_plume.m         % 羽流浓度场网格采样与更新
│   └── estimate_plume_extent.m % 羽流有效体积估计
│
├── density_function/          % 密度函数模块
│   └── compute_density.m      % 浓度→覆盖密度函数 φ(x,t) = α·C_norm + β
│
├── coverage_control/          % 覆盖控制算法模块
│   ├── compute_centroid.m     % 重要性采样Voronoi密度加权质心计算
│   ├── lloyd_iteration.m      % Lloyd迭代（质心反馈+前馈+边界追踪+分散）
│   ├── coverage_quality.m     % 覆盖质量函数 H(t) 计算（统一均匀采样估计）
│   ├── sample_plume_boundary.m % 溢油边界采样与均衡分配
│   ├── compute_monitoring_fraction.m % 边界监测覆盖率
│   ├── compute_dynamic_coverage_ratio.m % 动态覆盖率 CR(t)
│   └── compute_boundary_rmse.m % 边界追踪RMSE
│
├── comparison_methods/        % 基准算法模块
│   ├── standard_cvt.m         % 标准CVT（纯密度加权质心反馈，展示集群塌陷）
│   └── lawnmower_coverage.m   % 三维割草机路径规划（预规划开环控制）
│
├── visualization/             % 可视化模块
│   ├── plot_results.m         % 覆盖质量对比曲线（平滑）
│   ├── plot_coverage_metrics.m % 动态覆盖率+边界RMSE双面板（平滑）
│   ├── plot_control_input.m   % AUV实时速度/控制输入曲线（平滑）
│   ├── plot_plume_agents_3d.m % 三维羽流+彩色AUV轨迹
│   ├── plot_voronoi_3d.m      % 三维Voronoi透明区域面片（自适应渲染）
│   ├── plot_boundary_tracking.m % 边界追踪+RMSE指标（修复图例）
│   ├── plot_monitoring_dashboard.m % 2×4综合监测仪表盘
│   ├── create_dynamic_plume_gif.m  % 动态扩散GIF生成
│   └── smooth_trajectory.m    % AUV轨迹平滑（可视化用）
│
├── write_step8_analysis.m     % Step8结果分析文档生成
│
└── tests/
    └── test_step8_dynamic_tracking.m  % 唯一测试入口
```

### 架构设计原则

1. **模块化**：每个功能独立成函数，通过 `params` 结构体传递参数
2. **数据流清晰**：`plume_model → density_function → coverage_control` 单向依赖
3. **采样复用**：Lloyd迭代与覆盖质量计算共享采样点和距离矩阵，避免重复计算
4. **时间双轨制**：监测时间 $t_m$（AUV视角）与泄漏物理时间 $t_l = t_0 + t_m$（羽流视角）分离

---

## 二、模块一：参数初始化（init_parameters.m）

### 2.1 功能说明

定义仿真所需的所有全局参数，统一用 `struct` 组织。所有物理量采用国际单位制（SI）。

### 2.2 关键参数

```matlab
function params = init_parameters()

%% === 场景参数 ===
params.domain = struct( ...
    'xmin', 30,   'xmax', 390, ...
    'ymin', -160, 'ymax', 160, ...
    'zmin', -260, 'zmax', 0);

%% === 智能体参数 ===
params.agent = struct( ...
    'num', 8, ...
    'max_speed', 3.5, ...
    'control_gain', 0.28, ...
    'sense_radius', 50);

%% === 溢油羽流参数（持续泄漏三维对流-扩散-衰减模型） ===
params.plume = struct( ...
    'source_pos', [150, 0, -180], ...
    'Q', 100, ...
    'u_current', 0.5, ...
    'w_buoyancy', 0.22, ...
    'sigma_x0', 28, ...
    'sigma_y0', 16, ...
    'sigma_z0', 14, ...
    'diffusion_x', 1.20, ...
    'diffusion_y', 2.60, ...
    'diffusion_z', 1.10, ...
    'decay_rate', 0.0006, ...
    'release_substeps', 28, ...
    'front_smoothing', 45, ...
    'boundary_threshold', 0.035);

%% === 密度函数参数 ===
params.density = struct( ...
    'alpha', 10.0, ...
    'beta', 0.03);

%% === 算法参数 ===
params.algorithm = struct( ...
    'sample_num', 50000, ...
    'dt', 1.0, ...
    'ff_gain', 0.12, ...
    'boundary_gain', 1.10, ...
    'boundary_sample_num', 12000);

%% === 仿真参数 ===
params.sim = struct( ...
    'total_time', 200, ...
    'pre_release_time', 80, ...
    'plume_update_interval', 5, ...
    'sample_num_test', 5000);
```

### 2.3 参数物理依据

| 参数 | 值 | 物理依据 |
|------|-----|---------|
| 仿真域 360×320×260 m | 适当扩大使羽流有充分扩散空间，避免等值面被边界截断 |
| AUV最大速度 3.5 m/s | 典型AUV（如Bluefin-21）巡航速度 1-3 m/s，略提高保证动态追踪 |
| AUV感知半径 50 m | 典型AUV声纳探测距离，用于覆盖质量指标和边界监测判定 |
| 溢油源 [150, 0, -180] m | 位于域内中部水深，向下游和海面方向均有充分扩散空间 |
| 源强度 100 kg/s | 中等规模海底管道泄漏 |
| 主流速度 0.5 m/s | 近海典型表层/中层洋流 0.1-1.0 m/s |
| 浮力上升速度 0.22 m/s | 油滴在水中浮力上升的典型速度 |
| 扩散系数 $D_x=1.20, D_y=2.60, D_z=1.10$ m²/s | 各向异性湍流扩散，横向 > 垂向（密度分层抑制垂向混合） |
| 衰减率 0.0006 /s | 生物降解、溶解等自然衰减过程 |
| 预泄漏时间 80 s | AUV开始监测时溢油源已持续泄漏80秒，初始即有可见羽流 |
| 控制增益 k=0.28 | 兼顾收敛速度与稳定性 |
| 边界追踪增益 1.10 | 使AUV贴近扩散边界进行实时监测 |

---

## 三、模块二：持续泄漏三维溢油羽流模型（plume_model/）

### 3.1 数学模型

采用**非稳态三维对流-扩散-衰减高斯羽流模型**描述水下溢油扩散。模型同时考虑：主流沿下游输运、横向/垂向湍流扩散、油滴浮力上升、自然衰减，以及海面零通量边界的镜像反射。

**控制方程**：

$$\frac{\partial C}{\partial t}+u\frac{\partial C}{\partial x}+w_b\frac{\partial C}{\partial z}=D_x\frac{\partial^2 C}{\partial x^2}+D_y\frac{\partial^2 C}{\partial y^2}+D_z\frac{\partial^2 C}{\partial z^2}-\lambda C+Q\delta(x-x_s,y-y_s,z-z_s)$$

其中 $u$ 为下游海流速度，$w_b$ 为浮力上升速度，$D_x,D_y,D_z$ 为各向异性湍流扩散系数，$\lambda$ 为衰减系数。

**持续泄漏浓度场公式（时间卷积积分）**：

$$C(x,y,z,t)=\int_0^t \frac{Qe^{-\lambda(t-\tau)}}{(2\pi)^{3/2}\sigma_x(a)\sigma_y(a)\sigma_z(a)}\exp\left[-\frac{(x-x_c(a))^2}{2\sigma_x^2(a)}-\frac{(y-y_s)^2}{2\sigma_y^2(a)}\right]$$

$$\times\left\{\exp\left[-\frac{(z-z_c(a))^2}{2\sigma_z^2(a)}\right]+\exp\left[-\frac{(z+z_c(a))^2}{2\sigma_z^2(a)}\right]\right\}G_d(x)\,d\tau,\quad a=t-\tau$$

其中：
- $\tau$：历史释放时刻，$a=t-\tau$ 为该油团年龄
- $x_c(a)=x_s+ua$：每一历史油团随主流向下游输运
- $z_c(a)=\min(0,z_s+w_ba)$：每一历史油团受浮力沿 $z$ 正方向上升，不超过海面
- $\sigma_i(a)=\sqrt{\sigma_{i0}^2+2D_i a}$：历史油团随年龄扩散
- $G_d(x)=1/(1+\exp(-(x-x_s)/s_d))$：平滑下游门控，避免源点上游出现不合理高浓度
- 镜像项 $\exp[-(z+z_c)^2/(2\sigma_z^2)]$ 表示海面（$z=0$）零通量近似

**时间双轨制**：主仿真采用监测时间与泄漏物理时间分离的设定。AUV开始覆盖时刻记为 $t_m=0$，此时溢油源已经持续泄漏 $t_0=80\,\mathrm{s}$。羽流模型实际使用物理泄漏时间：

$$t_{leak} = t_0 + t_m$$

因此GIF第一帧和最终结果图在监测开始时已经存在可见扩散区域，随后AUV对已形成且继续扩大的羽流进行覆盖监测。

代码中使用中点积分离散该时间卷积（`release_substeps = 28` 个积分子步），以保证"持续溢油"是历史泄漏贡献的累积，而非单个高斯团的近似放大。

### 3.2 gaussian_plume_3d.m

```matlab
function C = gaussian_plume_3d(x, y, z, t, params)
% 持续泄漏的三维对流-扩散-衰减羽流浓度场
% 输入:
%   x, y, z: 标量或同维数组，空间坐标 (m)
%   t: 当前物理泄漏时间 (s)
%   params: 参数结构体
% 输出:
%   C: 浓度值 (kg/m^3)

    src = params.plume.source_pos;
    Q = params.plume.Q;
    u = params.plume.u_current;
    w = params.plume.w_buoyancy;
    % ... 提取扩散系数、初始sigma、衰减率等参数 ...

    tau = max(t, 0);
    C = zeros(size(x));
    if tau <= 0, return; end

    % 中点积分离散时间卷积
    release_times = linspace(0, tau, n_release + 1);
    release_times = 0.5 * (release_times(1:end-1) + release_times(2:end));
    dtau = tau / n_release;

    for r = release_times
        age = tau - r;
        center_x = src(1) + u * age;          % 下游输运
        center_z = min(0, src(3) + w * age);   % 浮力上升

        sigma_x = sqrt(sx0^2 + 2 * Dx * age);  % 随年龄扩散
        sigma_y = sqrt(sy0^2 + 2 * Dy * age);
        sigma_z = sqrt(sz0^2 + 2 * Dz * age);

        kernel = exp(-(x-center_x).^2./(2*sigma_x^2) ...
                     -(y-center_y).^2./(2*sigma_y^2));
        vertical = exp(-(z-center_z).^2./(2*sigma_z^2)) + ...
                   exp(-(z+center_z).^2./(2*sigma_z^2));  % 海面镜像
        downstream_gate = 1./(1+exp(-(x-src(1))./front_smoothing));
        amplitude = Q*dtau*exp(-lambda*age) ...
                    / ((2*pi)^(3/2)*sigma_x*sigma_y*sigma_z);
        C = C + amplitude .* kernel .* vertical .* downstream_gate;
    end
    C = max(C, 0);
end
```

### 3.3 模型说明

- **持续泄漏**：每个历史时刻释放的油团独立经历输运、扩散、浮力上升和衰减，浓度是所有历史油团的叠加
- **海面镜像**：公式中第二指数项模拟海面（$z=0$）对羽流的反射效应
- **下游门控**：$G_d(x)$ 平滑过渡，避免源点上游出现不合理高浓度
- **预泄漏设定**：监测开始时 $t_m=0$ 对应物理泄漏时间 $t_{leak}=80\,\mathrm{s}$，初始即有可见羽流

---

## 四、模块三：时变密度函数（density_function/）

### 4.1 数学模型

将溢油浓度场映射为覆盖控制的时变密度函数：

$$\phi(x, t) = \alpha \cdot C_{\text{norm}}(x, t) + \beta$$

其中：
- $C_{\text{norm}}(x,t) = \min\!\left(\frac{C(x,t)}{C_{\max}^{\text{est}}},\, 1\right)$：归一化浓度 ∈ [0,1]
- $\alpha=10.0$：浓度权重系数
- $\beta=0.03 > 0$：基底密度，保证全域正密度

**密度函数性质**：
- 处处为正：$\phi(x,t) \geq \beta > 0$
- 时变性：随羽流演化而变化，驱动智能体持续自适应调整
- 与控制目标的联系：密度越高的区域，对应Voronoi单元越小（更多智能体覆盖该区域）

---

## 五、模块四：三维Voronoi剖分与质心计算（coverage_control/）

### 5.1 方法选择：基于重要性采样的Lloyd算法

本实现采用**混合重要性采样**策略：

- 45%均匀采样：保证全域覆盖，避免遗漏低浓度区域
- 55%羽流区域加密采样：以当前羽流中心为均值、扩散尺度加偏移量为标准差的高斯采样

采样密度函数：

$$g(q) = p_{uniform} \cdot \frac{1}{V_\Omega} + (1 - p_{uniform}) \cdot \mathcal{N}(q; \mu(t), \Sigma(t))$$

重要性权重：

$$w(q) = \frac{\phi(q,t)}{g(q)}$$

### 5.2 compute_centroid.m — 质心计算

每个AUV的密度加权质心通过重要性采样估计：

$$c_i(t) = \frac{\sum_{q \in V_i} w(q) \cdot q}{\sum_{q \in V_i} w(q)}$$

其中 $V_i$ 为分配到第 $i$ 个AUV的采样点集合（最近邻分配，即三维Voronoi划分）。

### 5.3 关键实现要点

1. **采样点数**：50000，平衡精度与速度
2. **数据复用**：函数同时返回采样点、密度、距离矩阵和权重，供 `coverage_quality` 复用
3. **羽流跟踪采样**：采样中心随主流和浮力动态调整，始终覆盖有效羽流区域
4. **空单元处理**：若某AUV无采样点分配，质心退化为自身位置

---

## 六、模块五：Lloyd迭代与覆盖控制律（coverage_control/）

### 6.1 控制律设计

当前控制律由四项组成：

$$v_i = k(c_i - p_i) + \gamma_{\text{ff}} \dot{c}_i + k_b(b_i - p_i) + v_{\text{sep},i}$$

其中：
- **反馈项** $k(c_i - p_i)$：驱动AUV向密度加权质心移动（$k=0.28$）
- **前馈项** $\gamma_{\text{ff}} \dot{c}_i$：预测质心运动趋势，补偿密度场时变效应（$\gamma_{\text{ff}}=0.12$）
- **边界追踪项** $k_b(b_i - p_i)$：使AUV贴近分配的扩散边界（$k_b=1.10$）
- **分散项** $v_{\text{sep},i}$：防止AUV过度聚集，保持最小间距

质心速度近似：

$$\dot{c}_{V_i}(t) \approx \frac{c_{V_i}(t) - c_{V_i}(t - \Delta t)}{\Delta t}$$

### 6.2 边界追踪与均衡分配

`sample_plume_boundary.m` 根据阈值 $C \approx \eta C_{max}$ 采样扩散边界点，然后按角度均衡分配给所有AUV：

1. 计算每个AUV相对于羽流中心的角度 $\theta_i$
2. 计算每个边界点相对于羽流中心的角度 $\theta_b$
3. 按角度排序后，将边界点均匀分为 $n$ 个扇区
4. 每个AUV对应一个扇区的边界点

该策略保证所有 8 个AUV都参与边界监测，不会出现部分AUV远离有效区域的情况。

### 6.3 离散化实现

```
v_i = 反馈项 + 前馈项 + 边界追踪项 + 分散项
if ‖v_i‖ > v_max:
    v_i = v_max · v_i / ‖v_i‖        % 速度限幅
p_i(t+dt) = p_i(t) + v_i · dt
p_i = clip(p_i, domain)               % 域边界约束
```

### 6.4 coverage_quality.m — 覆盖质量计算

$$\mathcal{H}(t) = \int_\Omega \phi(q,t) \cdot \min_i \|q - p_i\|^2 \, dq$$

所有方法统一使用**均匀采样蒙特卡罗估计**确保公平对比：
$$H = \frac{V_\Omega}{N}\sum_{j=1}^{N} \phi_j \cdot d_j^2$$

> **注意**：Proposed方法在Lloyd迭代中内部使用重要性采样计算质心，但覆盖质量对比时统一使用均匀采样估计，避免不同估计器之间的数值偏差。

---

## 七、模块六：对比方法（comparison_methods/）

### 7.1 对比实验设计

| 方法 | 初始位置 | 控制策略 | 预期行为 |
|------|----------|----------|----------|
| **Proposed CVT-DBT** | 溢油源附近监测环 | Lloyd + 前馈 + 边界追踪 + 分散 | 兼顾核心覆盖和边界追踪，覆盖率保持高位 |
| **Standard CVT** | 同Proposed（公平对比） | 纯密度加权质心反馈 $v_i=\kappa(c_i-p_i)$ | 所有AUV向浓度峰值聚集（"集群塌陷"），覆盖率急剧下降 |
| **Lawnmower CPP** | 各扫描道起始位置 | 预规划三维锯齿路径（开环） | 固定扫描无法响应动态羽流，覆盖率不稳定 |

### 7.2 Standard CVT（standard_cvt.m）

标准质心Voronoi细分控制，仅使用密度加权质心反馈：

$$v_i = \kappa(c_i - p_i)$$

- 复用 `compute_centroid.m` 计算密度加权质心
- **去除**边界追踪、质心前馈、分离控制
- 速度限幅 $v_{max}$ + 域边界约束
- **预期效果**：所有AUV向羽流源头聚集，导致覆盖率随羽流扩散而暴跌

### 7.3 Lawnmower CPP（lawnmower_coverage.m）

三维割草机式预规划扫描路径，开环控制不响应羽流动态：

- **Y方向**：将域等分为 $n$ 条平行扫描道（每道 40m 宽），AUV固定在各道中心
- **X方向**：以 $0.75v_{max}$ 匀速前进，到达边界后三角波折返
- **Z方向**：三角波振荡（各AUV有相位偏移避免同步聚集）
- 接口：`new_positions = lawnmower_coverage(~, t, params)`，忽略位置输入（开环）

---

## 八、模块七：主仿真脚本（main_simulation.m）

### 8.1 完整仿真流程

```matlab
function main_simulation()
    params = init_parameters();
    rng(42);

    init_pos_proposed = init_agents(params);   % 溢油源附近监测环
    pos_proposed = init_pos_proposed;
    pos_cvt      = init_pos_proposed;          % 同Proposed（公平对比）
    pos_lawnmower = lawnmower_coverage([], pre_release, params); % 预规划起始

    % 主仿真循环：时间双轨制
    for step = 1:steps
        t_leak = pre_release + (step-1) * dt;

        % Proposed CVT-DBT
        [pos_proposed, centroids, ~, sd] = lloyd_iteration(pos_proposed, t_leak, ...);
        H_proposed(step) = coverage_quality(pos_proposed, [], t_leak, params);  % 统一均匀采样
        velocities(step, :) = |pos_new - pos_old| / dt;                         % 记录速度

        % Standard CVT（纯质心反馈）
        pos_cvt = standard_cvt(pos_cvt, t_leak, params);

        % Lawnmower CPP（预规划开环）
        pos_lawnmower = lawnmower_coverage(pos_lawnmower, t_leak, params);

        % 共享指标（单次update_plume调用，传入各指标函数）
        plume_state = update_plume(t_leak, params);
        CR_proposed(step)  = compute_dynamic_coverage_ratio(pos_proposed, params, plume_state);
        CR_cvt(step)       = compute_dynamic_coverage_ratio(pos_cvt, params, plume_state);
        CR_lawnmower(step) = compute_dynamic_coverage_ratio(pos_lawnmower, params, plume_state);
        RMSE_proposed(step)  = compute_boundary_rmse(pos_proposed, params, plume_state);
        RMSE_cvt(step)       = compute_boundary_rmse(pos_cvt, params, plume_state);
        RMSE_lawnmower(step) = compute_boundary_rmse(pos_lawnmower, params, plume_state);
    end

    % 可视化与结果保存（所有时间轴统一为泄漏时间）
    plot_results(...);              % 覆盖质量H(t)对比（平滑曲线）
    plot_coverage_metrics(...);     % CR(t) + RMSE双面板（平滑曲线）
    plot_control_input(...);        % AUV实时速度曲线（平滑）
    plot_plume_agents_3d(...);      % 三维羽流+AUV轨迹
    plot_voronoi_3d(...);           % 三维Voronoi区域
    plot_boundary_tracking(...);    % 边界追踪+RMSE（修复图例）
    plot_monitoring_dashboard(...); % 2×4综合仪表板
    create_dynamic_plume_gif(...);  % 动态GIF
    write_step8_analysis(...);      % 结果分析文档
end
```

### 8.2 init_agents.m — 监测环初始化

AUV初始位置围绕溢油源下游布置为三维环形，保证所有AUV从一开始就处于有效监测区域：

```matlab
x0 = src(1) + 45;     % 源下游45m
ry ≈ 37.5 m;          % 横向环半径（0.75 × R_s）
rz ≈ 27.5 m;          % 垂向环半径（0.55 × R_s）
positions(:,1) = x0 + 12*cos(theta + pi/n);
positions(:,2) = src(2) + ry*cos(theta);
positions(:,3) = src(3) + rz*sin(theta);
```

---

## 九、模块八：可视化（visualization/）

### 9.1 输出文件

| 输出文件 | 说明 |
|----------|------|
| `step8_coverage_quality.png` | 三种方法覆盖质量H(t)对比（平滑曲线，Proposed vs Standard CVT vs Lawnmower） |
| `step8_coverage_metrics.png` | 动态覆盖率CR(t)和边界追踪RMSE双面板对比（平滑曲线） |
| `step8_control_input.png` | 8台AUV实时速度曲线和集群速度统计（平滑曲线，验证控制输入有界性） |
| `step8_plume_agents_tracking.png` | 三维羽流等值面、8个AUV彩色轨迹和最终位置 |
| `step8_voronoi_3d.png` | 三维Voronoi透明区域面片（自适应渲染，稀疏区域降级为散点） |
| `step8_boundary_tracking.png` | 羽流体积（左Y轴）+ 三种方法边界RMSE/距离（右Y轴），图例已修复 |
| `step8_monitoring_dashboard.png` | 2×4综合面板：覆盖质量、覆盖率、RMSE、速度、边界距离、扩散体积、巡航距离、速度统计 |
| `step8_dynamic_plume_monitoring.gif` | 动态扩散过程GIF |

### 9.2 三维Voronoi可视化

使用规则三维网格 + 最近AUV分配 + `smooth3` 平滑 + `isosurface` 生成等值面 + 透明 `patch` 显示，每个AUV负责区域使用与其颜色相同的透明色填充，而非点云。

### 9.3 动态GIF

GIF标题显示双时间信息：
```
动态溢油源扩散与AUV实时监测  监测t = 0 s，泄漏已持续 80 s
```

GIF中包含：
- 溢油源（红色五角星）
- 外层扩散边界（浅黄色半透明等值面）
- 中浓度羽流（橙色半透明等值面）
- 高浓度核心（深红色半透明等值面）
- 浓度纵切片（turbo色图）
- 扩散前沿线（黑色虚线，跟随主流和浮力）
- 8个AUV的彩色平滑轨迹和当前位置

---

## 十、测试与验证

### 10.1 测试入口

仅保留 `tests/test_step8_dynamic_tracking.m` 作为唯一测试脚本。该测试隐式验证了所有底层模块：

- `gaussian_plume_3d` — 持续泄漏羽流模型
- `compute_density` — 密度函数
- `compute_centroid` — 重要性采样质心
- `lloyd_iteration` — 控制律（含边界追踪和分散）
- `coverage_quality` — 覆盖质量计算
- `sample_plume_boundary` — 边界采样与分配
- `estimate_plume_extent` — 羽流体积估计
- 所有可视化函数

### 10.2 验证指标

| 指标 | 判据 | 说明 |
|------|------|------|
| 羽流有效体积增长 | $V_{end} > 1.5 \cdot V_{start}$ | 验证持续泄漏模型使扩散范围动态扩大 |
| 边界采样有效 | 所有边界距离有限 | 验证边界采样和均衡分配正常工作 |
| 平均边界距离阈值 | $\bar{d} \leq 1.05 \cdot R_s$ | AUV在感知范围内追踪扩散边界（$R_s=50$m） |
| 后期监测覆盖率 | $\geq 75\%$ | 后20步平均边界监测覆盖率达标 |
| 覆盖率优劣 | $CR_{Proposed} \geq CR_{Lawnmower}$ | Proposed方法动态覆盖率应优于Lawnmower |
| 边界RMSE优劣 | $RMSE_{Proposed} < RMSE_{CVT}$ | Proposed方法边界追踪RMSE应优于Standard CVT |
| 速度有界 | $v_i \leq v_{max}$ | 所有AUV速度未超出物理极限 |

### 10.3 运行方式

```matlab
cd('d:/science/3Dvoronoi')
% 注意：不要使用 addpath(genpath('.'))，因为 .claude/worktrees/ 下有旧版本副本
addpath('.', 'coverage_control', 'comparison_methods', 'density_function', ...
        'plume_model', 'visualization', 'tests')
test_step8_dynamic_tracking   % 自动测试（40步快速验证）
main_simulation                % 完整仿真（200步，含全部指标和可视化）
```

---

## 十一、后期扩展方向（CBF安全约束）

当基础版本验证通过后，可参考 Liu et al. [1] 和 Collision-Aware Density-Driven [8] 加入控制障碍函数（CBF）安全约束：

### 11.1 安全约束定义

**避碰CBF**（智能体间）：

$$h_{ij}(\mathbf{p}) = \|p_i - p_j\|^2 - d_{\min}^2, \quad \forall i < j$$

**深度CBF**（边界约束）：

$$h_{i}^{z}(\mathbf{p}) = \min(z_i - z_{\min},\; z_{\max} - z_i)$$

### 11.2 CBF-QP安全滤波器

将Lloyd控制律 $u_0$ 作为名义输入，通过二次规划求解安全修正控制：

$$\min_{u} \|u - u_0\|^2 \quad \text{s.t.} \quad \dot{h}_k + \alpha_k h_k \geq 0, \quad \forall k$$

使用 MATLAB `quadprog` 求解。

### 11.3 扩展步骤

1. 在 `lloyd_iteration.m` 中将Lloyd输出作为名义控制 $u_0$
2. 新增 `cbf_filter.m` 模块，构建 CBF 约束矩阵
3. 每步调用 `quadprog` 求解安全控制输入
4. 对比有/无CBF时的覆盖质量与安全指标

---

## 十二、参考文献

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
