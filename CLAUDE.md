# 3Dvoronoi 项目指令

## 编码约束

- 在逐步生成代码过程中，如果发现 `.md` 设计文档存在问题（如数学公式错误、参数不合理、接口不一致等），**必须及时修改文档**以保持代码与文档的一致性
- 代码必须严格遵循文档中的数学公式和接口设计
- MATLAB 版本为 **2024b**，可使用该版本支持的所有函数和语法
- 所有注释使用中文
- 测试脚本 `tests/test_step8_dynamic_tracking.m` 必须独立可运行，使用 assert 进行自动验证
- **完成实验后，必须生成测试数据说明**，包含：
  - 测试结果汇总（PASS/FAIL 统计）
  - 关键数值分析（是否符合物理预期）
  - 生成的图片说明（图中展示了什么、验证了什么）
- **结果分析保存在 `测试结果/Step8_动态扩散边界追踪/` 文件夹下**，包含：
  - `结果分析.md` — 测试结果汇总、数值分析表、是否符合预期的判断
  - 测试生成的图片（.png 格式，使用 `exportgraphics` 保存）
  - GIF 动画（.gif 格式，使用 `imwrite` 逐帧写入）
- **测试脚本中的可视化必须自动保存图片**：用 `exportgraphics(gcf, save_path)` 保存到对应目录，GIF 动画用 `imwrite` 逐帧写入
- **运行时注意**：不要使用 `addpath(genpath('.'))`，因为 `.claude/worktrees/` 下有旧版本副本会导致冲突。应使用显式路径添加

## 项目说明

- 项目仅保留 `test_step8_dynamic_tracking.m` 作为唯一测试入口，该测试已隐式覆盖所有底层模块（羽流模型、密度函数、质心计算、Lloyd控制律、覆盖质量对比、可视化）
- 主仿真入口为 `main_simulation.m`

## 对比实验

三种覆盖方法对比：

| 方法 | 算法 | 初始位置 | 特点 |
|------|------|----------|------|
| **Proposed CVT-DBT** | Lloyd迭代 + 质心前馈 + 边界追踪 + 分散 | 溢油源监测环 | 所提方法，兼顾核心覆盖和边界追踪 |
| **Standard CVT** | 纯密度加权质心反馈 v=κ(cen-p) | 同Proposed（公平对比） | 展示"集群塌陷"现象，所有AUV向浓度峰值聚集 |
| **Lawnmower CPP** | 预规划三维锯齿扫描路径（条带分解） | 各扫描道起始位置 | 开环控制，不响应羽流动态 |

## 评估指标

| 指标 | 公式 | 说明 |
|------|------|------|
| **覆盖质量 H(t)** | ∫φ·min_i‖x-p_i‖²dx | 位置感知代价，越小越好 |
| **动态覆盖率 CR(t)** | 羽流体积中AUV感知范围覆盖的比例 | 随时间变化曲线，展示持续覆盖能力 |
| **边界追踪RMSE** | √(1/n·Σmin_dist²) | AUV到阈值等值面距离误差，越低边界追踪越精确 |
| **AUV实时速度 v_i(t)** | Δp/Δt | 控制输入有界性验证 |

## 代码架构

```
main_simulation.m                          % 主仿真脚本（入口）
├── init_parameters.m                      % 全局参数（sense_radius=50m）
├── init_agents.m                          % Proposed方法监测环初始化
├── plume_model/                           % 羽流模型
│   ├── gaussian_plume_3d.m                % 持续泄漏对流-扩散-衰减
│   ├── update_plume.m                     % 网格采样
│   └── estimate_plume_extent.m            % 有效体积估计
├── density_function/
│   └── compute_density.m                  % φ = α·C_norm + β
├── coverage_control/                      % 核心算法
│   ├── compute_centroid.m                 % 重要性采样质心
│   ├── lloyd_iteration.m                  % Proposed CVT-DBT控制律
│   ├── coverage_quality.m                 % H(t)计算（统一均匀采样估计）
│   ├── sample_plume_boundary.m            % 边界采样与角度均衡分配
│   ├── compute_monitoring_fraction.m      % 边界监测比例
│   ├── compute_dynamic_coverage_ratio.m   % 动态覆盖率CR(t)
│   └── compute_boundary_rmse.m            % 边界追踪RMSE
├── comparison_methods/                    % 基准算法
│   ├── standard_cvt.m                     % 标准CVT（纯质心反馈）
│   └── lawnmower_coverage.m               % 三维割草机路径规划
├── visualization/
│   ├── plot_results.m                     % 覆盖质量H(t)对比（平滑）
│   ├── plot_coverage_metrics.m            % CR(t) + RMSE双面板（平滑）
│   ├── plot_control_input.m               % AUV速度/控制输入（平滑）
│   ├── plot_boundary_tracking.m           # 边界追踪+RMSE（修复图例）
│   ├── plot_plume_agents_3d.m             % 三维羽流+AUV轨迹
│   ├── plot_voronoi_3d.m                  % 三维Voronoi区域（自适应渲染）
│   ├── plot_monitoring_dashboard.m        % 2×4综合仪表板
│   ├── create_dynamic_plume_gif.m         % 动态GIF
│   └── smooth_trajectory.m                % 轨迹平滑
├── write_step8_analysis.m                 % 结果分析文档生成
└── tests/
    └── test_step8_dynamic_tracking.m      % 唯一测试入口
```
