# 左右边界查询与局部脱困规划

`later_boundary` 读取 `frame.json` 中的原始栅格和世界坐标 footprint，查询左右边界，
生成差速底盘的短距离脱困轨迹，并用 Rerun 显示结果。

**查看 Rerun 请运行 `build/experiments/path_orientation/later_boundary`。**
`build/tests/planning/local_escape_planner_test` 是自动化测试程序，不启动 Rerun；
运行需要数秒，全部通过后输出 `Local escape planner tests passed`。
若设置了 `LATTICE_RERUN_SAVE`，可视化程序只保存录制文件，不会启动窗口。

```bash
cmake --build build --target later_boundary local_escape_planner_test -j 2
./build/experiments/path_orientation/later_boundary

# 不启动图形界面，保存后再查看
LATTICE_RERUN_SAVE=build/experiments/path_orientation/local_escape.rrd   ./build/experiments/path_orientation/later_boundary
rerun build/experiments/path_orientation/local_escape.rrd

ctest --test-dir build -R '^planning\.' --output-on-failure
```

可用第一个命令行参数指定其他 frame.json。机器人位置和方向仍在
`later_boundary_main.cc` 中配置，必须与该帧 footprint 对应。
`vehicle_pose` 必须是差速底盘的旋转中心；若传感器或轮廓中心与它不同，应先做坐标转换。

## 调用接口

```cpp
local_escape::Config config;
config.reverse_length = 0.6;       // 后退圆弧的长度，m
config.max_curvature = 1.2;        // 最大 |d(yaw)/ds|，1/m
config.safety_margin = 0.02;       // 完整车身的安全余量，m
config.allow_in_place_rotation = true;

// body_footprint：车体坐标，x 向前、y 向左，原点为旋转中心。
// 支持简单凹多边形，首尾顶点可重复；禁止自相交和零长度边。
local_escape::Pose start{position, yaw};
auto result = local_escape::plan(grid_map, start, body_footprint, config);
if(result.selected) {
  auto const& trajectory = result.candidates[*result.selected];
  // trajectory.poses：几何位姿序列。
  // trajectory.motions：前进/后退圆弧和原地旋转的分段参数。
}
```

`Motion.distance` 为有符号弧长，负数表示后退。曲率定义为 `d(yaw)/ds`：
正曲率后退时 yaw 减小，通常表现为车尾向左侧移、车头向右外摆。
原地旋转使用 `distance=0, curvature=0, rotation!=0`，rotation 逆时针为正。

## 候选生成与选择

1. 根据实际 footprint 的各纵向截面计算左右余量。查询覆盖当前车身和目标后退区域，
   在不移动地图的情况下估计前后收窄趋势。
2. 用小角度车身侧移模型拟合优先曲率。边界剖面中的余量按直线预测后的车身截面计算；
   它是候选生成的近似，不能代替旋转后的轮廓碰撞检查。
3. 保留优先曲率、其半值、直线和左右多档曲率，分别检查完整后退轨迹。
4. 直接后退均不可行时，再搜索原地旋转后退、前进后退，以及有限宽度的前进—旋转—后退。
   默认尝试前进 0.15/0.3 m，原地旋转 ±0.15/±0.3 rad；每次重新查询边界。
5. 成功轨迹必须有至少 `min_reverse_progress` 的净后退量。评分综合净后退量、
   整段最小余量、前进距离、转向量和偏离；完整直接后退优先于调整动作。

障碍判定默认 `cell >= 1`，通过 `occupied_threshold` 修改。地图外也视为障碍。
多边形与完整障碍格做距离计算，既检查轮廓边缘，也检查轮廓内部障碍。

圆弧和旋转都按 `|ds| + footprint_radius * |dtheta|` 约束每次检查的最大角点位移，
步长不超过半个栅格。中点车身余量减去半步最大位移，作为整个区间余量的保守下界。
因此不是只检查离散端点；薄障碍和旋转角点扫掠也会被检查。
`min_clearance` 为这个保守下界，截断于 `clearance_cap`（默认 0.3 m）。

状态：

- `success`：`selected` 指向通过检查且有净后退进展的完整候选。
- `start_blocked`：当前 footprint 已碰撞或不足指定安全余量，不输出轨迹。
- `no_path`：当前有限候选集合中未找到方案，不输出碰撞前的部分轨迹。

这是静态地图上的有界局部几何搜索，并非全局完备规划或实时控制器。
`no_path` 不表示环境中不存在其他绕行路径。执行前需要按底盘速度、加速度和制动能力
做时间参数化；分段换向/原地旋转处可停车切换，并在执行过程中重新检查地图与轨迹。

## Rerun

- `world/grid_map`：原始栅格，按原始类别分组。
- `world/robot`：红色 footprint、位置和朝向。
- `world/centerline`、`world/boundary`：黄色中心线，绿色左边界，蓝色右边界及测距连线。
- `world/planner/candidates`：候选轨迹；灰色为失败/调整前缀，淡紫色为可行候选。
  红色表示首个未通过碰撞或安全余量检查的位姿及 footprint，可整体隐藏该分组减少遮挡。
- `world/planner/selected`：紫色最终轨迹和沿途 footprint，绿色动画轮廓。
- `planner_summary`：选择状态、边界趋势、分段参数、净后退量及余量。
- `planner_details`：各候选被接受/排除的原因和评分。

`trajectory_sample` 时间轴仅为几何采样序号，不代表机器人执行时间。
