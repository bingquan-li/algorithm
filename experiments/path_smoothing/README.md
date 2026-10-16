# 投影梯度路径平滑实验

`pg_smooth` 读取 CSV，求解路径平滑问题，再用 Rerun 显示原路径和结果。
默认参数保存在 `pg_smooth.cc`：参考权重 `20`、平滑权重 `100`、固定端点、
容差 `1e-8`、圆形位移半径 `1.0`、最大迭代次数 `1000`。默认开启求解日志和查看器。

## 编译和运行

在包含 `algorithm/`、`build/`、`data/` 的工作区目录执行，依赖沿用项目现有 CMake 配置：

```bash
cmake -S algorithm -B build -DBUILD_TESTING=ON
cmake --build build --target pg_smooth
./build/experiments/path_smoothing/pg_smooth
```

默认 CSV 是工作区的 `data/narrow_passage_reversing/path.csv`，其位置由 CMake 根据源码目录设置，
无需在代码中填写本机绝对路径。指定 CSV 时，前两列为 `x,y`，可包含 `x,y` 表头及后续额外列。

无图形界面时，可只运行求解器并保存日志：

```bash
./build/experiments/path_smoothing/pg_smooth --no-viewer > /tmp/pg_smooth.log 2>&1
./build/experiments/path_smoothing/pg_smooth --no-viewer --path /tmp/repro.csv
./build/experiments/path_smoothing/pg_smooth --no-viewer --quiet
./build/experiments/path_smoothing/pg_smooth --help
```

`verbose` 日志写入 `std::clog`（标准错误），`Result` 写入标准输出，所以保存完整日志需要 `2>&1`。
`--quiet` 关闭求解日志，仍输出 `Result`。`--no-viewer` 跳过查看器启动，求解过程与默认运行相同。
求解及结果输出在启动查看器之前完成，查看器故障不会遮住已得到的求解信息。

退出码：`0` 表示收敛，`2` 表示达到迭代上限，`1` 表示参数、输入、数值计算或查看器错误。
错误会输出具体原因；非法 CSV 坐标包含行号，求解期间的数值错误包含迭代号，并在可定位时包含点索引。

## 如何读日志

日志包含问题规模、参数、迭代表格和结束摘要。默认保留第 `0` 轮初始状态、第 `1` 轮和最终轮，
中间每 `max(1, ceil(max_iterator / 20))` 轮输出一次，设置区显示实际 `log_interval`。

| 字段 | 含义 |
|---|---|
| `objective` | 当前路径的总代价，即 `reference + smoothness` |
| `reference` | 加权参考代价：`w_ref * sum(||p[i] - origin[i]||²)` |
| `smoothness` | 加权平滑代价：`w_smooth * sum(||p[i-1] - 2*p[i] + p[i+1]||²)` |
| `max_step` | 本轮更新中最大的单点移动量；小于 `tolerance` 时停止 |
| `pg_norm` | `max_step / step_size`，本轮更新起点的投影梯度映射最大单点范数 |
| `clipped` | 本轮投影前越出允许区域的自由点数，不包含固定端点 |
| `violation` | 当前路径相对原路径允许区域的最大超限量，下限为零；圆形用 L2 距离，方形用 L∞ 距离 |
| `time` | 累计墙钟时间，包含此前日志输出的开销 |

`pg_norm` 来自最近一次更新，不是在当前表格所显示的更新后路径上重新计算梯度。
有约束时，它也不等于原始梯度范数。初始第 `0` 轮尚未更新，`max_step`、`pg_norm` 和 `clipped` 显示 `N/A`。

排查时同时观察 `objective`、`max_step`、`pg_norm`、`clipped` 和 `violation`：
小的 `max_step` 可能来自小步长；`clipped` 较多说明很多点的更新受到区域限制；
`violation` 应接近零。`converged=true` 表示满足当前停止条件，仍需用解析解或回归测试验证实现。

参数要求：权重有限且非负，容差有限且严格大于零，位移半径有限且非负。
两个权重都为零时目标恒为零，直接返回原路径，报告收敛、零次迭代。
遇到非有限或不可表示的数值更新时抛出异常，不能将其当作正常收敛并报告 `solved`。

## 自动验证

自动测试放在 `algorithm/tests/path_smoothing/`，使用代码中的小路径，不需要查看器或真实 CSV。
实验程序用于检查真实路径、调参和观察日志；测试用于重复确认算法和诊断输出行为。

```bash
cmake --build build --target projected_gradient_test
ctest --test-dir build --output-on-failure -R path_smoothing
```
