# MuJoCo C++ 单摆示例

使用 MuJoCo 原生 API 加载 MJCF 单摆模型，通过 PD 力矩控制使其回到竖直向下的位置。默认使用 GLFW + MuJoCo OpenGL 渲染器打开实时三维窗口，持续运行到关闭窗口。

## 构建与运行

依赖由 `algorithm/third_party/mujoco/CMakeLists.txt` 统一管理，与项目其他第三方库一样通过命名空间 target 提供：

- `algorithms::mujoco`：MuJoCo 头文件及动态库，默认路径 `/opt/mujoco`，可通过 `MUJOCO_ROOT` 修改。
- `algorithms::mujoco_viewer`：MuJoCo、GLFW 和 OpenGL，用于图形显示。

需要系统安装 GLFW 和 OpenGL 开发库，并满足主工程原有依赖。所有依赖在配置时检查，缺少时会报错。示例的 CMake 仅定义可执行文件、链接依赖和模型路径，通过主工程构建，不再单独配置。

在仓库根目录执行：

```bash
cmake -S algorithm -B build -DMUJOCO_ROOT=/opt/mujoco
cmake --build build --target mujoco_pendulum -j
./build/experiments/mujoco_pendulum/mujoco_pendulum
```

模型会复制到构建目录，程序可以从其他工作目录启动。CMake 的构建 RPATH 用于定位 MuJoCo 动态库，通常无需设置 `LD_LIBRARY_PATH`。

## 图形交互

- 空格：暂停 / 继续仿真。
- R：重置至初始角度 0.8 rad，保留当前暂停状态。
- Esc 或关闭窗口：退出。
- 鼠标左键拖动：旋转视角；配合 Shift 切换旋转方向。
- 鼠标右键拖动：平移视角；配合 Shift 切换平移方向。
- 滚轮或中键拖动：缩放。

窗口显示仿真时间、角度和运行状态。仿真根据墙钟时间推进；窗口卡顿时限制追赶步数，保持交互响应。单摆稳定后窗口仍保持打开，可以按 R 重新观察运动。

运行图形模式需要可用的桌面显示和 OpenGL 上下文。无桌面环境可以使用：

```bash
./build/experiments/mujoco_pendulum/mujoco_pendulum --headless
```

无窗口模式执行 2500 步，每 250 步打印一次状态；默认步长 0.002 秒，对应 5 秒仿真，不按真实时间等待。此选项仅切换运行模式，构建仍链接图形依赖。

## 代码入口

- `pendulum.xml`：长 1 米、质量 1 千克的杆，绕 Y 轴旋转，电机力矩限制为 ±10 N·m。
- `main.cc`：加载模型、创建状态并设置初始角度；控制律为 `torque = clamp(-8 * angle - 2 * angular_velocity, -10, 10)`；调用 `mj_step` 推进仿真。通过 RAII 自动释放模型和状态。
- `viewer.cc`：窗口和输入事件、相机控制、实时仿真循环，以及 `mjv_updateScene` / `mjr_render` 渲染。渲染资源在 OpenGL 上下文销毁前释放。

可传入修改后的单摆 XML，例如：

```bash
./build/experiments/mujoco_pendulum/mujoco_pendulum algorithm/experiments/mujoco_pendulum/pendulum.xml
```

控制器针对本例单自由度、直接力矩驱动模型，不支持直接替换为任意机器人模型。修改模型步长会改变无窗口模式的总仿真时长。
