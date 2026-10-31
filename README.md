# Kinodynamic A* + B-spline

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![C++](https://img.shields.io/badge/Language-C++17-orange.svg)](CMakeLists.txt)

面向地面移动机器人的 ROS 2 二维局部轨迹规划工程，采用 **Kinodynamic A* 前端搜索 + Fast Planner 风格的 B 样条后端优化**。前端负责在占据地图中找到满足运动约束的避障轨迹，后端以该轨迹为初值进行平滑、距离场避障和动力学可行性调整。

ROS 2 节点提供 RViz 交互输入与规划结果可视化。ROS 包名为 `kinodynamic_astar`，可执行文件为 `motion_plan`。

## 核心功能

- **二维地图**：原始占据层与圆形膨胀层，局部地图随当前车体位置重建。
- **单圆碰撞模型**：以车辆参考点为中心进行圆形膨胀与碰撞检查。
- **Kinodynamic A\* 前端**：搜索状态包含平面位置与速度，通过加速度控制采样拓展状态，并在二维膨胀占据地图中检查碰撞，生成可行搜索轨迹。
- **Fast Planner 风格后端**：把 A* 轨迹参数化为三次非均匀 B 样条，使用 NLopt 梯度优化控制点；结合曲率、ESDF 障碍距离、速度/加速度可行性和端点代价，并调整时间间隔。
- **结果校验与回退**：优化轨迹按固定步长采样，并在膨胀占据地图中复核碰撞。优化满足动力学可行性且无碰撞时发布优化结果；否则回退发布 A* 搜索轨迹。
- **RViz 可视化**：显示参考路径、搜索/优化轨迹、原始障碍物及膨胀障碍物。

当前仓库提供规划与可视化节点、Launch 一键启动文件及预置 `rviz.rviz` 配置，不包含小车动力学仿真或轨迹跟踪控制器。

## 编译与运行

### 环境与依赖

已在 ROS 2 Humble 环境完成编译验证。其他 ROS 2 发行版尚未验证。

需要 CMake 3.22 或更新版本、支持 C++17 的编译器、Eigen3、NLopt，以及以下 ROS 2 依赖：

`ament_cmake`、`rclcpp`、`nav_msgs`、`geometry_msgs`、`sensor_msgs`、`visualization_msgs`、`tf2`、`tf2_ros`。

一键启动还需要 `launch`、`launch_ros` 和 `rviz2`。

### 编译

本项目当前在 `ros2_dev` 容器中开发。进入容器后，在项目根目录执行：

```bash
docker exec -it ros2_dev bash
```

然后在容器终端中编译：

```bash
source /opt/ros/humble/setup.bash
cd /workspace/develop_front_end/kinodynamic_astar_bspline
colcon build --packages-select kinodynamic_astar
source install/setup.bash
```

如果容器中尚未安装 NLopt，先执行 `apt update && apt install -y libnlopt-cxx-dev`。目录改名后若看到 CMake 报旧目录不存在，删除项目 `build/kinodynamic_astar` 后重新编译：

```bash
rm -rf build/kinodynamic_astar
colcon build --packages-select kinodynamic_astar
```

### 一键启动（推荐）

在上述容器终端中（已 source `install/setup.bash`）执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch kinodynamic_astar demo.launch.py
```

也可以从宿主机直接启动（要求 `ros2_dev` 容器正在运行且已配置 RViz 图形转发）：

```bash
docker exec -it ros2_dev bash -lc 'source /opt/ros/humble/setup.bash && cd /workspace/develop_front_end/kinodynamic_astar_bspline && source install/setup.bash && ros2 launch kinodynamic_astar demo.launch.py'
```

该命令同时启动 `motion_plan` 规划节点和 RViz，并自动加载随包安装的 `rviz.rviz`。配置已设置 **Fixed Frame = map** 和规划显示项，无需手动添加；按 `Ctrl+C` 结束本次启动的两个进程。Docker 需要已配置图形显示转发，才能打开 RViz。

新增或修改 Launch 文件、`rviz.rviz` 后，请重新执行上述编译命令并加载环境。加载安装环境后，启动命令不依赖当前目录。运行 RViz 需要可用的图形显示环境；在 Docker 内启动时需配置显示转发。

也可指定自己的 RViz 配置：

```bash
ros2 launch kinodynamic_astar demo.launch.py rviz_config:=/absolute/path/to/custom.rviz
```

### 分别启动（可选）

调试时可在两个终端分别启动；每个终端都先加载 ROS 2 和工作空间环境：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

终端 1：

```bash
ros2 run kinodynamic_astar motion_plan
```

终端 2：

```bash
rviz2 -d "$(ros2 pkg prefix --share kinodynamic_astar)/rviz.rviz"
```

预置配置中的主要显示项如下：

| RViz 显示类型 | 话题 | 内容 |
| --- | --- | --- |
| Grid | — | 平面网格 |
| Path | `/visual_global_path` | 全局参考路径 |
| MarkerArray | `/trajectories` | Kinodynamic A* 搜索路径 |
| Path | `/visual_local_trajectory` | Kinodynamic A* 采样轨迹 |
| PointCloud2 | `/visual_obstacles` | 原始障碍物 |
| PointCloud2 | `/visual_local_obstacles` | 局部地图膨胀结果 |

### RViz 交互流程

1. 使用 **Publish Point**（`/clicked_point`）沿期望路线依次点击参考路径点。至少添加两个点；最后几个点也用于确定终点附近的路径方向。
2. 使用 **2D Goal Pose**（`/goal_pose`）在障碍物中心点击，以添加障碍物。该工具在当前实现中不是设置导航终点。
3. 使用 **2D Pose Estimate**（`/initialpose`）设置机器人起点和航向。已有参考路径时会触发规划。
4. 观察 `/visual_local_trajectory`。终端会输出 `planner_->makePlan() 耗时: ... ms`。

参考路径点和障碍物都会追加到当前场景；当前没有清空接口，重新开始场景请重启 `motion_plan` 节点。

## 碰撞模型

当前使用以车辆参考点为中心的**单圆碰撞模型**。圆半径通过 `vehicle.circle_radius` 配置，障碍物地图按该半径膨胀；地图外区域视为不可通行。当前版本不使用双圆车体模型。

## 规划架构：前端搜索 + 后端优化

规划入口位于 `PlannerInterface::makePlan()`。前端和后端串联工作：前端优先找到一条可行的避障初始轨迹，后端再改善其平滑性和动态可行性。

```mermaid
flowchart LR
    A[起点、参考路径、障碍物] --> B[Kinodynamic A* 前端]
    B -->|搜索成功：离散状态轨迹| C[Fast Planner 风格后端]
    C --> D[三次非均匀 B 样条参数化]
    D --> E[梯度带优化：曲率、ESDF 距离、动力学代价]
    E --> F[时间调整与动力学可行性检查]
    F --> G{优化轨迹无碰撞且可行？}
    G -->|是| H[发布优化后的 B 样条轨迹]
    G -->|否| I[回退到 A* 搜索轨迹]
```

### 前端：Kinodynamic A* 搜索

- 搜索状态包含二维位置和速度，控制量通过加速度采样生成；运动模型和最大速度、最大加速度用于状态拓展及可行性检查。
- 前端在二维占据栅格及其单圆膨胀层中检查碰撞，使用解析连接启发式加速搜索。它生成的是后端的初始轨迹；若后端优化不能通过检查，前端轨迹也作为安全回退结果。
- 航向不是搜索状态中的独立维度。输出轨迹的中间点方向由相邻轨迹点切向计算，末端方向沿用参考路径终点航向。

### 后端：参考 Fast Planner 的梯度带和 B 样条优化

本项目从工作区的 `Fast-Planner-2D-ROS2` 移植了后端所需模块，包括距离场地图、梯度弹性带优化器、三次非均匀 B 样条和时间调整逻辑；这是接在 Kinodynamic A* 之后的优化阶段，并非再运行一遍 Fast Planner 搜索器。

1. 将 A* 采样轨迹及起终点导数参数化为三次非均匀 B 样条，优化时固定端点相关控制点。
2. 优化器使用 NLopt L-BFGS 调整内部控制点。代价包含曲率平滑、ESDF 障碍距离、速度/加速度惩罚和端点项；ESDF 的障碍膨胀对应当前单圆车体半径，并额外设置 0.15 m 的后端安全间距。
3. 按最大速度和最大加速度检查轨迹；若不满足则调整时间间隔并重试。后端最多优化尝试 5 次，并逐步提高障碍距离项权重。
4. 将候选 B 样条按 0.05 s 间隔采样，检查数值有效性与单圆膨胀栅格碰撞。只有动力学检查通过且整段采样无碰撞时才接受优化轨迹。
5. 任何优化失败、动力学不可行或碰撞复核失败的情况，均使用原始 A* 采样轨迹回退，避免发布未经验证的优化结果。

RViz 的 `/trajectories` 会显示 A* 初始轨迹；后端成功时也会显示优化轨迹。`/visual_local_trajectory` 发布最终采用的轨迹。`planner_->makePlan()` 耗时日志覆盖一次调用内的 A* 搜索、ESDF 更新及后端优化/检查，不包括 ROS 输入处理和可视化发布。

## ROS 2 参数

节点启动时读取单圆碰撞半径：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `vehicle.circle_radius` | 0.4 m | 车辆参考点周围的单圆碰撞半径 |

示例：

```bash
ros2 run kinodynamic_astar motion_plan --ros-args \
  -p vehicle.circle_radius:=0.5
```

地图尺寸、分辨率和 Kinodynamic A* 运动约束仍在源码中配置。

## 话题接口

节点名为 `kinodynamic_astar_interactive_node`。下表使用默认命名空间下的话题名。

| 方向 | 话题 | 消息类型 | 用途 |
| --- | --- | --- | --- |
| 订阅 | `/clicked_point` | `geometry_msgs/msg/PointStamped` | 追加参考路径点 |
| 订阅 | `/goal_pose` | `geometry_msgs/msg/PoseStamped` | 追加障碍物，使用位置 XY |
| 订阅 | `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 更新车体起点、航向并触发重规划 |
| 发布 | `/visual_global_path` | `nav_msgs/msg/Path` | 参考路径 |
| 发布 | `/trajectories` | `visualization_msgs/msg/MarkerArray` | Kinodynamic A* 搜索路径 |
| 发布 | `/visual_local_trajectory` | `nav_msgs/msg/Path` | 规划轨迹 |
| 发布 | `/visual_obstacles` | `sensor_msgs/msg/PointCloud2` | 原始障碍物 |
| 发布 | `/visual_local_obstacles` | `sensor_msgs/msg/PointCloud2` | 膨胀障碍物 |

`/rviz_input_global_path` 和 `/rviz_input_obstacles` 的订阅创建代码目前被注释，不能直接作为已启用的数据入口。

## 目录结构

```text
.
├── CMakeLists.txt
├── package.xml
├── launch/demo.launch.py                 # 一键启动规划节点与 RViz
├── rviz.rviz                             # RViz 预置配置
├── include/trajectory_obstacles_publisher.h
├── src/trajectory_publisher.cpp           # ROS2 交互、参数、发布及路径离散
└── planner/
    ├── GridMap2D/                         # 二维地图与单圆碰撞查询
    ├── path_searching/                    # Kinodynamic A* 搜索
    ├── fast_backend/                      # 移植的 Fast Planner 梯度带、B 样条与 ESDF 后端
    └── plan_manage/                       # 规划流程与 ROS 接口
```


## 参考与致谢

- [ZJU-FAST-Lab/ego-planner](https://github.com/ZJU-FAST-Lab/ego-planner)：原始 EGO-Planner 算法。
- Fast Planner 后端参考：工作区中的 `Fast-Planner-2D-ROS2`，移植了梯度弹性带、非均匀 B 样条及 ESDF 距离场模块。
- [JackJu-HIT/Ego-Planner-2D-ROS2](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2)：本项目的二维 ROS 2 基础工程，也是当前 Git remote 指向的仓库。
-  [JackJu-HIT/SCAN-Planner-Pure-ROS2](https://github.com/JackJu-HIT/SCAN-Planner-Pure-ROS2)：局部占据地图与沿航向前后偏移查询的参考实现。

本项目由 Two-Circle-EGO-Planner-2D-ROS2 改造而来，并集成 standalone_planner 中的 Kinodynamic A*。用于二次开发或论文时，请注明所使用的仓库地址与版本，并保留上游署名。

## 作者与交流

作者：**鞠柏贤**。

- 微信公众号：**机器人规划与控制研究所**。[相关技术文章](https://mp.weixin.qq.com/s/tjHMyyEMzsonYaVbrq4WTQ)
- Bilibili：**机器人算法研究所**。[原项目演示视频](https://www.bilibili.com/video/BV11RUfB8ELb/)

以上链接沿用基础项目资料，不代表本双圆版本的验证记录。

## 许可证

项目采用 [MIT License](LICENSE)，使用、修改与分发请遵循许可证并保留版权声明。

*文档更新：2026-09-20*
