# pb_nav_goal_decision — 自动目标决策节点

> 哨兵导航栈中的"决策层"：把 **裁判系统的状态** 翻译成 **Nav2 的导航目标**，让机器人按规则自动跑点，
> 取代人工/脚本发点（`to_center.sh` / `to_home.sh` 那类做法）。

---

## 一、包的作用

**一句话**：订阅几个简单的裁判状态信号，按有限状态机（FSM）决定"下一个该去哪"，通过
`navigate_to_pose` action 下发给 Nav2，并依据执行结果继续推进状态。

**它做什么**
- 读取配置文件中预定义的**明确坐标点**（不随机采样）。
- 监听裁判输入信号，判断"游戏是否开始""血量是否过低"。
- 据此选择目标点，调用 Nav2 的 `navigate_to_pose` action 下发。
- 接收 action 结果（成功/失败/取消），更新状态，形成闭环。

**它不做什么**
- ❌ 不做路径规划、不做速度控制（那是 Nav2 planner/controller 的事）。
- ❌ 不读串口、不解析裁判协议（那是"转发节点"的事）。
- ❌ 不做目标点的实时动态生成（点位来自配置文件）。

**在整条链路中的位置**
```
裁判系统 → 控制硬件(MCU) → 串口 → 转发节点 → ROS(标准消息) → 【本节点】 → navigate_to_pose action → Nav2 → 底盘
                                          ↑ 输入                       ↑ 输出
```

### 1.1 接口一览

| 方向 | 名称（相对名） | 类型 | 说明 |
|---|---|---|---|
| 输入 | `referee/game_progress` | `std_msgs/msg/UInt8` | 赛制进度，`>=` 阈值视为"比赛开始" |
| 输入 | `referee/current_hp` | `std_msgs/msg/Int32` | 当前血量 |
| 输入 | `referee/maximum_hp` | `std_msgs/msg/Int32` | 满血值（用于算百分比阈值） |
| 输出 | `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 作为 **action client** 向 Nav2 下发目标 |

> **接口原则**：输入**只用标准 ROS 消息**（`std_msgs`），不引入 `pb_rm_interfaces` 等自定义 msg 包。
> 转发节点负责"拆包"成标准消息话题，本节点只订阅标准消息。
> **这些话题名是相对名**——节点必须与 Nav2 处于同一命名空间（本项目为 `red_standard_robot1`），
> 否则会解析到 `/navigate_to_pose` 之类不存在的地方。

### 1.2 节点形态

- 包名：`pb_nav_goal_decision`　节点名：`goal_decision_node`
- C++ / `rclcpp_components` 组件：既注册为组件（可进 composition 容器），也产出独立可执行文件。
- 真机启动中作为**独立进程**运行（不进 `nav2_container`），以避免决策节点崩溃牵连导航栈。
- **不使用 tf2 监听器**：仅用 `tf2::Quaternion` 做 yaw→四元数的数学转换，因此**不需要** `/tf` 重映射。

---

## 二、有限状态机（FSM）设计规范

### 2.1 两个"条件量"（FSM 的全部输入）

状态机不直接读话题，而是先由回调把输入归约成两个布尔量：

| 条件量 | 计算式 | 更新时机 |
|---|---|---|
| `game_started_` | `game_progress >= game_start_progress_threshold_` | 收到 `referee/game_progress` 且结果变化时 |
| `hp_low_` | `maximum_hp > 0 && current_hp <= maximum_hp * hp_return_threshold_ / 100` | 收到 `current_hp` 或 `maximum_hp` 任一，重新计算 |

> `maximum_hp == 0` 时强制 `hp_low_ = false`，避免"还没收到满血值"被误判成低血量。

### 2.2 状态定义

| 状态 | 含义 |
|---|---|
| `WAIT_GAME_START` | 初始态，等待比赛开始 |
| `GOING_CENTER` | 已下发"去中心"，正在前往 |
| `AT_CENTER` | 已抵达中心（上一个目标结果为 SUCCEEDED） |
| `GOING_SPAWN` | 已下发"回出生点"，正在前往 |
| `AT_SPAWN` | 已抵达出生点 |

> 之所以区分 `GOING_x` / `AT_x`，是为了让"在途"与"到位"两种情形都能被同一条转移规则覆盖
> （例如：无论在途还是已到位，只要血量低了都要回出生点）。

### 2.3 转移表（第一版）

| # | 当前状态 | 条件 | 动作 | 次态 |
|---|---|---|---|---|
| T1 | `WAIT_GAME_START` | `game_started_` | `sendGoal("center")` | `GOING_CENTER` |
| T2 | `GOING_CENTER` / `AT_CENTER` | `hp_low_` | `cancelCurrentGoal()` → `sendGoal("spawn")` | `GOING_SPAWN` |
| T3 | `GOING_SPAWN` / `AT_SPAWN` | `game_started_ && !hp_low_` | `sendGoal("center")` | `GOING_CENTER` |

```
        ┌──────────────────┐
        │ WAIT_GAME_START  │
        └────────┬─────────┘
                 │ T1: game_started
                 ▼
        ┌──────────────────┐   T2: hp_low    ┌──────────────────┐
        │  GOING_CENTER    │ ──────────────▶ │  GOING_SPAWN     │
        │  AT_CENTER       │ ◀────────────── │  AT_SPAWN        │
        └──────────────────┘  T3: started   └──────────────────┘
                                 && !hp_low
```
即 **"开始→去中心；血量低→回出生点；血量恢复且比赛未结束→再回中心"** 的闭环。

### 2.4 到达判定

目标执行结束（`resultCallback`）时按**点位名**回填状态：
- `center` 结束 → `AT_CENTER`
- `spawn` 结束 → `AT_SPAWN`

随后立即调用 `onConditionsChanged()` 复判，使闭环能自动继续。

### 2.5 设计规范（写代码/扩展时必须遵守）

1. **条件驱动，不轮询**：状态迁移只能由"条件量变化"或"action 结果"触发，不允许无条件周期性重发目标。
   实现在 `onConditionsChanged()`，并在 `gameProgressCallback` / `recomputeHpLow` / `resultCallback` /
   `serverCheckTimerCallback` 四处被调用。
2. **链式传导**：`onConditionsChanged()` 内部用有界循环（最多 4 次）连续应用转移，
   使"条件同时满足多条规则"也能一次性收敛；`server_ready_ == false` 时直接返回，不发任何目标。
3. **防重发**：`sendGoal()` 内若"同一目标正在进行中"（`goal_active_ && current_goal_name_ == point_name`）则直接返回。
4. **抢占先取消**：切换到不同目标点前必须先 `cancelCurrentGoal()`（当前仅 T2 需要），避免两个目标并存。
5. **忽略陈旧结果**：`resultCallback` 中若 `point_name != current_goal_name_`（已被抢占/取消的旧目标）直接丢弃。
6. **点位必须配置化**：代码中**不硬编码任何坐标**，一律从 YAML 按名字取；`center` 与 `spawn` 是**必需点位**，
   缺失时启动日志会告警。
7. **阈值必须配置化**：`game_start_progress_threshold`、`hp_return_threshold` 均写在配置里，改赛制不改代码。
8. **命名空间一致**：节点必须与 Nav2 同命名空间启动，否则相对名解析失败（见 4.2）。
9. **只用标准消息**：新增输入时优先用 `std_msgs`，避免给导航侧引入自定义 `.msg` 包依赖。
10. **单线程假设**：回调访问共享成员（`state_`、`goal_active_` 等）**未加锁**，依赖默认单线程 executor。
    若改为 `MultiThreadedExecutor`，需自行加互斥保护。
11. **扩展方式**：新增决策 = ① 在配置里加点位 → ② 加/改一个条件量 → ③ 在转移表加一条规则。
    尽量不引入新状态，优先"复用 `AT_x`/`GOING_x` 的成对写法"。

### 2.6 待完善（已知限制）

- **无重试机制**：目标 `FAILED/ABORTED` 后不会自动重发，仅更新状态并复判。
- **无超时监控**：长时间卡住不会自动取消。
- **无仲裁**：与手柄 `pb_teleop_twist_joy` 的 `auto_control`、RViz 手动目标共用同一 action，互相之间没有优先级约定。
- **定位就绪未校验**：仅在 action server ready 后才允许发目标；若定位未初始化，目标可能被 Nav2 静默丢弃。
- `sendGoal()` 存在"早退但调用方仍推进状态"的分支（server 未就绪 / 目标重复）；
  由于 `onConditionsChanged()` 已先行拦截 server 未就绪，实际影响很小，但扩展时需留意。

---

## 三、配置文件的作用

**路径**：`config/goal_decision_params.yaml`
（安装后位于 `share/pb_nav_goal_decision/config/goal_decision_params.yaml`）

**为什么要有它**：**地图会变，坐标随之变**。把点位与阈值集中在此文件，
**改地图/改战术只改 YAML，不动代码、不用重新编译**（重启节点即生效）。

**加载方式**
- 节点参数 `config_file`（`std::string`，默认 `""`）：
  - 留空 → 自动用 `ament_index` 定位到本包 share 目录下的默认配置；
  - 也可显式传入绝对路径，指向自定义配置。
- 文件由 **yaml-cpp 直接读取**（不是 ROS 参数系统），所以**不是** `ros2 param` 能改的；
  改完需重启节点（或重新 `ros2 run/launch`）。

### 3.1 字段说明

```yaml
goal_points:                     # 预定义坐标点列表（决策时按 name 引用）
  - name: home                   #   点位名（任意，但 center/spawn 为必需）
    frame_id: map                #   坐标系，缺省 "map"
    x: 0.0                       #   X（米），缺省 0.0
    y: 0.0                       #   Y（米），缺省 0.0
    yaw: 0.0                     #   朝向（弧度），缺省 0.0；代码内部转四元数

decisions:                       # 决策阈值
  game_start_progress_threshold: 4   # game_progress >= 此值 => 视为比赛开始（缺省 4）
  hp_return_threshold: 30            # current_hp <= maximum_hp * 30% => 返回出生点（缺省 30，百分比）
```

| 字段 | 作用 | 缺省 |
|---|---|---|
| `goal_points[].name` | 点位标识，`sendGoal("center")` 即按此匹配 | **必填** |
| `goal_points[].frame_id` | 目标点坐标系（应为 `map`） | `map` |
| `goal_points[].x` / `.y` | 平面坐标（米） | `0.0` |
| `goal_points[].yaw` | 朝向（弧度），`0` 表示朝 +X | `0.0` |
| `decisions.game_start_progress_threshold` | "比赛开始"判定阈值 | `4` |
| `decisions.hp_return_threshold` | 返航血量阈值（满血百分比） | `30` |

### 3.2 必需点位与校验

- **`center`**：T1/T3 的目标（"去中心"）。
- **`spawn`**：T2 的目标（"回出生点"）。
- 二者缺一时，启动日志会打 `WARN: Config missing required points 'center' and/or 'spawn'!`；
  此时对应的 `sendGoal` 会因找不到点位而报 `ERROR: Unknown goal point 'xxx'!` 并发出一个空目标（需避免）。
- 配置加载成功后，日志会逐条打印已加载点位与两个阈值，便于核对。

### 3.3 当前默认配置（示例）

| 点位 | x | y | yaw |
|---|---|---|---|
| `home` | 0.0 | 0.0 | 0.0 |
| `spawn` | 0.5 | 0.0 | 0.0 |
| `center` | 3.5 | 1.2 | 1.57 |

> ⚠️ 这些是**占位数值**，随赛场地图变化需自行填写。

---

## 四、编译与启动

### 4.1 编译

```bash
cd ros_ws
colcon build --paths src/pb2025_sentry_nav/pb_nav_goal_decision \
             --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

> 本仓库实测：`colcon build` 会被工作区内与新版发行版 API 不兼容的**无关旧包**（如 `fake_vel_transform`
> 引用 `message_filters/subscriber.h`）阻断，故用 `--paths <包目录>` 只编本包。

### 4.2 独立启动（调试用）

```bash
ros2 launch pb_nav_goal_decision goal_decision_launch.py
```
该 launch 中 `namespace` 一行默认被注释——**若 nav2 在 `red_standard_robot1` 下运行，必须取消注释**，
否则 `navigate_to_pose` 与 `referee/*` 都会解析到根命名空间而找不到。

### 4.3 已接入真机启动链路

`pb2025_nav_bringup/launch/rm_navigation_reality_launch.py` 已加入本节点：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py                  # 默认启动决策
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py use_goal_decision:=False   # 临时关闭
```

- 节点以 `namespace=<namespace>`（默认 `red_standard_robot1`）启动，与 Nav2 同命名空间。
- 未加 `/tf` 重映射（本节点不用 tf2 监听器）。
- 仿真入口 `rm_navigation_simulation_launch.py` **暂未接入**（如需，按同样三处添加即可）。

### 4.4 运行前置条件

1. **Nav2 已就绪**：`navigate_to_pose` action server 必须存在（`autostart=true` 时会自动拉起）。
   节点用 1 秒定时器轮询 `action_server_is_ready()`，就绪前不会发任何目标。
2. **转发节点已发布输入**：`referee/game_progress` / `referee/current_hp` / `referee/maximum_hp`
   必须在**同一命名空间**下被发布。⚠️ **当前仓库中尚无这些话题的发布者**，
   转发节点做出来之前，本节点会一直停在 `WAIT_GAME_START`（无害，但不会有动作）。
3. **定位已初始化**：否则 Nav2 可能静默丢弃目标（见 2.6）。

---

## 五、目录结构

```
pb_nav_goal_decision/
├── CMakeLists.txt
├── package.xml
├── README.md                       # 本文件
├── config/
│   └── goal_decision_params.yaml   # 点位 + 阈值（改地图只改这里）
├── include/pb_nav_goal_decision/
│   └── goal_decision_node.hpp      # 类定义、State 枚举、条件量成员
├── src/
│   └── goal_decision_node.cpp      # 配置加载 / FSM / action 客户端
└── launch/
    └── goal_decision_launch.py     # 独立启动（调试用）
```

---

## 六、相关文档

- `../../../../通信规范.md`：本包的输入/输出接口规范（含"为何用 action 而非话题/service"）。
- `../../../../TODU.md`：设计讨论记录（需求来源、待定问题、实现过程）。
