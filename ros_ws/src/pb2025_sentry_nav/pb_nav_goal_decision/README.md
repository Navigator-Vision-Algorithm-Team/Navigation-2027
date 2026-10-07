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
| 观测 | `navigate_to_pose/_action/status` | `action_msgs/msg/GoalStatusArray` | action 自身的状态话题：用它识别**别的客户端**（RViz / 手柄）发的目标，做优先级仲裁 |
| 观测 | `tf` / `tf_static` | `tf2_msgs/msg/TFMessage` | 只扫 `map -> odom` 是否出现，作为"定位已就绪"的判据 |

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
| T4 | `GOING_CENTER` / `GOING_SPAWN` | **无在飞目标**（`!goal_active_`，即被人工抢占/结果丢失） | 补发该状态对应的点位 | 不变 |

> T1–T3 是**业务转移**，T4 是**自愈规则**：它保证"只要状态说在去某点，就一定有一个目标在飞"。
> T4 受 `retry_pending_` 与 `goal_given_up_` 约束，因此不会变成无限重发。

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

### 2.4bis 发送门控（gate）

任何目标都只能在 `canSendGoals()` 为真时才能下发，否则 `onConditionsChanged()` 直接返回：

| 门控条件 | 不满足时的行为 | 由谁解除 |
|---|---|---|
| `server_ready_`：`navigate_to_pose` action server 已就绪 | 不发目标 | 看门狗（2 Hz 轮询 `action_server_is_ready()`） |
| `localization_ready_`：tf 上已出现 `map -> odom` | 不发目标（否则 Nav2 会拒绝/中止） | `tfCallback` 解析到该变换即置位 |
| `!external_goal_active_`：没有别的客户端在占用该 action | 不发目标（让位给人工） | 看门狗在 action "安静"够久后清除 |

> 门控是**正交于状态机**的：它不改变 `state_`，只决定"现在能不能发"。三者任一满足后都会回调
> `onConditionsChanged()` 让 FSM 立刻继续（无需等待下一次条件变化）。

### 2.5 设计规范（写代码/扩展时必须遵守）

1. **条件驱动，不轮询**：状态迁移只能由"条件量变化""action 结果""门控解除"触发，不允许无条件周期性重发目标。
   实现在 `onConditionsChanged()`，由 `gameProgressCallback` / `recomputeHpLow` / `resultCallback` /
   `tfCallback` / `watchdogTimerCallback` 调用。
2. **链式传导**：`onConditionsChanged()` 内部用有界循环（最多 4 次）连续应用转移，
   使"条件同时满足多条规则"也能一次性收敛。
3. **防重发**：`sendGoal()` 内若"同一目标正在进行中"（`goal_active_ && current_goal_name_ == point_name`）则直接返回。
4. **抢占先取消**：切换到不同目标点前必须先 `cancelCurrentGoal()`（当前仅 T2 需要），避免两个目标并存。
5. **忽略陈旧结果**：`resultCallback` 中若 `point_name != current_goal_name_`（已被抢占/取消的旧目标）直接丢弃。
6. **人工优先（仲裁）**：只要 action 上有**不属于本节点**的目标处于 ACCEPTED/EXECUTING，本节点就**让位不动**；
   待 action 安静 `arbitration.resume_delay_sec` 后再自动恢复。
   - 判定依据是该 action 的状态话题；用目标 UUID 区分"自己的"和"别人的"（`own_goal_uuid_`）。
   - **清除外部占位必须带迟滞**：手柄 `auto_control` 每 ~0.25 s 连发目标，其间存在瞬时空隙，
     故"空闲多久才算真空闲"由 `resume_delay_sec` 控制，不能一看到没有活动目标就抢。
   - 若状态话题长时间无更新（`status_stale_sec`），视为对方已消失，同样恢复。
7. **目标失败要重试**：`ABORTED`/`UNKNOWN`、以及被自身超时取消的目标，按 `safety.max_retries` 与
   `retry_delay_sec` 重试；预算耗尽后置 `goal_given_up_`，**彻底停止**对该点的重试直到下一次状态转移。
8. **超时必须能自愈**：超时先取消；若取消后 `kTimeoutCancelGraceSec` 内收不到结果（服务端静默），
   则本地判定失败并照常重试，绝不无限等待。
9. **"GOING_x" 必须始终有在飞目标（T4 恢复）**：若处于 `GOING_CENTER`/`GOING_SPAWN` 却没有活动目标
   （被人工抢占、结果丢失等），自动补发该点；由 `goal_given_up_` 与 `retry_pending_` 双重闸门防止死循环。
10. **点位必须配置化**：代码中**不硬编码任何坐标**，一律从 YAML 按名字取；`center` 与 `spawn` 是**必需点位**，
    缺失时启动日志会告警。
11. **阈值必须配置化**：`game_start_progress_threshold`、`hp_return_threshold` 及 safety/arbitration
    各项均写在配置里，改赛制不改代码。
12. **命名空间一致**：节点必须与 Nav2 同命名空间启动——action、referee 话题、`tf` 全是相对名（见 4.2）。
13. **只用标准消息**：新增输入时优先用 `std_msgs`，避免给导航侧引入自定义 `.msg` 包依赖。
14. **单线程假设**：回调访问共享成员（`state_`、`goal_active_`、`retry_count_` 等）**未加锁**，
    依赖默认单线程 executor。若改为 `MultiThreadedExecutor`，需自行加互斥保护。
15. **扩展方式**：新增决策 = ① 在配置里加点位 → ② 加/改一个条件量 → ③ 在转移表加一条规则。
    尽量不引入新状态，优先"复用 `AT_x`/`GOING_x` 的成对写法"。

### 2.6 异常处理与优先级仲裁

**目标生命周期里的四种结局分别怎么处理：**

| 结局 | 判定 | 处理 |
|---|---|---|
| 成功 | `SUCCEEDED` | 置 `AT_CENTER`/`AT_SPAWN`，清零重试计数与放弃标志，复判 |
| 超时 | 超过 `safety.goal_timeout_sec` | 取消该目标，并按**失败**计入重试 |
| 被外部取消 | `CANCELED` 且非本节点超时所为 | **不重试**，让位；等 T4 在人工目标结束后恢复 |
| 失败 | `ABORTED` / `UNKNOWN`（或目标被服务端拒绝） | 按 `safety.max_retries` 重试；若人工目标正在占用 action，则延后到恢复后再试 |

**优先级仲裁（`arbitration`）：**

```
优先级：  RViz 手动标点  >  本决策节点  >  手柄 pb_teleop_twist_joy
```

- 三者用的是**同一个** `navigate_to_pose` action。RViz 侧是 `nav2_rviz_plugins/GoalTool`，
  手柄侧是 `control_mode: auto_control`（真机 `nav2_params.yaml` 即此模式，每 0.25 s 连发目标）。
- 本节点通过 action 的状态话题**观测到任何不属于自己的活动目标就立刻让位**，
  等 action 安静 `resume_delay_sec`（默认 1 s，必须大于手柄的 0.25 s 连发间隔）后自动恢复决策。
- 也就是说：**手动操作期间决策节点完全不介入；手动目标结束/取消/失败后自动接回**。

> ⚠️ **关于"手柄优先级最低"**：从 action 接口**无法区分**一个外部目标来自 RViz 还是手柄——
> 两者都只是"别的客户端发的目标"。因此当前实现是"**任何人工来源都优先于决策**"，
> 这同时满足"RViz 优先、空闲时接入决策"，并且在安全上更稳妥（人工操作永远不被自动驾驶抢走）。
> 若确实需要"决策压过手柄"，只能从手柄侧解决（例如手柄 `auto_control` 时先停用本节点，
> 或给手柄加一个使能开关），无法在本节点内实现。

### 2.7 仍然存在的限制

- **无路径重规划引导**：导航失败只做原样重试，不做"换个中间点再试"等策略。
- **仲裁依赖 action 状态话题**：若把 action 改名（remap），`.../_action/status` 的名字必须同步，
  否则仲裁会失效（代码里两者由同一个 `kActionName` 常量派生，改一处即可）。
- **恢复只覆盖两个点位**：T4 只认 `center`/`spawn`；若将来新增点位与状态，需同步扩展 T4 的映射。
- **不做速度层仲裁**：本节点只管 action；若手柄处于 `manual_control` 模式（发 `cmd_vel` 而非目标），
  与本节点之间没有任何互斥，需要在系统层面自行保证不同时使用。

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

safety:                          # 单个目标的失败处理
  goal_timeout_sec: 60.0         # 单个目标超过此秒数 => 取消并按失败处理（必须 > 0）
  max_retries: 2                 # 同一个点最多重发几次（0 = 不重试）
  retry_delay_sec: 3.0           # 每次重试前的等待

arbitration:                     # 与人工目标（RViz / 手柄）的优先级仲裁
  enable: true                   # 关掉则不看别人的目标、直接发（不推荐）
  resume_delay_sec: 1.0          # action 需安静多久才恢复决策（须大于手柄 0.25s 连发间隔）
  status_stale_sec: 3.0          # 状态话题静默这么久 => 认为对方已消失，照样恢复

localization:                    # 定位就绪门控
  check_ready: true              # 未出现 map -> odom 前不发目标
  map_frame: map                 # 该变换的父坐标系
  odom_frame: odom               # 该变换的子坐标系
```

### 3.1bis 新增字段的作用

| 字段 | 作用 | 调大 / 调小的效果 |
|---|---|---|
| `safety.goal_timeout_sec` | 单目标最长执行时间 | 调小 → 卡住能更快发现，但正常长途导航可能被误杀 |
| `safety.max_retries` | 同一目标的重试预算 | 调大 → 更执着，但失败时会占用更久 |
| `safety.retry_delay_sec` | 重试间隔 | 调大 → 给 Nav2 更多恢复时间，但恢复更慢 |
| `arbitration.resume_delay_sec` | 人工目标结束后多久接回 | **必须 > 手柄连发间隔（0.25 s）**，否则会和手柄抢 action |
| `arbitration.status_stale_sec` | 状态话题静默多久算"对方消失" | 调大 → 更保守（宁可多等也不抢） |
| `arbitration.enable` | 是否启用仲裁 | 调试时可关掉，让决策节点无条件发目标 |
| `localization.check_ready` | 是否校验定位就绪 | 若 tf 不在同一命名空间导致一直不就绪，可临时关掉排查 |

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
该 launch 里 `namespace='red_standard_robot1'` **已默认开启且不建议去掉**：本节点的 action、
referee 输入与 `tf` 观测**全部使用相对名**，脱离命名空间后会去找 `/navigate_to_pose`、`/tf`、
`/referee/*`，结果是什么都收不到（节点会静默空转）。若你们的 nav2 换个命名空间，改这一行即可。

### 4.3 已接入真机启动链路

`pb2025_nav_bringup/launch/rm_navigation_reality_launch.py` 已加入本节点：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py                  # 默认启动决策
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py use_goal_decision:=False   # 临时关闭
```

- 节点以 `namespace=<namespace>`（默认 `red_standard_robot1`）启动，与 Nav2 同命名空间。
- **不需要任何 remap**：本节点对 action、referee 话题、`tf` 全部使用相对名，命名空间自动生效
  （注意 `tf` 用的是相对名 `tf`，而非 tf2_ros 那种绝对的 `/tf`，所以省掉了 `/tf -> tf` 重映射的坑）。
- 仿真入口 `rm_navigation_simulation_launch.py` **暂未接入**（如需，按同样三处添加即可）。
- `use_goal_decision` 参数可随时开关本节点；调试仲裁时可临时 `arbitration.enable: false`。

### 4.4 运行前置条件（三条门控，任一不满足就不发目标）

1. **Nav2 已就绪**：`navigate_to_pose` action server 必须存在（`autostart=true` 时会自动拉起）。
   节点用 2 Hz 看门狗轮询 `action_server_is_ready()`，就绪前不发任何目标。
2. **定位已就绪**：必须在 `<namespace>/tf` 上看到 `map -> odom` 变换
   （由 `small_gicp_relocalization` 在重定位成功后以 20 Hz 广播）。
   未就绪前本节点**主动不发目标**，避免被 Nav2 拒绝/中止而白白消耗重试预算。
   若这一步迟迟不就绪，先确认 tf 是否真的在同一命名空间下。
3. **转发节点已发布输入**：`referee/game_progress` / `referee/current_hp` / `referee/maximum_hp`
   必须在**同一命名空间**下被发布。⚠️ **当前仓库中尚无这些话题的发布者**，
   转发节点做出来之前，本节点会一直停在 `WAIT_GAME_START`（无害，但不会有动作）。
4. 另需注意：若 RViz/手柄正在占用 action，本节点会**主动让位**（这是正常行为，不是故障）。

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
