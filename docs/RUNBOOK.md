# 跟随功能 · 保姆级操作与验收手册

**你只需要敲一种命令**：`bash scripts/follow.sh <动作>`。
所有容器名、ROS 环境、路径都封在脚本里，不会打错。

**红线**：本手册所有操作**只动跟随**（容器 `robonix_follow_dev`、仓 `skill-follow-person-rbnx`）。
导航（容器 `robonix_ros_dev`、仓 `robot-agilex-ranger_mini_v3`）**只读不改**。
`follow.sh` 里的容器名是写死的，不可能误伤导航。

---

# 第 0 步 · 开工前的环境

## 0.1 打开终端，进到目录

在**宿主机**（就是你平时敲命令的那台机器）打开一个终端：

```bash
cd /mnt/ssd/wxf/jiaofu/skill-follow-person-rbnx
```

> 后面所有命令，**如果没有特别说明，都在这个目录下、都在宿主机上敲**。

## 0.2 先输入一次 sudo 密码

脚本要用 docker，需要管理员权限。第一次先做一次：

```bash
sudo -v
```

会提示 `[sudo] password for wangxiaofeng:`，输入密码（**输入时屏幕不显示，是正常的**）。
输完没报错就行。**这一步只做一次**，之后 5 分钟内不用再输。

> 如果跳过这步直接跑后面的命令，会看到 `需要 sudo 权限，但当前环境无法输入密码`，
> 按提示回来做这一步即可。

---

# 第 1 步 · 看一眼现在什么状态

```bash
bash scripts/follow.sh status
```

**正常应该看到**（下面是我刚才实跑的真实输出）：

```
== 容器 ==
robonix_follow_dev	Up 43 hours
robonix_ros_dev	Up 2 days

== 模式 ==
  manifest: mode:dry-run
  /cmd_vel: Type: geometry_msgs/msg/Twist Publisher count: 0 Subscription count: 1

== 底盘控制权 ==
  跟随栈的底盘进程: 1
  导航栈的底盘进程: 0  (必须是 0 —— 两个栈不能同时开)
```

## 怎么判读

| 看什么 | 正常值 | 不对怎么办 |
|---|---|---|
| 两个容器都在 | `Up xxx` | 跟随容器没了 → `bash scripts/run_container.sh follow`；导航容器没了 → 检查是否被人为关过 |
| `Publisher count` | `0`（dry-run）或 `1`（live） | 是 `>1` → **立刻** `bash scripts/follow.sh stop`，有多个程序在指挥车 |
| **导航栈的底盘进程** | **必须是 0** | 不是 0 → 导航还开着，先停导航 |

**这一步没通过，不要往下走。**

---

# 第 2 步 · 准备物品（人去做的事）

在切"挂挡"之前，请把下面这些备齐。**缺任何一项都不要继续。**

## 2.1 场地

- [ ] 车周围 **前后左右各 8 米**清空（目标距离最大 7 米，留余量）
- [ ] 地面平整，没有台阶、门槛、水渍
- [ ] 光线够亮（太暗 AI 认不出人）
- [ ] 没有其他人乱入（AI 分不清跟谁）

## 2.2 人员

- [ ] **一个"被跟随的人"**：知道接下来要配合走位，穿的衣服颜色和背景有区别（提高识别率）
- [ ] **一个"操作员"（你）**：手持遥控器，站在车后方或侧后方，**不要站在车正前方**
- [ ] 两人商量好：一出问题，谁按急停

## 2.3 器材

- [ ] **遥控器**：电池有电；**先按一下确认能控制车**（比如手动推一下车，确认能推得动、能停）
- [ ] **急停键位置**：现在就用手指确认一遍按哪个键
- [ ] 卷尺（可选，但强烈建议——量距离比目测准得多）
- [ ] 手机（可选，用来计时和拍照留证）

## 2.4 确认底盘没有"被远程抢走"

```bash
bash scripts/follow.sh status
```

再看一眼 `导航栈的底盘进程: 0`。

---

# 第 3 步 · 切到"挂挡"（live）

## 3.1 切模式

```bash
bash scripts/follow.sh live
```

**应该看到**：

```
manifest mode -> live
  [ OK ] command chain: nav2 velocity guard -> /cmd_vel_guarded -> chassis guard
  [ OK ] command chain: chassis guard -> /cmd_vel -> ranger_chassis
  [ OK ] command chain: source mux -> /cmd_vel_nav -> velocity smoother
  [ OK ] config keys: all 46 robottrack config keys are recognized
  [ OK ] providers: 9 providers declared by soma.yaml
  [ OK ] follow profile: live profile values match the reference (0.50 / 0.10 / 0.50)
  [ OK ] follow profile: command mode is 'live' (ARMED)
  [ OK ] pre-smoother opt-in: nav2 vendored and exposes controller_velocity_output_topic (/ranger/robottrack/nav_cmd_vel_raw)
  [ OK ] guard ceiling: guard 0.25 m/s <= smoother 0.5 m/s
  [ OK ] guard ceiling: Nav2 lifecycle manages smoother_server + velocity_smoother

  ============================================================
   LIVE MODE ARMED.  Before restarting the stack, confirm:
     * the navigation stack is stopped (both own can_ranger)
     * the test area is clear and the remote controller is in hand
     * the person being followed knows the robot will move
   The chassis guard starts with live mode and is the only
   writer of /cmd_vel.
  ============================================================
```

**如果看到的是 `[FAIL]`**：脚本会自动把配置还原，不会留下坏状态。
把失败信息发我，不用自己修。

## 3.2 重起（**必须**，模式改了不重起不生效）

```bash
bash scripts/follow.sh stop
```

**应该看到**结尾是：

```
[ OK ] follow stack shutdown complete — can_ranger + ports released
```

然后：

```bash
bash scripts/follow.sh start
```

这一步要 **75 秒左右**（脚本自己会等）。**应该看到** 18 条 `[ OK ]`，最后一行是：
`OK 条数: 18 （应为 18）`

关键的那一行是：

```
[ OK ]  pilot   :50071  vlm=ranger-follow-distance-router@127.0.0.1:18080
```

**看到 `[FAIL]` 或 OK 条数不是 18** → 把输出发我，先别动。

---

# 第 4 步 · 静态检查（**人还没动，车必须静止**）

```bash
bash scripts/follow.sh check
```

**应该看到四项全 OK**（我实跑的输出）：

```
-- 1/4 推理服务 --
[ OK ] 推理服务已加载模型
-- 2/4 意图路由（Pilot 的 VLM）--
[ OK ] 意图路由在线
-- 3/4 相机三路 --
[ OK ] /camera_435i/color/image_raw  30.494 Hz
[ OK ] /camera_435i/depth/image_raw  28.935 Hz
[ OK ] /camera_435i/color/camera_info  29.972 Hz
-- 4/4 底盘唯一控制者 --
[ OK ] /cmd_vel 发布者 1 个 —— live，车可以动
```

## 判读表

| 项 | 期望 | 不符合怎么办 |
|---|---|---|
| 1 推理服务 | `已加载模型` | 查 `sudo docker exec robonix_follow_dev bash -lc 'tail -40 /root/deploy/logs/inference.log'` |
| 2 意图路由 | `在线` | 语音会失效，但**跟随本身不受影响**，可以先继续 |
| 3 相机三路 | 都有 Hz（20~30 正常） | 某一项 `无数据` → **停下**，摄像头问题 |
| 4 底盘发布者 | **`1 个`** | `0 个` → 栈没起全；`>1 个` → **立刻 stop** |

**⚠️ 此刻车必须完全静止、没有任何抖动。** 如果车在动 → **立刻按遥控器急停**，然后 `bash scripts/follow.sh stop`。

---

# 第 5 步 · 开仪表盘

**再开一个终端**（原来那个留着，别关）：
- 如果用的是图形界面：右键终端 → 新建标签页
- 如果是 SSH：再开一个 SSH 连接，或按 `Ctrl+Shift+T`

在新终端里：

```bash
cd /mnt/ssd/wxf/jiaofu/skill-follow-person-rbnx
sudo -v
bash scripts/follow.sh watch
```

**会看到每秒刷新的表格**：

```
follow command chain   15:32:07   (Ctrl-C 退出)

topic                        pub  sub  Δmsg        vx        wz  备注
--------------------------------------------------------------------------------------------
/ranger/robottrack/cmd_vel_raw  1    1    50    +0.203    +0.031
  模型原始输出（未融合距离）
/cmd_vel_nav                    1    2    50    +0.203    +0.031
  source mux 唯一输出
/cmd_vel_guard_input            1    2    20    +0.203    +0.031
  velocity smoother 输出
/cmd_vel_guarded                1    2    20    +0.203    +0.031
  velocity guard 输出
/cmd_vel                        1    2    50    +0.203    +0.031
  chassis guard 输出 -> 底盘

检查点: /cmd_vel 的 pub 必须恰好为 1（只有 chassis guard）
检查点: /cmd_vel_nav 的 pub 必须恰好为 1（只有 source mux）
注意: sub 计数包含本观测器自己，所以比实际多 1
```

## 表格逐列读法

| 列 | 含义 | 怎么看 |
|---|---|---|
| `topic` | 指令走过的 5 个站点，从上到下 | 上→下 = 指令的流向 |
| `pub` | **有几个程序在往这里发** | 每一行都应该是 `1`。**是 0 → 上一级没在跑；大于 1 → 危险** |
| `sub` | 有几个程序在听 | 比实际多 1（仪表盘自己也在听） |
| `Δmsg` | 这一秒收到几条消息 | 50 左右正常；`0` = 这一站没数据 |
| `vx` | **前进速度**（米/秒） | 正数=往前，0=不动。**不可能为负**（代码禁止倒车） |
| `wz` | **转向速度**（弧度/秒） | 正数=左转，负数=右转 |
| `备注` | 自动标出的异常 | 有字就是有问题 |

## 三种必须立刻停的情况

1. 任何一行 `pub` **大于 1** → 有多个程序抢方向盘 → **立刻 `bash scripts/follow.sh stop`**
2. `/cmd_vel` 那行 `pub = 0` 但你在 live 模式 → 车不会动，栈没起全
3. 车在动，但仪表盘所有 `vx` 都是 0.000 → 有别的程序在开车

---

# 第 6 步 · 定距测试（**核心，对照数值表**）

## 6.0 先理解车会怎么动

车保持的距离 = **目标距离**，出厂默认 **5.0 米**。它这样算速度：

```
误差 = 人离相机的实际距离 − 目标距离
误差绝对值 ≤ 0.10 米  →  不动（这叫"死区"）
否则  前进速度 = 0.50 × (|误差| − 0.10)，最大不超过 0.50 米/秒
转向速度 = AI 模型说了算，最大 ±0.30
```

**目标 5.0 米时，照下表核对**：

| 人离车 | `/cmd_vel` 的 `vx` 应该是 | 车会做什么 |
|---|---|---|
| 4.50 米 | **0.00** | 人太近，**停住不动**（不倒车） |
| 4.90 – 5.10 米 | **0.00** | 在死区里，正常 |
| 5.20 米 | **约 0.05** | 非常缓慢地往前挪 |
| 5.50 米 | **约 0.20** | 明显加速追 |
| 6.10 米 及以上 | **约 0.50** | 全速追（到上限） |

> 距离靠目测有误差，**只看趋势对不对**（越远越快、太近停住）就算过，别纠结小数点。
> 有卷尺的话量一下更准。

## 6.1 操作步骤（一个动作一个动作来）

**准备**：被跟随的人走到车正前方；操作员（你）站在车侧后方拿遥控器，盯仪表盘。

---

**① 人站到车正前方 5.0 米，站住不动，等 10 秒。**

看仪表盘 `/cmd_vel` 那行的 `vx`：

- ✅ **`+0.000`** → 正确（死区），继续
- ❌ 有明显数值（比如 0.2）→ 说明测距不对或目标不对，**停下告诉我**

记录：`vx = ______`

---

**② 人慢慢往后退到约 5.2 米，站住，等 10 秒。**

- ✅ `vx` 变成 **0.03 ~ 0.08** 之间（理想 0.05）
- ❌ 还是 0.000 → 可能距离没到 5.2（死区 0.1 米，退得不够）
- ❌ 大于 0.15 → 测距偏大

记录：`vx = ______`

---

**③ 人继续退到约 5.5 米，站住，等 10 秒。**

- ✅ `vx` 变成 **0.15 ~ 0.25** 之间（理想 0.20）

记录：`vx = ______`

---

**④ 人退到 6.1 米以上（比如 6.5 米），站住，等 10 秒。**

- ✅ `vx` 变成 **0.45 ~ 0.50**（到上限封顶）
- ❌ 超过 0.50 → **停下告诉我**（限速没生效，这是安全问题）

记录：`vx = ______`

---

**⑤ 人慢慢走回到约 4.5 米（比目标近很多），站住，等 10 秒。**

- ✅ `vx` = **0.00**，车**停住**
- ✅ **车绝对不倒车**（`vx` 不可能是负数）
- ❌ 车往后退 → **立刻急停**

记录：`vx = ______`

---

## 6.2 记录表（可以直接填）

| 步骤 | 人的位置 | 期望 vx | 实测 vx | 通过? |
|---|---|---|---|---|
| ① | 5.0 米 | 0.00 | | ☐ |
| ② | 5.2 米 | ≈0.05 | | ☐ |
| ③ | 5.5 米 | ≈0.20 | | ☐ |
| ④ | ≥6.1 米 | ≈0.50 | | ☐ |
| ⑤ | 4.5 米 | 0.00（不倒车） | | ☐ |

---

# 第 7 步 · 转向测试

**操作**：让被跟随的人站在约 **5.5 米**处（这时候车应该在中速前进），然后：

1. 人**往左横着走** 2 米 → 车应该**一边前进一边左转**
2. 人**往右横着走** 2 米 → 车应该**一边前进一边右转**

**看仪表盘 `/cmd_vel` 那行的 `wz`**：

| 人的动作 | `wz` 应该是 |
|---|---|
| 往左走 | **正数**（0 到 +0.30 之间） |
| 往右走 | **负数**（0 到 −0.30 之间） |

**取值范围必须是 −0.30 ~ +0.30**。超过 → 停下告诉我。

## ⚠️ 这一步最关键：看车有没有"原地打转不前进"

**正确表现**：车**一边往前走、一边转**，能看出它在绕一条弧线接近人。

**错误表现**：车**停住不动，只在自己原地转圈** ← 这是 Ranger 底盘的老毛病，我加的保护层就是防这个。

**如果出现"原地打转"**：

1. 看这个输出（在第三个终端，或者用 `bash scripts/follow.sh shell` 进去）：

```bash
sudo docker exec robonix_follow_dev bash -lc \
  'tr "\r" "\n" < /root/deploy/logs/start_follow.log | grep "chassis guard" | tail -10'
```

2. 对照：

| 看到什么 | 说明 |
|---|---|
| `chassis guard adjusted command: yaw clamped to ...` | ✅ **保护层正在工作**，它把转向压小了、保住了前进 |
| `chassis guard adjusted command: holding zero: ...` | 上游断流了，查推理服务 |
| **什么都没看到，但车在打转** | ❌ 保护层没生效 → **急停 + 停下告诉我** |

记录：转向测试 ☐ 通过 ☐ 不通过

---

# 第 8 步 · 语音调距测试

**这一步需要 Robonix Client 连着**（电脑上那个客户端程序）。
如果客户端没连，可以跳过语音、改用文字（见 8.3）。

## 8.1 逐条口令

**每条说完，等 3 秒，再看结果。**

| # | 说什么 | 目标距离应变成 | 车会做什么 |
|---|---|---|---|
| 1 | **保持 5 米** | 5.0 米 | 基本没变化（本来就是 5） |
| 2 | **离我远一点** | 6.0 米 | **立即停止前进**，等人拉开到 6 米再继续追 |
| 3 | **离我远一点** | 7.0 米 | 同上 |
| 4 | **靠近一点** | 6.0 米 | **主动往前**缩短距离 |
| 5 | **保持 3 米** | 3.0 米 | 车会往前追到 3 米 |
| 6 | **设置为一点五米** | 1.5 米 | 验证中文小数能听懂 |

## 8.2 两条必须验的边界

**① "离我远一点"说一次只能加 1 米。**

说完一次看是不是 5→6。**反复说同一句不应该累加**——说三遍不该变成 8 米。
（这是路由器专门防的：收到执行反馈后不再重发。）

**② "保持 9 米"应该被拒绝。**

9 米超出允许范围（0.5~7.0 米）。正确反应是**明确拒绝**，
**而不是**悄悄改成 7 米。

## 8.3 怎么确认"目标真的变了"

**看路由器日志**：

```bash
sudo docker exec robonix_follow_dev bash -lc \
  'tail -20 /root/deploy/logs/semantic_intent_router.log'
```

**不连客户端也能测文字**（复制整段到终端，一次一行）：

```bash
sudo docker exec robonix_follow_dev bash -lc 'curl -s --noproxy "*" -X POST http://127.0.0.1:18080/v1/chat/completions -H "Content-Type: application/json" -d "{\"messages\":[{\"role\":\"system\",\"content\":\"- capability_name: go2_robottrack.follow_distance\n\"},{\"role\":\"user\",\"content\":\"靠近一点\"}]}"'
```

**返回什么，取决于路由器模式**（它现在自动跟着栈模式走）：

| 栈模式 | 路由器模式 | 应该看到 |
|---|---|---|
| `live` | live | `"cap": "go2_robottrack.follow_distance"`, `"meters": -1.0` ✅ 真的会调能力 |
| `dry-run` | preview | `"children": []`，文字里写"**当前是预览模式，未调用**" ✅ 只识别、不执行 |

**如果栈已经是 `live`，但返回的是 `children: []` 加"预览模式"** → 路由器没跟上栈模式。
这不该发生了（`start_follow.sh` 会自动推导），真遇到就重起栈：

```bash
bash scripts/follow.sh stop && bash scripts/follow.sh start
```

> **为什么这条重要**：这正是我开发时踩到的坑——`.env` 里把路由器锁死在 preview，
> 结果栈切到 live 之后，语音指令**会被听懂、然后被拒绝执行**，表现就是"说什么都没用"。
> 现在路由器模式由 manifest 的 `mode:` 自动推导，一个开关同时管两边。

## 8.4 记录表

| # | 口令 | 期望目标 | 实际目标 | 通过? |
|---|---|---|---|---|
| 1 | 保持 5 米 | 5.0 | | ☐ |
| 2 | 离我远一点 | 6.0 | | ☐ |
| 3 | 离我远一点 | 7.0 | | ☐ |
| 4 | 靠近一点 | 6.0 | | ☐ |
| 5 | 保持 3 米 | 3.0 | | ☐ |
| 6 | 设置为一点五米 | 1.5 | | ☐ |
| 边界① | 说一次只加 1 米 | 不累加 | | ☐ |
| 边界② | 保持 9 米被拒 | 拒绝 | | ☐ |

---

# 第 9 步 · 停止与应急

## 9.1 三种停止方式（随时可用）

| 方式 | 怎么做 | 用在哪 |
|---|---|---|
| **急停（最快）** | 按遥控器急停键 | 任何意外 |
| **软停** | 说"停" / "停下" | 正常结束跟随 |
| **断流停** | 关掉推理服务 | 验证保护层兜底 |

## 9.2 验证"上游断了车会自己停"

**这一条很重要**，验证保护层的兜底能力：

1. 让车正常跟随中（人在走，车在追）
2. **记录时间**（看手机秒表），然后执行：

```bash
sudo docker exec robonix_follow_dev bash -lc \
  'kill $(cat /root/deploy/.run/inference.pid) 2>/dev/null; echo killed'
```

3. **盯仪表盘**：`/cmd_vel` 那行的 `vx` 应该在 **0.25 秒内归零**，车停下

| 结果 | 判定 |
|---|---|
| 车在 1 秒内停下 | ✅ 通过（代码里设的 0.25 秒超时） |
| 车继续走超过 2 秒 | ❌ **急停**，告诉我 |

4. 测完要恢复：`bash scripts/follow.sh stop` 然后 `start`（因为推理服务被杀了）

---

# 第 10 步 · 收工

## 10.1 切回空挡（重要——下次启动默认安全）

```bash
bash scripts/follow.sh dry-run
```

## 10.2 停跟随栈

```bash
bash scripts/follow.sh stop
```

**应该看到** `follow stack shutdown complete — can_ranger + ports released`

## 10.3 起导航，验证没被搞坏

```bash
cd /mnt/ssd/wxf/jiaofu/robot-agilex-ranger_mini_v3
bash start-no-arm.sh
```

起来后确认：**建图正常、定位正常、导航正常**。

顺便确认导航仓一个字节都没被改过：

```bash
cd /mnt/ssd/wxf/jiaofu
find robot-agilex-ranger_mini_v3 -newermt "$(date +%Y-%m-%d) 00:00" -type f \
  -not -path '*/.git/*' -not -path '*/rbnx-boot/*' -not -path '*/logs/*'
```

**空输出 = 没被动过** ✅

---

# 附录 A · 总记录表

| 阶段 | 检查项 | 结果 |
|---|---|---|
| 4 | 静态四项全 OK | ☐ |
| 4 | 此刻车完全静止 | ☐ |
| 6 | 定距 5 档全部符合 | ☐ |
| 7 | 转向方向正确 | ☐ |
| 7 | **没有"原地打转不前进"** | ☐ |
| 8 | 语音 6 条 + 2 条边界 | ☐ |
| 9 | 断流后 1 秒内停车 | ☐ |
| 10 | 导航恢复正常 | ☐ |
| 10 | 导航仓零改动 | ☐ |

---

# 附录 B · 想先不动车看看？(dry-run 观察)

**完全安全，车一步都不会动。** 适合先熟悉界面：

```bash
bash scripts/follow.sh status    # 确认 Publisher count: 0
bash scripts/follow.sh check     # 四项检查
bash scripts/follow.sh watch     # 开仪表盘（各行的 vx 都应该是 0）
```

浏览器打开 `http://192.168.1.50:5801/web`，能看到：
- **Raw camera** —— 摄像头原始画面
- **模型输入** —— 实际喂给 AI 的 384×384 图
- **overlay** —— AI 认出来的人

让人在镜头前左右走动，画面和 overlay 应该实时跟着变。**这时候车不会动。**

---

# 附录 C · 出问题速查

| 现象 | 先做这个 |
|---|---|
| 某个 `follow.sh` 命令报 sudo 错 | `sudo -v` 输一次密码 |
| `start` 后 OK 条数不是 18 | 把输出发我 |
| 仪表盘某行 `pub = 0` | 上一级没跑，`bash scripts/follow.sh stop` 再 `start` |
| 仪表盘某行 `pub > 1` | **立刻 stop** |
| 车原地打转不前进 | §7 的排查表 |
| 车完全不动但仪表盘有数据 | 看 `/cmd_vel` 的 `vx` 是不是被压成 0 |
| 语音没反应 | `bash scripts/follow.sh check` 看第 2 项 |
| 车不停 | **按遥控器急停**，然后 stop |
| 网页打不开 | 用 `192.168.1.50` 不是 `127.0.0.1`；地址要带 `/web` |
