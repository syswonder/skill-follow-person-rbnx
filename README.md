# skill-follow-person-rbnx · 命令手册

Ranger Mini v3 上的 **MiniCPM-RobotTrack 跟随**整套操作命令。想先看"它是怎么搭的"，
见 [docs/OVERVIEW.md](docs/OVERVIEW.md)（原 README）和
[docs/specified-person-follow.md](docs/specified-person-follow.md)。

> **本仓不是 git 仓**。改配置前先看 §8；出问题按 §11 查。

---

## 0. 三条红线

1. **一次只起一套栈**：跟随栈和导航栈都占 `can_ranger` 和 `/cmd_vel`，**绝不同时跑**。
   起跟随栈之前先停导航栈。
2. **默认先 dry-run**：dry-run 下 go2_robottrack 节点**不建速度发布者、不起 chassis guard**，
   车不可能动。切 live 用 `scripts/follow.sh live`，**改完必须重起栈才生效**。
3. **`/cmd_vel` 的唯一写者是 chassis guard**。`ros2 topic info /cmd_vel` 的发布者数
   **0 = dry-run，1 = live，>1 = 立刻停**。

---

## 1. 约定

| 名字 | 值 |
|---|---|
| 宿主机仓库路径 | 本 README 所在目录（下面写 `<repo>`）。`scripts/run_container.sh` 内写死为 `/mnt/ssd/wxf/jiaofu/skill-follow-person-rbnx` |
| 跟随容器 | **`robonix_follow_dev`**（脚本里写死；碰不到导航容器 `robonix_ros_dev`） |
| 容器内路径 | `/root/deploy`（= 仓库 bind mount） |
| Robonix 工作区 | 宿主机 `/mnt/ssd/wxf/jiaofu/robonix` → 容器 `/root/workspace` |
| 模型树 | `<repo>/upstream/MiniCPM-Robot`（约 7.1 GB，挂载进容器，**不在容器里**） |
| ROS 环境 | `source /opt/ros/humble/setup.bash && export RMW_IMPLEMENTATION=rmw_zenoh_cpp` |
| docker | **需要 sudo**（没有 docker 组） |

宿主机侧命令几乎都经过这两个入口：

```bash
bash scripts/follow.sh <命令>       # 主机侧总入口（自己会要 sudo 密码）
bash scripts/run_container.sh follow  # 重建容器
```

---

## 2. 主机侧总入口 `scripts/follow.sh`

| 命令 | 作用 |
|---|---|
| `bash scripts/follow.sh status` | 看一眼现在的状态：容器 / 模式 / 底盘控制权 |
| `bash scripts/follow.sh check` | 静态四项检查（**起栈后、动车之前跑**） |
| `bash scripts/follow.sh watch` | 开实时仪表盘（Ctrl-C 退出） |
| `bash scripts/follow.sh shell` | 进容器交互终端（ROS 已激活、`cd /root/deploy`） |
| `bash scripts/follow.sh start` | 起跟随栈（约 80–85 s） |
| `bash scripts/follow.sh stop` | 停跟随栈 + 重启容器（清僵尸） |
| `bash scripts/follow.sh live` | 切"挂挡"（改 manifest，提示重起） |
| `bash scripts/follow.sh dry-run` | 切回"空挡" |

不用子命令时会打印上面这张用法表。

`check` 是四组静态判据（起栈后、动车之前必须全过）：

1. **推理服务**：`curl 127.0.0.1:5801/health` 里 `model_loaded/ok` 都为 true，并且
   `inference.log` 里能看到两塔的 TRT 横幅（0/1/2 会分别报"torch 或没热身 / 只有一半 / 两塔都行"）；
2. **意图路由**：`curl 127.0.0.1:18080/v1/models` 里有 `ranger-follow-distance-router`
   （挂了只影响语音调距，不影响跟随）；
3. **相机三路**：color / depth / camera_info 各 10 s 取一次 `ros2 topic hz`；
4. **`/cmd_vel` 发布者数**：0 = dry-run，1 = live，>1 = 立刻停。

最后顺带确认导航栈的 `ranger_base_node` 没在跑。整个 `check` 一般应打出 **16 条 `[ OK ]`**
（`start` 后也会打印"OK 条数（应为 16）"）。

`status` 判读：容器状态、manifest 里的 `mode:`、`/cmd_vel` 的 publisher 数、视觉后端，
以及"底盘控制权"两行。**注意**：底盘控制权是按 cgroup id（`7e99f182` / `af51b263`）归属的，
容器一重建 id 就变，这两行可能失真——此时以 `pgrep -f 'ranger_base_node|velocity_guard'`
和 `ros2 topic info /cmd_vel` 为准。

`start` 的行为细节（知道这些才好排障）：

- 先用 **50051 端口**判断"是不是已经在跑"（容器 PID 1 是 `tail -f /dev/null`，`pgrep` 会被
  僵尸骗，端口不会）；
- 后台 `docker exec -d` 跑 `scripts/start_follow.sh`，输出重定向到 `logs/start_follow.log`；
- 等 85 s，然后打印日志里的 `[ OK ] / [FAIL]` 尾巴和 **OK 条数（基线 16）**；
- 被 `start_follow.sh` 的预检拒绝时，会把"拒绝启动"那一段原文贴出来。

`stop` 的行为细节：先 `scripts/shutdown-follow.sh`，再**重启容器**（`docker restart`；如果容器
`Init != true` 就改用 `run_container.sh follow` 重建），最后报告 `running / Init / 僵尸数`。
重启容器是刻意的：一次 teardown 会杀掉成百上千个进程，孤儿只能由父进程回收，重启是唯一能保证
进程表干净的动作。

---

## 3. 起服务

### 3.1 标准流程（主机侧）

```bash
bash scripts/follow.sh status
bash scripts/follow.sh start
bash scripts/follow.sh check
bash scripts/follow.sh watch          # 另开一个终端
```

### 3.2 起栈时到底发生了什么（`scripts/start_follow.sh`）

它按顺序做这些事，**任何一步失败都拒绝启动、不会起半套栈**：

1. **读 `.env`**（`.env` 不存在就只用默认值）；
2. **预检**：
   - 50051/50061/50071/50081/50091/50093 是否有端口被占（占着 = 上一轮没停干净，或导航栈在跑）；
   - 容器里有没有僵尸（有就提示重建容器，僵尸会让 `pgrep` 误判）；
   - `can_ranger` 是否存在；
   - 视觉后端若是 `trt_direct`（默认），检查两个 engine 和 TensorRT 10.7 树在不在；缺了就给
     `ROBOTTRACK_VISION_BACKEND=torch` 的回退命令；
3. **起 MiniCPM-RobotTrack 推理服务**（127.0.0.1:5801，日志 `logs/inference.log`，
   pid `.run/inference.pid`）；已经在响应就跳过；
4. **起跟随距离意图路由**（127.0.0.1:18080，Pilot 的"VLM"，日志
   `logs/semantic_intent_router.log`，pid `.run/semantic_intent_router.pid`）。
   它的 `live/preview` **由 manifest 的 `mode:` 推出来**，不单独配；
5. `exec bash start.sh` → 必要时起 `rmw_zenohd`（7447，pid `.run/rmw_zenohd.pid`，日志
   `logs/rmw_zenohd.log`）→ `rbnx boot --no-update-check -f robonix_manifest.yaml`。

两个 sidecar **不归 rbnx 管**，所以只能由 `shutdown-follow.sh` 按 pid 文件停掉。

### 3.3 容器内手工起/停（调试用）

```bash
# 起：容器内前台（Ctrl-C 即停）
sudo docker exec -it robonix_follow_dev bash -lc 'cd /root/deploy && bash scripts/start_follow.sh'

# 停：容器内，完整的 teardown（rbnx + 两个 sidecar + 孤儿）
sudo docker exec robonix_follow_dev bash -lc 'cd /root/deploy && bash scripts/shutdown-follow.sh'

# 只停 rbnx 那一层（sidecar 会留着）
sudo docker exec robonix_follow_dev bash -lc 'cd /root/deploy && bash stop.sh'
```

**注意 `stop.sh` 不是全停**：它不杀推理服务和意图路由。完整 teardown 用
`shutdown-follow.sh`（它按 pid 文件杀 sidecar，并在最后核对 50051/50061/50071/50081/50091/
50093/7447 都已释放，成功时打印 `follow stack shutdown complete — ports released`）。

单个 sidecar 急救（例如推理服务假死、端口还在）：

```bash
# 推理服务（5801）
sudo docker exec robonix_follow_dev bash -lc 'kill $(cat /root/deploy/.run/inference.pid)'
# 意图路由（18080）
sudo docker exec robonix_follow_dev bash -lc 'kill $(cat /root/deploy/.run/semantic_intent_router.pid)'
# 下次 follow.sh start 会按需重新拉起
```

### 3.4 建/重建容器（宿主机）

```bash
bash scripts/run_container.sh follow
ROBONIX_FOLLOW_IMAGE=robonix:humble-orin-follow bash scripts/run_container.sh follow  # 指定镜像
```

容器参数是**刻意**的：`--init`（tini 回收僵尸，不可省）、`--network host`、
`--runtime nvidia -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all`
（推理在本机；**本机 `--gpus all` 已不可用**，会在 prestart hook 报
`Auto-detected mode as 'csv'`，脚本已改为前者）、`-v /dev:/dev`、
`--device-cgroup-rule 'c <usb_major>:* rmw'`（相机 + USB-CAN）。`run_container.sh` 每次都
**停→删→跑**，因为 `docker update` 改不了 device cgroup；build 产物在挂载盘里，不会丢。

---

## 4. dry-run（车不会动）

```bash
# 1) 切 dry-run（只改文件，不启停）
bash scripts/follow.sh dry-run
# 等价于：bash scripts/set_follow_mode.sh dry-run

# 2) 重起才生效
bash scripts/follow.sh stop
bash scripts/follow.sh start

# 3) 静态检查：/cmd_vel 发布者必须是 0
bash scripts/follow.sh check

# 4) 看预测（dry-run 只打日志：预测值 + 理由）
tail -f logs/start_follow.log
tail -f rbnx-boot/logs/go2_robottrack.log 2>/dev/null || tail -f rbnx-boot/logs/*.log
```

dry-run 下的验收判据：`check` 里 `/cmd_vel` 发布者 0 个；仪表盘能看到相机 → 检测 → 模型 →
cmd_vel_raw 每一跳有数；`/cmd_vel` 那一列恒为 0。

---

## 5. live（会动）

```bash
bash scripts/follow.sh live          # 改 manifest 的 mode 行；会自动跑一致性校验
bash scripts/follow.sh stop
bash scripts/follow.sh start
bash scripts/follow.sh check         # /cmd_vel 发布者必须是 1
bash scripts/follow.sh watch
```

`scripts/set_follow_mode.sh dry-run|live` 做的事：

- 只接受这两个值；manifest 缺失/是软链/`mode:` 行不唯一 → 拒绝；
- 已经是目标模式 → `already in … mode; nothing to do`；
- 改之前自动备份 `robonix_manifest.yaml.bak.<时间戳>`；
- 改完跑 `scripts/validate_manifest_consistency.py`，**失败就自动回滚**；
- 切 live 会打印 armed 警告（导航栈必须已停、场地净空、遥控器在手、被跟随的人知情）。

注意：**它不改意图路由的模式**——路由在 `start_follow.sh` 启动时按 manifest 推导
（dry-run→preview，live→live），所以必须是"改文件 + 重起"这一条路。

---

## 6. 看日志

| 路径 | 内容 |
|---|---|
| `logs/start_follow.log` | 起栈全过程（预检、两个 sidecar、rbnx boot 的 OK/FAIL） |
| `logs/inference.log` | MiniCPM-RobotTrack 推理服务；**视觉塔 banner 在这里**（`DINO/SigLIP backend = TRT FP16`） |
| `logs/semantic_intent_router.log` | 跟随距离路由（Pilot 的 VLM） |
| `logs/rmw_zenohd.log` | zenoh 路由 |
| `logs/build.log` / `logs/rbnx.log` | 构建 / rbnx 自己的日志 |
| `rbnx-boot/logs/` | 每个组件一份（atlas / executor / pilot / chassis / nav2 / go2_robottrack …） |
| `rbnx-boot/state.json` | `rbnx shutdown` 唯一的入口 |
| `.run/*.pid` | sidecar 的 pid（`inference.pid`、`semantic_intent_router.pid`、`rmw_zenohd.pid`） |

常用：

```bash
tail -f logs/start_follow.log
tail -f logs/inference.log
tail -f logs/semantic_intent_router.log
grep -a -c '\[server\] \(DINO\|SigLIP\) backend = TRT FP16' logs/inference.log   # 2 = 两塔都是 TRT
sudo docker logs -f robonix_follow_dev
```

⚠️ **rbnx 的日志用 `\r` 刷进度**，`start_follow.log` 直接 `grep` 会九成抓不到东西。
先转换再筛（`follow.sh start` 内部就是这么干的）：

```bash
sudo tr '\r' '\n' < logs/start_follow.log | grep -E 'OK|FAIL'
sudo tr '\r' '\n' < logs/start_follow.log | tail -40
sudo docker exec robonix_follow_dev bash -lc \
  'tr "\r" "\n" < /root/deploy/logs/start_follow.log | grep FAIL'
```

`logs/`、`.run/`、`rbnx-boot/` 里的文件都是容器以 root 写的，宿主机读要 `sudo`。
容器 stdout 不是主日志路径，所有内容都落在 bind mount 里的文件上。

`follow.sh status`/`check` 也会自动读 `inference.log` 的 banner——**banner 没出现就说明这一轮
还没真正用上 TensorRT 引擎**（引擎是第一帧才懒加载的）。

---

## 7. 看链路 / 话题

```bash
bash scripts/follow.sh watch          # = 容器内 python3 scripts/watch_follow_chain.py
```

一条 rclpy 节点订阅每一跳、每秒刷一张表。判读：

- 某一跳 **发布者 0** → 它的上一级没起来；
- 频率远低于 50 Hz → 那一级在卡；
- **vx 恒 0 而 wz 在动** → chassis guard 在压前进（Ranger 上是转弯半径钳制）。

手工 `ros2` 命令（容器内，先按 §1 导 ROS 环境）：

```bash
ros2 topic list
ros2 topic info /cmd_vel                 # 0 dry-run / 1 live / >1 危险
ros2 topic hz /camera/color/image_raw
ros2 topic hz /depth/image_raw 2>/dev/null
ros2 topic echo --once /ranger/robottrack/status   # 若该话题存在
rbnx list
rbnx path robonix-api                    # 看某个包/服务在容器里的落点
```

命令链与话题（README 原图，见 [docs/OVERVIEW.md](docs/OVERVIEW.md)）：

| 话题 | 谁写 → 谁读 |
|---|---|
| `/ranger/robottrack/cmd_vel_raw` | RobotTrack 桥 → 源选择器 |
| `/ranger/robottrack/nav_cmd_vel_raw` | Nav2 → 源选择器 |
| `/cmd_vel_nav` | 源选择器 → Nav2 速度平滑器 |
| `/cmd_vel_guard_input` | 平滑器 → Nav2 velocity guard |
| `/cmd_vel_guarded` | Nav2 guard → **Ranger chassis guard** |
| `/cmd_vel` | chassis guard → `ranger_chassis`（唯一写者） |

口子：50051 atlas / 50061 executor / 50071 pilot / 50081 liaison / 50091 soma / 50093 vitals /
5801 推理服务 / 18080 意图路由 / 7447 zenoh。

---

## 8. 改配置

### 8.1 `.env`（机器本地，不进包）

```bash
cp .env.example .env
$EDITOR .env
```

| 变量 | 作用 |
|---|---|
| `LIDAR_IP` | Mid-360 雷达 IP（建图 + Nav2 服务需要），默认示例 `192.168.1.112` |
| `RANGER_CAN_INTERFACE` / `RANGER_CAN_BITRATE` | 默认 `can_ranger` / `500000` |
| `ROBOTTRACK_UPSTREAM_ROOT` / `ROBOTTRACK_PYTHON` / `ROBOTTRACK_SERVER_HOST` / `ROBOTTRACK_SERVER_PORT` / `ROBOTTRACK_DEVICE` | 推理服务定位与设备 |
| `ROBOTTRACK_INSTRUCTION` / `ROBOTTRACK_TARGET_DISTANCE_M` | 服务端默认提示词 / 目标距离 |
| `ROBOTTRACK_VISION_BACKEND` | `trt_direct`（默认）或 `torch`（回退）；也有 `ROBOTTRACK_DINO_BACKEND` / `ROBOTTRACK_SIGLIP_BACKEND` 分塔设 |
| `SEMANTIC_INTENT_HOST` / `SEMANTIC_INTENT_PORT` | 意图路由监听，默认 127.0.0.1:18080 |
| `VLM_BASE_URL` / `VLM_API_KEY` / `VLM_MODEL` | Pilot 的 VLM 指向；**必须是 `http://127.0.0.1:18080/v1`**，否则语音白说 |
| `TENCENT_ASR_APPID` / `TENCENTCLOUD_SECRET_ID` / `TENCENTCLOUD_SECRET_KEY` | 腾讯语音 |
| `LIDAR_HOST_IP` | 宿主机雷达网口地址（默认 192.168.1.50，可覆盖） |

**注意**：`docker exec` 不继承宿主机 shell 的环境变量。所以
`ROBOTTRACK_VISION_BACKEND=torch bash scripts/follow.sh start` 由 `follow.sh` **显式转发**；
手工 `docker exec` 时导出的变量**不会**进容器——要么写进 `.env`（持久、推荐），要么加 `-e`。

两个坑：

- `ROBOTTRACK_TARGET_DISTANCE_M` 是**遗留键**，本仓代码没有任何地方读它；真正生效的是
  manifest 里的 `target_distance_m`。改这个环境变量没用。
- CAN 口变量名不一致：`start.sh` 读 `RANGER_CAN_INTERFACE`，`setup-hw.sh` 读
  `RANGER_CAN_NAME`（默认都是 `can_ranger`）。改了一个不等于改了另一个。

### 8.2 `robonix_manifest.yaml`

| 键 | 现值 | 说明 |
|---|---|---|
| `mode` | `live`（**当前文件就是这样**） | 用 `set_follow_mode.sh`/`follow.sh live|dry-run` 改 |
| `target_distance_m` / `min_…` / `max_…` | 1.0 / 0.5 / 7.0 | 定距跟随的目标与范围 |
| `distance_deadband_m` / `distance_kp` / `distance_max_forward_mps` | 0.10 / 0.50 / 0.50 | 定距闭环 |
| `distance_identity_max_age_s` / `distance_min_confidence` / `distance_pair_max_delta_s` | 0.60 / 0.50 / 0.20 | 指定人锁定 / RGB-D 配对 |
| `max_vx` / `max_wz`（节点侧） | 0.50 / 0.30 | 节点输出上限 |
| `guard_forward_only` | true | Ranger 只允许前进 |
| `guard_max_vx_mps` / `guard_max_wz_rps` | 0.25 / 0.40 | guard 侧硬上限（与 Go2 契约一致） |
| `guard_max_linear_accel_mps2` / `…angular…` | 0.30 / 0.80 | 加加速度限制 |
| `guard_command_timeout_s` | 0.25 | 超时停车（固件看门狗 500 ms 之前） |
| `guard_min_turn_radius_m` / `guard_turn_policy` | 0.81 / `clamp_yaw` | 厂商值，**未实测** |
| `model_input_mode` | `center_crop_height` | 模型输入取景 |

**校验器不是通用 linter**——它把几个调参值写死了：`max_vx` 必须 `0.50`；开定距时
`distance_deadband_m` 必须 `0.10`、`distance_kp` 必须 `0.50`；`guard_max_vx_mps` 必须
`<=` Nav2 smoother 的 `max_velocity[0]`（0.50）；`distance_identity_max_age_s` 不得大于
`distance_measurement_max_age_s`。改这些必须连校验器一起改，否则 `set_follow_mode.sh`
会判定失败并**自动回滚**。

实际速度上限别忘了：模型侧 `max_vx: 0.50`，但 chassis guard 只放 `guard_max_vx_mps: 0.25`；
转向受模型 `max_wz: 0.30` 限制。**真正能跑到的就是前进 0.25 m/s、转 0.30 rad/s。**

### 8.3 `config/`

| 文件 | 用途 |
|---|---|
| `config/nav2_params.yaml` | Nav2 参数 + smoother / velocity_smoother |
| `config/navigate.xml` | Nav2 行为树 |
| `config/rtabmap_params.yaml` | 建图参数 |
| `config/robottrack_assets.yaml` | MiniCPM 模型资产 pin（不下载） |

### 8.4 生效方式

| 改动 | 生效 |
|---|---|
| `mode` | **重起栈** |
| `.env`（含视觉后端、VLM 地址） | **重起栈**（容器内 source） |
| manifest 参数 | 校验 + **重起栈** |
| Nav2 / rtabmap 参数 | **重起栈** |
| 代码 | `bash build.sh` + 重起 |

---

## 9. rebuild

```bash
# 容器内（或 follow.sh shell 进去）
cd /root/deploy
bash build.sh                                   # = rbnx build -f robonix_manifest.yaml

# 原始形式（推荐带 --no-update-check，跳过联网检查）
rbnx build --no-update-check -f robonix_manifest.yaml     # 全部
rbnx build --no-update-check -p packages/go2_robottrack    # 只编一个包

# 更新 manifest 里 url: 引入的包到上游最新（只快进，不覆盖本地改动）
bash update-packages.sh                # 全部检查并快进，最后打印要执行的 rbnx build
bash update-packages.sh --check        # 只检查（= -n / --dry-run）
bash update-packages.sh -l             # 离线列出条目与 checkout 状态
bash update-packages.sh -f <manifest>  # 换 manifest
bash update-packages.sh --clone        # 只补 clone 缺失的包
bash update-packages.sh --force        # 强制快进（先跑 --check 确认）
# 网络慢可调： FETCH_TIMEOUT_S=120 bash update-packages.sh

# 历史分叉（diverged，快进不了）时的官方恢复路径
rbnx clean -f robonix_manifest.yaml --cache
rbnx build -f robonix_manifest.yaml

# 容器重建（换镜像 / 清僵尸 / 修 PID 1）
bash scripts/run_container.sh follow
```

几点要知道：

- `update-packages.sh` **只 ff、从不 reset**；它也不会动 vendored 的 `path:` 包（Nav2 是
  `third_party/service-navigation-rbnx`，要按 `third_party/README.md` 手动刷新）。
- `build.sh` 用的是不带 `--no-update-check` 的 `rbnx build`，所以可能多做一次联网检查；
  想要"纯离线构建"就自己敲上面的 `rbnx build --no-update-check -f …`。
- `rbnx-boot/cache/` 里的 url 包只在第一次 clone，之后**不会自动更新**，而本仓的启动脚本都带
  `--no-update-check`——想同步上游，只能显式跑 `update-packages.sh`。
- 构建完必须 `follow.sh stop` → `start`；若换了镜像/init/挂载，还得 `run_container.sh follow`。

ROS overlay 构建器 `scripts/build_robonix_ros2_overlay.sh` 是给包构建脚本 **source** 的
（直接执行会退出 2），不是操作命令。

---

## 10. 校验 / 测试 / 诊断

```bash
# 离线门：包内单测 + 一致性校验（不碰 ROS/相机/网络，导航栈在跑也能跑）
bash scripts/validate_offline.sh

# 只校验 manifest / soma / 包声明
python3 scripts/validate_manifest_consistency.py

# 模型资产 pin 校验（不联网）
python3 scripts/verify_robottrack_assets.py            # 可加 --json / --manifest / --upstream-root
#   ⚠️ 默认 upstream 根是 `/mnt/ssd/wxf`（脚本 parents[1] 的 bug），实际树在
#      本仓 upstream/MiniCPM-Robot，所以要显式给：
python3 scripts/verify_robottrack_assets.py \
  --upstream-root "$PWD/upstream/MiniCPM-Robot"

# 推理服务 + 视觉后端单测（容器内，不碰 ROS 和底盘）
bash scripts/_probe_vision_backend.sh trt       # 它自己起一个推理服务：先确认 5801 没人在用
bash scripts/_probe_vision_backend.sh torch
MINICPM_ROBOT_TRACK_ROOT=/root/deploy/upstream/MiniCPM-Robot \
  python3 scripts/_bench_vision_backends.py      # 还要有 TRT 的 PYTHONPATH，否则 import 不了

# 为什么定距层没有测量值（只读话题，不会动车；脚本里写死了 /root/deploy 路径，必须容器内跑）
python3 scripts/probe_person_detection.py 2>/dev/null

# 身份闸门 / 注册节流的离线复现
python3 scripts/_diag_no_identity_no_vx.py
PYTHONPATH=packages/go2_robottrack python3 scripts/_diag_enroll_throttle.py

# 相机预览往本地 RobotTrack 服务器推（只订阅，不发布 ROS 话题）
#   注意 --rgb-topic 的默认值是 Go2 时代的，Ranger 必须显式给
python3 scripts/robottrack_camera_preview_bridge.py \
  --rgb-topic /camera_435i/color/image_raw --server-url http://127.0.0.1:5801/eval_dual
```

意图路由 smoke test（容器内；本机有 `http_proxy` 时要清掉，否则 127.0.0.1 的请求会被代理吞掉）：

```bash
env -u http_proxy -u https_proxy -u all_proxy -u HTTP_PROXY -u HTTPS_PROXY -u ALL_PROXY \
curl -s --noproxy '*' -X POST http://127.0.0.1:18080/v1/chat/completions \
  -H 'Content-Type: application/json' -d '{
    "messages":[
      {"role":"system","content":"- capability_name: go2_robottrack.follow_distance\n"},
      {"role":"user","content":"靠近一点"}
    ]}'
```

---

## 11. 排障速查

| 现象 | 先查 |
|---|---|
| `follow.sh start` 立刻说"已经在运行" | 50051 被占：`ss -tuln \| grep 50051`。要么是上一轮，要么是导航栈 |
| `start.sh` 报"拒绝启动：这些端口已被占用" | 同样：先 `shutdown-follow.sh`，或先在导航侧停栈 |
| 导航栈还在占底盘 / 端口 | 导航侧停：`cd /mnt/ssd/wxf/jiaofu/robot-agilex-ranger_mini_v3 && sudo bash shutdown-robonix.sh`；恢复：`bash start-no-arm.sh` |
| 不想整个视觉后端回退 torch | 可以分塔：`ROBOTTRACK_DINO_BACKEND=torch bash scripts/follow.sh start` 或 `ROBOTTRACK_SIGLIP_BACKEND=torch …` |
| 容器里一堆 `<defunct>` | PID 1 不回收。`run_container.sh follow` 重建（换带 `--init` 的容器） |
| `pgrep` 说在跑但端口是空的 | 僵尸骗了 `pgrep`。以端口为准 |
| 起栈被拒："engine 不在 …trt_artifacts" | 视觉引擎缺失：`ROBOTTRACK_VISION_BACKEND=torch bash scripts/follow.sh start`，或把 engine 补回去 |
| 起栈被拒："TensorRT 10.7 树不在" | 容器镜像被换过：确认 `/root/tensorrt10.7/TensorRT-10.7.0.23`，或切 torch |
| 推理服务起不来 | `tail -25 logs/inference.log`；确认 `upstream/MiniCPM-Robot` 挂载还在 |
| 语音调距没反应 | `VLM_BASE_URL` 是否等于 `http://127.0.0.1:18080/v1`；`tail -f logs/semantic_intent_router.log` |
| 车原地打转不前进 | 正常现象的解释：Ranger 在半径小于 `min_turn_radius` 时会**默默丢掉 linear.x**；guard 已按 `clamp_yaw` 钳住。若还是转，查 `watch` 里 `/cmd_vel_guarded` |
| `/cmd_vel` 发布者 >1 | **立刻 `follow.sh stop`**：有别的进程也在控车 |
| 相机没图 | `lsusb \| grep -i orb`；容器重建有没有带 USB cgroup 规则 |
| CAN 不通 | `ip -details link show can_ranger`；`./setup-hw.sh` |
| 跟随距离不对（离线+7 m / 在线+1 m） | 参考 `docs/specified-person-follow.md` 的定距语义；先 `probe_person_detection.py` 看测量值 |

---

## 12. 文档导航

| 文件 | 给谁看 |
|---|---|
| [docs/OVERVIEW.md](docs/OVERVIEW.md) | 原 README：移植关系、命令链、脚本调了什么 |
| [docs/RUNBOOK.md](docs/RUNBOOK.md) | 保姆级操作与验收（含逐项判读表） |
| [docs/USER_GUIDE.md](docs/USER_GUIDE.md) | 使用说明 |
| [docs/跟随功能使用说明书.md](docs/跟随功能使用说明书.md) | 中文使用手册 |
| [docs/specified-person-follow.md](docs/specified-person-follow.md) | 设计全记录（含视觉后端、定距、身份闸门） |
| [docs/ACCEPTANCE.md](docs/ACCEPTANCE.md) | 验收清单 |
