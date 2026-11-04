# 跟随栈验收清单（Ranger Mini v3）

对照 `Robonix_MiniCPM_RobotTrack部署文档.pdf` §15.5 与附录 A，去掉 Go2 专属项、
换成 Ranger 的对应物。**每一项都要留下证据文件**；没有证据的项不能对外说"已通过"。

## 0. 前置（不满足就不要往下走）

- [ ] 导航栈已停：`pgrep -f 'ranger_base_node|velocity_guard'` 无输出
- [ ] `can_ranger` 空闲，且本栈是唯一 owner
- [ ] 跟随容器已起，仓库挂载在 `/root/deploy`
- [ ] `.env` 已填：`LIDAR_IP`、`TENCENT_ASR_APPID`、`TENCENTCLOUD_SECRET_ID`、
      `TENCENTCLOUD_SECRET_KEY`；`VLM_BASE_URL` 指向本机 router
- [ ] 遥控器在手，测试区域清空，目标距离 5 m 有足够进退空间

## 1. 离线门（不碰硬件，可随时跑）

```bash
bash scripts/validate_offline.sh
```

- [ ] `go2_robottrack` 132 项通过
- [ ] `semantic_intent_router` 38 项通过
- [ ] manifest 一致性 9 项通过（命令链、配置键、provider、live 档位、
      pre-smoother opt-in、限速上限）

## 2. 模型与推理服务

- [ ] `curl --noproxy '*' --max-time 3 http://127.0.0.1:5801/health` → `model_loaded: true`
- [ ] 推理设备是 `cuda:0`，不是 CPU
- [ ] `logs/inference.log` 无 CUDA/cuDNN 报错

## 3. 意图路由（Pilot 的"VLM"）

- [ ] `curl http://127.0.0.1:18080/v1/models` 返回 model 列表
- [ ] 预处理：`curl -s http://127.0.0.1:18080/v1/chat/completions -d '{...保持3米...}'`
      返回的 RTDL 里恰好一个 `go2_robottrack.follow_distance` 叶子
- [ ] 相对口令只下发一次：返回里 `adjust` 的 `meters` 为 `1.0` / `-1.0`
- [ ] 无关语句（"去厨房"）返回空 RTDL

## 4. 干运行（`mode: dry-run`，默认）

- [ ] 相机三路约 5 Hz：`/camera_435i/color/image_raw`、
      `/camera_435i/depth/image_raw`、`/camera_435i/color/camera_info`
- [ ] 人在画面中左右移动，模型输入与预测随之变化
- [ ] **`/cmd_vel` 上 publisher 数为 0**（干运行不发速，这是硬要求）
- [ ] 深度测距数值合理，人靠近/远离时随之变化

## 5. 命令链（切 live 后）

逐段验，任一段断了就往回查：

- [ ] `/ranger/robottrack/cmd_vel_raw` 有数据，非零占比高，`vx ∈ [0, 0.50]`、`wz ∈ [-0.30, 0.30]`
- [ ] `/cmd_vel_nav` 有数据（source mux 的唯一输出）
- [ ] `/cmd_vel_guard_input` 有数据（velocity smoother 输出）
- [ ] `/cmd_vel_guarded` 有数据（velocity guard 输出）
- [ ] **`/cmd_vel` 上 publisher 数恰好为 1**（只有 chassis guard）
- [ ] 底盘确实在动，且**没有出现"原地打转不前进"**
- [ ] `guard` 日志里能看到 `clamp_yaw` 生效的记录（说明约束在工作）
- [ ] `linear.y` 全程为 0，底盘没有切换转向模式

## 6. 固定距离与语音调距

按参考文档 §15.3 的顺序：

- [ ] 目标约 5 m 时 `vx ≈ 0`
- [ ] 拉到 5.2 m 时 `raw vx ≈ 0.05 m/s`
- [ ] 拉到 5.5 m 时 `raw vx ≈ 0.20 m/s`
- [ ] 拉到 6.1 m 以上时 `raw vx` 到 0.50 上限
- [ ] 说"离我远一点"：目标 5 → 6 m，车停止前进等人拉开
- [ ] 说"靠近一点"：目标 6 → 5 m，车主动向前
- [ ] 说"保持 1 米"：目标切到 1 m
- [ ] 每次相对口令**只执行一次**，不累加

## 7. 长稳与停止

- [ ] ≥ 10 min 干运行无 OOM、无 thermal throttle
- [ ] ≥ 5 min 运动窗口内 raw/mux/final 三路命令连续
- [ ] `tegrastats` 记录了 RAM / SWAP / GR3D_FREQ / 温度
- [ ] 停止后明确进入 `DISARMED`，底盘不再持有控制权
- [ ] `bash scripts/shutdown-follow.sh` 后 `can_ranger` 与各端口确实释放

## 8. 对本体的回归（硬性）

- [ ] 跟随栈关掉后，**导航栈能正常起来**（建图/定位/导航都正常）
- [ ] `robot-agilex-ranger_mini_v3` 仓**零改动**（`git status` 无新增修改）

## 证据留存

参考附录 B 的目录约定，至少要留：

- `git rev-parse HEAD`、模型 revision
- `uname -a` / L4T / CUDA / TensorRT / torch / ROS 版本
- 相机三路频率、最大帧间隙
- 推理 p50/p95 与 FPS
- `tegrastats` 功耗与温度轨
- 各段 cmd_vel topic 的计数与取值范围
- 每次语音调距的 ASR 文本 + Pilot/Executor 响应
- 起始/结束时间、目标距离序列、最终 `DISARMED` 状态

## 与参考渲染结果的差异（逐条说明）

参考的定距跟随入口渲染出的清单是：system `atlas/executor/pilot/liaison/soma`；
primitive `go2_chassis/go2_sensors/robot_description/audio_client_bridge/go2_d435i/go2_robottrack`；
service `mapping/speech/nav2/go2_dashboard`；skill `semantic_navigation`。
本仓的对应关系：

| 参考项 | 本仓 | 说明 |
|---|---|---|
| `go2_d435i` | `front_camera`（Orbbec） | 硬件替换 |
| `go2_sensors` | `mid360_lidar` + `mid360_imu` | 硬件替换 |
| `go2_chassis` | `ranger_chassis` + chassis guard | **唯一授权的偏离** |
| `go2_dashboard` | **未包含** | Go2 专用只读 UI（D435i/Mid-360 topic 名写死）；文档 §17.7 明确"不要启动不使用的 dashboard" |
| `semantic_navigation` skill | **未包含** | 文档 §14.1 的纯跟随最小 profile 不含它；本仓 router 为 follow-only，不含地标意图族 |
| 其余 | 一致 | mapping / speech / nav2 / pilot / robot_description / audio bridge |

## 当前状态（诚实口径）

- ✅ 离线门全过（170 项测试 + 9 项清单一致性）
- ✅ 底盘 4WS 语义问题已定位并有 guard 修复，离线量化 35% → 0%
- ✅ 发现并修掉一个会静默失效的坑：Nav2 的 `controller_velocity_output_topic`
  不在上游 `main` 上，`branch: main` 拿不到 → 已按参考的做法 vendor 到
  `third_party/`，并加了离线断言防回归
- ⚠️ **尚未做过任何 ROS 实机验证**——写这份文档时导航栈正在运行，
  碰 `/cmd_vel` 会驱动真车，所以第 4～8 节全部**待跑**
- ⚠️ `guard_min_turn_radius_m = 0.81` 来自厂商手册而非实测，需在 §5 阶段标定
- ⚠️ 本仓 vendor 的 nav2 服务与导航仓用的是**不同 revision**（本仓带
  pre-smoother opt-in）。这是刻意的，但意味着两栈的 Nav2 行为不完全同源。
