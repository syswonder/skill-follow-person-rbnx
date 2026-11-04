# 指定跟随（Specified Person Follow）实施记录

## 目标

基于现有跟随栈，实现"只跟随指定的人"功能：
- 语音说"跟着我" → 锁定当前看到的人（ReID 特征入库）
- 之后只跟这个人，出现其他人也不切换
- 语音说"别跟了" → 解锁，恢复原有逻辑

---

## 方案概述

在现有跟随链路中插入 ReID 过滤层：

```
摄像头 → HOG 检测 → 所有框 → [ReID 门] → _select_target() → P 控制 → /cmd_vel
                          ↑
                      OSNet 512-dim
                      余弦相似度 ≥ 0.7
                      ↓ 无匹配
                  vx=0, wz=0 (绝不切人)
```

---

## 实施步骤

### Step 1: 新增 `person_reid.py` 模块 ✅

**位置**：`packages/go2_robottrack/go2_robottrack/person_reid.py`

**功能**：
- 加载 OSNet x1.0 模型（CUDA + FP16）
- 输入 bbox 图像，输出 512 维特征向量
- 提供 batch forward 接口（一次处理多个人）
- `enroll_template()`：多帧特征取均值 + L2 归一化

**状态**：✅ 完成，容器内测试通过

**测试结果**（2026-09-29）：
```
feature_dim: 512
n_feats: 2 | shape: (512,) | dtype: float32
cos(self)=1.0000  cos(diff_person)=0.1566
template norm: 1.0
empty: []
oob_allzero: True
REID_OK
```

**踩坑记录**：
- `global_avgpool` 输出是 `(N, 512, 1, 1)` 不是 `(N, 512)`，必须 `flatten(1)`，否则余弦相似度计算会 `TypeError: only length-1 arrays can be converted to Python scalars`
- 容器需显式传 `-e LD_LIBRARY_PATH=...`，否则 `import torch` 直接失败


### Step 2: 修改 `rgbd_distance.py` ✅

**改动点**：`_select_target()` 里、**任何关联/选择逻辑之前**插 ReID 门

**逻辑**：
1. 锁未激活 → 完全走原逻辑，零开销（不加载 OSNet 就不碰）
2. 锁激活且有待注册 → 对每个框提特征，选**最大框**（离相机最近＝下指令的人）累积样本，够数即激活
3. 锁激活 → 只保留余弦相似度 ≥ 阈值的框
4. 无匹配 → 返回 `reid-no-match` + `None`，且**跳过 center-fallback**（画面中央的陌生人正是本功能要防的情况）

**状态**：✅ 完成

**额外改动**：
- 新增 `RgbdDistanceConfig` 字段：`reid_enabled` / `reid_match_threshold` / `reid_enroll_samples` / `reid_min_bbox_height_px`
- 新增私有方法 `_enroll_candidate_index()`
- 新增 `source = "reid-no-match"`，与原有 `none` / `hog` / `hog-associated` / `short-track` 并列

### Step 3: 接入 `core.py` / `ros_node.py` / `provider.py` ✅

**core.py**：`RuntimeConfig` 新增 5 个字段 + 校验
- 约束：`reid_enabled` 必须 `distance_enabled`（ReID 门在 RGB-D 估计器里，需要逐帧的人框）

**ros_node.py**：加载 OSNet 一次（非每帧），把模型 + 共享锁状态交给估计器
- 加载失败**降级不中断**：打日志，退回原有跟随逻辑

**provider.py**：`follow_distance` 能力新增 `lock` / `unlock` 两个 operation
- **不需要新合约、不需要 IDL、不需要 codegen** —— 复用现有 `FollowDistance.srv` 的 `string operation` 字段
- 模型没加载成功时 `lock` 返回 `accepted=false`，避免"语音说成功但实际没锁"

**状态**：✅ 完成

### Step 4: 验证 ✅

**新增测试**：`tests/test_person_reid_lock.py`（23 个用例）

**全量测试**：
```
Ran 156 tests in 0.382s
OK
```
（133 个原有用例全过 + 23 个新增，无回归）

**关键行为验证**：

| 场景 | 结果 |
|---|---|
| HOG 置信度 0.99 的陌生人 + 0.10 的目标同时出现 | ✅ 仍只跟目标 |
| 画面里只有陌生人 | ✅ `reid-no-match` → None（车停，不切人） |
| 画面里没人 | ✅ `reid-no-match` → None |
| 解锁后 | ✅ 恢复原有关联逻辑，能重新捕获新人 |
| 不传锁状态 | ✅ 行为与改动前完全一致 |

**实测输出**：
```
enroll (A big, B small)            src=hog              bbox=A(10,100,200,380)
   lock: locked
only stranger B                    src=reid-no-match    bbox=None
A + B both present                 src=hog-associated   bbox=A(10,100,200,380)
   --- unlocked ---
unlock frame 4 (only B)            src=hog              bbox=B(300,100,60,120)
```

### Step 5: 语音意图识别 ✅

**位置**：`packages/semantic_intent_router/`

这是 Pilot 的 `VLM` 后端（本地语义路由 sidecar），不是 go2_robottrack 原语的一部分，
但它决定了**一句话能不能变成一次能力调用**。不接这一步，"跟着我"根本到不了 provider。

| 文件 | 改动 |
|---|---|
| `semantic_intent_router/person_lock.py` | **新增**：`parse_person_lock_intent()` |
| `semantic_intent_router/server.py` | 修改：`_preview()` / `decide()` 先试 lock 意图，再落到距离意图 |

**关键点**：
- **先判 unlock 再判 lock**。"别跟着我了" 里含 "跟着我"，顺序反了会把取消听成锁定
- 解锁模式写成 `(?:跟着|跟随|跟)` 而不是 `(?:跟着|跟随)`，否则 "别跟了" / "不用跟了" 匹配不上
- 复用现有 `robonix/primitive/follow/distance` 能力，只多传一个 `operation`，**没有新能力、没有新 RTDL 形状**

**状态**：✅ 完成

### Step 6: 部署接线 ✅

**`packages/go2_robottrack/start.sh`**：新增 `ROBONIX_TORCH_LIBRARY_PATH` → `LD_LIBRARY_PATH`
- 镜像里 torch 依赖的 cuDNN/CUDA 目录**不在默认 loader 路径**上（容器自身 `LD_LIBRARY_PATH` 为空），
  不设这一条 `import torch` 直接失败
- 变量为空时不改任何东西：普通跟随 profile 完全不受影响

**`robonix_manifest.yaml`**：
- `env:` 加 `ROBONIX_TORCH_LIBRARY_PATH`（值与 RobotTrack 推理服务保持一致）
- `go2_robottrack.config` 加 5 项：`reid_enabled: true` / `reid_model_dir: ${ROBONIX_DEPLOY_DIR}/models/osnet`
  / `reid_match_threshold: 0.70` / `reid_enroll_samples: 5` / `reid_device: cuda:0`

**状态**：✅ 完成，manifest 校验通过（51 个 robottrack 配置键全部识别）

### Step 7: 对外声明 ✅

- `packages/go2_robottrack/CAPABILITY.md`：说明 `lock` / `unlock` 复用同一能力、
  防切换保证、目标不在画面时 `vx=0` 保持、模型不可用时 `accepted=false`
- `capabilities/lib/go2_robottrack_control/srv/FollowDistance.srv`：仅补头部注释，**类型未变**
  （`rbnx-build/codegen/` 下的是构建产物，`build.sh` 会重新生成，不用手改）

**状态**：✅ 完成

### Step 8: 端到端验证 ✅

**新增测试**：
- `packages/go2_robottrack/tests/test_provider_contract.py` +4 个用例（provider 的 lock/unlock 契约）
- `packages/semantic_intent_router/tests/test_person_lock.py` 新增（13 个用例）

**全量测试**：
```
go2_robottrack:          Ran 160 tests in 0.394s   OK   (133 原有 + 23 Step4 + 4 Step8)
semantic_intent_router:  Ran  51 tests in 2.548s   OK   ( 38 原有 + 13 新增)
```

**关键行为验证**：

| 场景 | 结果 |
|---|---|
| 部署里没开 `reid_enabled` | ✅ `accepted=false` / `status=reid_unavailable` / detail 含 "not enabled" |
| 开了但模型没加载成功 | ✅ `accepted=false` / detail 含 "not loaded"（绝不假报成功） |
| `lock` → `unlock` 全流程 | ✅ 共享锁状态 `enrolling` → `unlocked`，`active=false` |
| `set` / `adjust` / `get` 距离操作 | ✅ 完全不受 lock 分支影响 |
| "跟着我" / "别跟了" 语音 | ✅ 各发出一个 RTDL leaf，`operation` 分别为 `lock` / `unlock` |
| "离我远一点" / "跟随距离设为3米" | ✅ 仍走原有距离路由（新分支不遮蔽旧分支） |
| 预览模式 | ✅ 不发任何能力 leaf |
| 执行失败回执 | ✅ 播报失败原因，不重放 |

**注意**：跑 router 测试时要清掉 `http_proxy`，否则 localhost 请求被 VPN 代理拦成 502：
```bash
http_proxy= https_proxy= no_proxy=127.0.0.1,localhost NO_PROXY=127.0.0.1,localhost \
  PYTHONPATH="$(pwd)" python3 -B -m unittest discover -s tests -p 'test_*.py'
```
这不是代码问题。

### Step 9: 容器内实机加载验证 ✅

**不是单测**：直接 import 真实的 `PersonReID` 类，用 manifest 里的原值
（`reid_model_dir=/root/deploy/models/osnet`、`reid_device=cuda:0`、
`reid_match_threshold=0.70`、`reid_enroll_samples=5`），
环境变量按 `start.sh` 的方式设 `ROBONIX_TORCH_LIBRARY_PATH`。

```
A. 加载        model_dir=/root/deploy/models/osnet  device=cuda:0
               -> 加载成功 2.20s   feature_dim=512   走 FP16
B. 推理        首帧(含预热) 125.20 ms | 稳态 36.67 ms
               同图自身 cos=1.0000
               A 相邻帧(同人) cos=0.9929   ← > 0.70 ✅
               A vs B(异人)   cos=0.5337   ← < 0.70 ✅
C. 注册+匹配    模板 512 维，L2 范数 1.0
               画面内 A+B: keep=[False, True]  陌生人不进候选集 ✅
               画面内只有 B: keep=[False]  status=locked_no_match ✅
D. 失败路径     目录不存在 -> FileNotFoundError
               device 非法 -> RuntimeError
               两者都被 ros_node 的 try/except 接住 -> 降级 ✅
结果: 全部通过 —— OSNet 在容器内加载成功，未降级
```

**结论：加载不会失败。** 之前担心的 `LD_LIBRARY_PATH` 空、cuDNN 找不到等问题，
已由 `start.sh` + manifest 的 `ROBONIX_TORCH_LIBRARY_PATH` 解决。

**踩坑记录（这次）**：第一版测试夹具里 `make_person()` 的随机噪声被后面的纯色块
整个覆盖，seed=1 和 seed=2 生成的是**完全相同的图**，于是 `cos(A,B)=1.0000`，
4 项断言假失败。这是夹具的 bug 不是代码的 —— 差异必须放在 resize 到 256×128 后
仍然保留的粗尺度上（色块布局 + 粗条纹），细噪声会被下采样平均掉。

**尚未验证**：A/B 是合成色块不是真人，所以只证明了"同人跨帧 > 异人"且两者分居
阈值两侧；真人场景下 0.70 这个值是否合适，需要真机复核。

### Step 10: 容器内 dry-run 实机联调 ✅（发现 1 个缺陷）

**前置**：`bash scripts/set_follow_mode.sh dry-run`（原为 live，脚本已自动备份 +
跑离线一致性校验，全部 OK）。此时 `can_ranger` 已 UP，无任何残留进程。

#### 10.1 全量跟随栈：起不来，但**不是新功能的问题**

```
[ 139.428] [FAIL] primitive  soma exited after provider failure(s): mid360_lidar
[Boot failed] ... logs: rbnx-boot/logs/mid360_lidar.log
```

`mid360_lidar.log`：`Init lds lidar fail!` + 3 次 respawn 后放弃。
根因是**宿主 `eno1` 是 DOWN**（本机只走 WiFi `wlP1p1s0`），雷达网段 192.168.1.112
根本不可达 —— 与本次改动无关。

栈按设计整体回滚（`rbnx shutdown` 是原子的：一个 provider 失败就全停）。

**但回滚前，新功能已经在真实进程里跑起来了**：

```
09:24:47.277  Loading OSNet x1.0 from /root/deploy/models/osnet ...
09:24:47.665  OSNet loaded in FP16 mode
09:24:47.666  OSNet ready on cuda:0, feature_dim=512
09:24:47.671  specified-person lock ready (threshold 0.70, 5 enroll samples)
09:24:47.678  RobotTrack active: mode=dry-run, rgb=/camera_435i/color/image_raw
09:24:47.679  state INACTIVE -> ACTIVE
09:24:47.690  Driver(cmd=3) received        ← 11ms 后收到 SHUTDOWN（栈在回滚）
09:24:47.696  [ERROR] RobotTrack inference request failed: ResponseNotReady: Idle
```

注意最后那条 `ResponseNotReady` 是**回滚的结果不是原因** —— SHUTDOWN 先到，
HTTP 连接被掐断才报的错。**没有走降级路径**（日志里没有
`specified-person lock unavailable`）。

#### 10.2 最小 profile：跑通了

雷达挡住全量 boot，所以生成一个临时 manifest（`robonix_manifest.reid-test.yaml`，
由真实 manifest 程序化裁剪而来）：

| | 保留 | 去掉 |
|---|---|---|
| primitive | `robot_description` `front_camera` `go2_robottrack` | `mid360_lidar` `mid360_imu` `ranger_chassis` `audio_client_bridge` |
| service | （无） | `speech` `mapping` `nav2` |

**安全性**：去掉了 `ranger_chassis` → **没有任何进程打开 `can_ranger`**；
robottrack 是 dry-run → 不发速度；chassis guard 只在 live 构造 → 不启动。
所以即使 router 临时开成 live，也不可能有任何运动。

```
✓ 6 component(s) up
go2_robottrack  state INACTIVE -> ACTIVE
                指定跟随锁 ready (threshold 0.70, 5 enroll samples)
front_camera    publishing intrinsics: fx=369.1 fy=369.3 cx=321.1 cy=244.1
```

#### 10.3 语音 → 能力：四句全部路由正确

`rbnx ask`（真实链路：atlas → Pilot → router → executor）：

| 说法 | router 识别 | 结果 |
|---|---|---|
| 跟着我 | 锁定当前看到的人 | ✅ 发 `{operation:"lock"}` |
| **别跟了** | 解除指定跟随 | ✅ 发 `{operation:"unlock"}`（**"跟着我"子串没被误判**） |
| 离我远一点 | 将跟随距离增加 1 米 | ✅ 仍走原有 adjust（新分支没遮蔽旧分支） |
| 跟随距离设为3米 | 设置跟随距离为 3 米 | ✅ 仍走原有 set |

dry-run 下 router 按设计是 `preview`，识别但**拒绝执行** —— 这是对的：

> 已识别"锁定当前看到的人"；当前是预览模式，未调用 RobotTrack 跟随能力

#### 10.4 执行：lock / unlock 走通真实能力接口

把 router 临时切成 live（**仅本次测试**，安全性见 10.2）后：

```json
// rbnx ask "跟着我" 发出的 plan
{"calls":[{"args_json":"{\"meters\":0.0,\"operation\":\"lock\"}",
           "contract_id":"robonix/primitive/follow/distance",
           "provider_id":"go2_robottrack"}]}

// 能力返回
{"accepted": true, "enabled": true, "active": false, "has_measurement": false,
 "target_distance_m": 1.0, "status": "enrolling",
 "detail": "collecting 5 samples of the target person"}

// rbnx ask "别跟了" 的能力返回
{"accepted": true, "status": "unlocked", "active": false,
 "detail": "specified-person lock cleared"}
```

`operation=lock/unlock` 正确落到 `reid_lock_state.request_enroll()` / `.clear()`，
**复用现有合约、没有新 IDL、没有重新 codegen、没有重新 build**。

#### 10.5 ⚠️ 发现的缺陷：注册没有超时

相机前没站人时，锁**永久卡在 `enrolling`**，且不会报失败。

**证据** —— 用真实估计器跑实时相机流（`scripts/probe_person_detection.py`）：

```
rgb frames 39   size 640x480
depth frames 39  size 640x480
pair         paired                            ← RGB/深度配对正常
estimate     valid=False  status=target_not_detected  source=none
             conf=0.000
```

相机在出图、配对也正常，但 **HOG 看不到人** → `usable` 为空。

**代码路径**：`rgbd_distance.py:729` 的门是

```python
if lock is not None and self._reid is not None and usable:   # ← usable 为空就整段跳过
    ...
    if lock.pending_enroll() > 0:
        lock.add_enroll_feature(features[enroll_index])       # ← 永远到不了
```

而 `pending_enroll` 只在 `add_enroll_feature()` 里递减，`ReidLockState` 里
**没有任何超时/过期机制**。所以：

- HOG 有框但裁剪全废 → 会递减，5 帧后正确报 `enroll_failed`（"no usable person crop"）
- HOG **完全没框**（人说"跟着我"时背对镜头 / 不在画面里）→ 永不递减，**卡死**

**影响**：用户说"跟着我"，机器人答"正在采集"，然后**永远**没有下文，
既不成功也不失败。用户无法判断。

**修法**：见 **Step 11** —— 最终没有采用"超时报失败"，而是改成
**不过期 + 每 2 秒循环播报**（用户要求"直到有人出现为止"）。

### Step 11: 注册等待改为循环播报 ✅（替代 10.5 的超时方案）

**决策修订**：10.5 提的「3 秒没注册成功就报失败」**作废**。用户的要求是
「循环提示，直到有人出现为止」：

- 操作者说了"跟着我"，意图就还在 —— 锁该一直等，而不是自己放弃
- 报 `enroll_failed` 会让人以为得重说一遍，其实只要走到镜头前就行
- 真正的 bug 不是"卡住"，是**卡住了不说话**：既没成功也没失败，用户无从判断

所以定稿为：注册**永不过期**，但**每 2 秒重新播报一次**仍在等待。

#### 11.1 实现

`person_reid.py`：

- 模块常量 `_ENROLL_PROMPT_INTERVAL_S = 2.0`
- `ReidLockSnapshot` 新增 `waiting_s` / `prompts`，**均带默认值**（旧构造调用不受影响）
- 新增 `note_enroll_wait(now=None, reason=None) -> str | None`：没在注册 → `None` 静默；
  距上次播报 < 2s → `None`；否则记时、计数、置 `status="enrolling_no_person"`，
  返回 `{reason}; still waiting to enrol the target (Ns elapsed, prompt #k)`
  （`reason` 由估计器给出，默认 `no person in view`；Step 12 起还会是
  `the person being enrolled is not in view`）
- `request_enroll()` 记 `_enroll_started_at`；`clear()` 三者全部复位；
  **`_pending_enroll` 不因等待而递减** —— 这就是"不过期"

`rgbd_distance.py` 的门拆出 `enroll_stalled` 分支，把"这一帧没凑出可注册的人"
和"锁还开着在等人"分开。节流放在**锁内部**，所以每帧调用一次也安全；
日志只在真有播报时打，间隔 ≥ 2s。

#### 11.2 离线验证：哪些路径会"等待"，哪些本来就终止

`scripts/_diag_enroll_throttle.py`（保留为常备诊断）用**真实**
`RgbdPersonDistanceEstimator` 以 ~30Hz 跑 6 秒：

| 路径 | 构造 | 结果 |
|---|---|---|
| `empty` | HOG 一个框都没有 | `prompts=3  gaps=[2.024, 2.016]` |
| `short` | 有框但太矮（OSNet 裁剪无效） | `prompts=3  gaps=[2.024, 2.025]` |
| `zerofeat` | 框够高但特征全 0 | `prompts=0  status=enroll_failed` |

前两条**只有**会走等待分支的路径：6 秒 3 次、间隔稳定 2.0s，锁状态
`enrolling_no_person`，`pending_enroll` 保持 5（不掉 → 确认不过期）。

第三条**不是等待路径**：候选（够高的框）找得到，`add_enroll_feature()` 照常递减，
5 帧后正确报 `enroll_failed`（"no usable person crop"）—— 这条路本来就有终止条件，
不需要等待播报。10.5 里"卡死"的只有"连框都没有"那两条。

#### 11.3 实机联调暴露的缺陷：播报抖动（已修）

离线两条路径间隔都很干净，但**实机**第一次测出了 4 次播报间隔
`0.42 / 0.63 / 0.27` 秒 —— 一个突发。

**根因**：`add_enroll_feature()` 在"收到样本但还没凑齐"的分支里把
`_last_prompt_at = None` 复位了。实机上 HOG 会闪（有人→无人→有人），
每闪一次就重置节流窗口 → 窗口一开立刻播报 → 听觉上变成连珠炮。
离线脚本的 HOG 是静态的，所以复现不出来。

**修法**：删掉那句复位。`_last_prompt_at` 只在 `request_enroll()` / `clear()`
里置 `None`，**收到样本也不提前解锁播报窗口** —— 间隔是硬上限。

对应回归用例改名加强为
`test_prompts_are_rate_limited_even_when_detections_flicker`。
`test_person_reid_lock.py` **167 个用例全绿**（原 160，本次 +7）。

#### 11.4 修后复核：抖动路径实跑（dry-run，未发任何速度）

在真机容器里用**正在运行的同一份代码**复现抖动：30Hz 循环，检测框每 25 帧
出现一次（即 `add_enroll_feature` 以 ~1.2/s 触发 —— 正是老代码复位的条件），
`enroll_samples=50` 以保持等待：

```
flicker-30Hz: status=enrolling | prompts=4 | pending=40 | gaps=[2.023, 2.016, 2.017]
```

8 秒 4 次、间隔 2.0s 硬上限，**没有突发**；`pending` 保持 40 → 确认**不过期**。
（对照：老代码下每次收到样本都会重置窗口，同条件应产出 ~30 次/秒。）

#### 11.5 当前环境下测不到的一条路径

"画面里**完全没人**"这一路在**本房间测不出来**：相机前始终有东西被检测到，
日志里全是 `state=too_close_forward_hold`（有测量值），修复后日志中
`still waiting to enrol` 出现 **0 次**。所以实机只覆盖了"抖动/半可见"，
"完全无人"由 11.2 的 `empty` 路径离线覆盖。

#### 11.6 安全声明

全部联调在 **dry-run** 下完成：manifest `mode: dry-run`；`go2_robottrack`
只输出 `dry-run prediction`，**零条速度发布**；`ranger_chassis` 只收到自身
init 配置行，**零条指令**。11.4 的复核只调用估计器的选择路径，不经过
相机、不经过 router、不发布任何话题。

#### 11.7 回归审查发现（未修，待决定）

对整份改动做过一次独立回归审查，其中与本节相关的一条：**能力返回的
`status` 字符串没有任何消费方**（全仓 grep `enrolling` / `locked_no_match` /
`enroll_failed` 只命中 `person_reid.py`、`rgbd_distance.py`、文档和测试；
唯一的生产消费方 `semantic_intent_router/server.py:_follow_distance_rejected`
只看 `accepted`/`ok`/`success`）。所以新增的 `enrolling_no_person` 不影响调用方。

---

### Step 12: 指定跟随 = 全程只跟"注册时看到的第一人" ✅

**需求（用户原话）**：*"要指定跟随，全程必须跟随的是注册是看到的第一人，其他人不允许跟随。"*

改动前有三处会让"别人"顶替目标，本步全部修掉：

| # | 漏洞 | 后果 |
|---|---|---|
| 1 | `_enroll_candidate_index()` 每帧重新选"最大的框"，**没有记忆** | 注册的那几帧里走进第二个人，模板变成两个人的平均 → 两个人都过阈值 |
| 2 | 过阈值后由**位置关联**挑人（`2×IoU − 中心距 + 0.25×置信度`） | 只要陌生人过了阈值，就会因为"站位更像上一帧"而被顶上 |
| 3 | 锁定后 `usable` 为空时**仍走光流桥**（`short-track`） | 光流跟的是"框"不是"人"：陌生人走到目标位置就会被接管 |

#### 12.1 改法

**注册：盯住第一人**（`rgbd_distance._enroll_sample_index()` / `person_reid.add_enroll_feature()`）

- 第一帧出现可注册的人时**定锚**（取最大的框 —— 站得最近的人就是发指令的人），此后锚点不动
- 之后每帧只取**与锚点关联**的那个人的特征（关联规则复用 `_association_rank()`）；画面里只剩别人时
  **跳过该帧且不递减计数**，继续等（配合 Step 11 的 2 秒播报，用户看得见"还在等人"）
- 双保险：模板缓冲非空时，新样本与当前均值的余弦 `< _ENROLL_CONSISTENCY_FLOOR (0.50)` 直接丢弃 ——
  即使锚点被抢走，模板也不会被平均成两个人

**选人：像谁就跟谁**（`rgbd_distance._select_target()` / `person_reid.match_features()`）

- `ReidLockState.match_features()` 返回**每个框与模板的相似度**（`filter_features()` 保留为兼容包装，语义不变）
- 锁定期间**老的关联/选择逻辑整段不跑**：候选 = 相似度 ≥ 阈值的框，其中**相似度最高**的成为目标；
  与上一帧框的位置关联**只用于打破近似平局**
- 锁定期间没有合格候选（只剩陌生人 / 一个人都没有 / OSNet 裁剪无效）→ 不产出目标 →
  `reid-no-match` → 距离控制器给 `vx = 0`（配合 `distance_center_fallback: false`，没有"图像中心"兜底）

**关掉光流桥**（`_select_target()` 尾部）

- `reid_blocked` 的提前返回挪到光流桥**之前**：锁定期间光流桥完全不参与
- 代价：锁定后目标被 HOG 漏检 1~3 帧时不再"续框"，机器人短暂停车。**停比跟错人安全**，
  且丢帧本来就可能是 `distance_center_fallback: false` 的停车路径

#### 12.2 为什么"调阈值"治不了这个病

Step 10 实测：同一人的 5 帧 vs 两人各 5 帧交替注册 —— 后者模板成两人平均，对**两个人**的相似度都是
**0.837**。阈值 0.70 放行两个，调到 0.80 仍放行两个，调到 0.85 连真目标一起拒。
根因是**模板被平均**，不是阈值低 —— 所以本步修的是注册与选人，不是阈值。阈值仍留在 0.70。

#### 12.3 验证（离线；不碰相机、不发速度）

`packages/go2_robottrack/tests/test_person_reid_lock.py` 新增 3 组共 10 个用例（167 → 177，全绿）：

| 用例 | 断言 |
|---|---|
| `EnrollFirstPersonTests.test_a_second_person_is_never_averaged_into_the_template` | 第一人注册途中只剩陌生人 → 不递减、不注册；人回来才锁上，且对第一人 1.00 / 对陌生人 0.00 |
| `EnrollFirstPersonTests.test_a_stranger_alone_after_enrolment_is_blocked` | 锁定后只剩陌生人 → `reid-no-match`，无目标 |
| `IdentityBeatsPositionTests.test_the_best_match_wins_over_the_box_that_continues_the_previous_one` | 陌生人站在目标原位（IoU=1）且 HOG 置信度 0.99，真目标移动后相似度 0.90 vs 0.80 → **仍跟真目标** |
| `IdentityBeatsPositionTests.test_a_box_in_the_targets_place_below_the_threshold_is_not_followed` | 目标原位出现 0.55 的框 → 不跟，停车 |
| `LockedBridgeTests`（2 个） | 同一组帧：无锁 → `hog, short-track, short-track`；有锁 → `reid, reid-no-match, reid-no-match` |

`bash scripts/validate_offline.sh`：**177 + 51 全绿**，manifest 一致性 OK。
`source` 新增取值 `"reid"`：全仓确认它是**纯数据**（`ros_node.py` 只把它透传进 capability 响应），无分支消费。

#### 12.4 仍然存在的窗口（其中第一条已由 Step 13 关闭）

- ~~**注册的那几帧锁还没生效**（`reid_enroll_samples` 帧 ≈ 0.17s），这几帧仍走老的 HOG 选择，
  可能先框住置信度更高的陌生人~~ → **Step 13 已关闭**：未验证出身份前一律不产出目标
- **撞衫**：真目标与陌生人外观极像（两人都 ≥ 阈值）时仍由"谁更像模板"决定 —— 纯视觉的固有边界
- 阈值仍未用真人数据校准（见"尚未做"）

---

### Step 13: vx 只在"画面里有我"时才非 0 ✅

**用户要求（原文）**："我要求只有一个，画面中有'我'，vx才能有非0值，否则任何情况都不能有值。"
原地转无所谓 —— 所以本条只管 `vx`，`wz` 不动。

Step 12 关掉的是"锁定之后跟错人"。Step 13 关掉的是**锁定之前那段没人管的时间**：
从说了"跟着我"到你走进画面之间，锁还没有模板，`_select_target` 走的是老路（HOG 选人 → 光流桥 → 画面中心兜底），
而 Step 11 把"等你"改成了**永不过期**，等于把这段没闸的时间拉成了无限长。

#### 13.1 改法：给"已下达但未验证"这个状态一个名字

`person_reid.py`：

- 新增 `_engaged`：`request_enroll()` 置 `True`，`clear()` 置 `False`，暴露 `is_engaged()`，
  快照新增 `engaged` 字段（带默认值，不影响旧构造）
- 它跨"收样本中 / 等不到人 / 注册失败 / 已锁定"四种状态。**注册失败也不解除**是刻意的：
  否则 `enroll_failed` 会把机器人悄悄退回"见人就跟"，正是这条要堵的洞

`rgbd_distance.py`：

- `_select_target` 新增 `reid_unverified = lock.is_engaged() and not lock.is_active()`
  （**Step 14 已把这条闸从"已下达"扩大到"只要部署了锁"**，下文的两个标志位随之被
  `self._config.reid_enabled` 取代）
- 该状态下**不产出目标**（`source="reid-unverified"`），且与 `reid_blocked` 共用**同一条提前返回**，
  位置在光流桥与画面中心兜底**之前** —— 桥跟的是框不是人，兜底跟的是画面正中间，两者都会跟错人

命令链上的效果：`estimate.valid=False` → `ros_node` 清掉测量（`update_measurement(None, ...)`）
→ `FixedDistanceController` 走 `no_measurement` → **`vx=0`**，`wz` 保持模型值。

#### 13.2 离线证据（已进 `validate_offline.sh` 门禁）

`scripts/_diag_no_identity_no_vx.py`：30Hz，**真**估计器 → **真** `FollowDistanceRuntime` → **真**
`FixedDistanceController`，用 manifest 里的部署参数（target 1.0 / kp 0.50 / deadband 0.10 /
max_forward 0.50 / max_reverse 0.0 / max_age 0.60）。探测器恒报一个"人形框"（就是 HOG 把家具看成人的样子），
深度给 3.0m —— 远超死区，正常情况下必然全速前进：

| 阶段 | 构造 | 期望 vx≠0 的帧 | 实测 |
|---|---|---|---|
| A 无锁 | 没人说过"跟着我" | 全部（老的见人就跟） | 3/3 ✅ |
| B 已下达，画面无人 | 等不到人（`enrolling_no_person`） | **0** | 0/30 ✅ |
| C 已下达，第 1 帧有人形框 | 身份未定 | **0** | 0/31 ✅ |
| D 已锁定，人在画面里 | 唯一允许前进的情况 | 全部 | 3/3 ✅ |
| E 已锁定，人走出画面 | 从人走掉的那帧起 | 只有人在的那帧 | 1/31 ✅ |
| F 注册失败，画面仍有人形框 | `enroll_failed` | **0** | 0/30 ✅ |

`bash scripts/validate_offline.sh`：**183 + 51 全绿** + 本门禁 OK。
`packages/go2_robottrack/tests/test_person_reid_lock.py` 新增 6 个用例（177 → 183）：
`UnverifiedIdentityBlocksVxTests`（5 个，逐条对应上表 B/C/F 与"取消后恢复"、"已锁定照样跟"）
+ `ReidLockStateTests.test_engagement_spans_the_command_and_outlives_a_failed_enrolment`。
新增 `source` 取值 `"reid-unverified"`：与 `"reid"` 同属纯数据，无分支消费方。

#### 13.3 按这条要求当时**未**闭合的两处 → **已由 Step 14 闭合**

1. ~~**相机整帧停摆**：最后一条已验证测量按普通容差保留 0.60 秒~~ → Step 14.2
2. ~~**没人说过"跟着我"**：走老的"见人就跟"，`vx` 只看 HOG~~ → Step 14.1

---

### Step 14: 收口 —— 部署了锁，`vx` 就只认已验证的"我" ✅

承接 Step 13.3 的两处未闭合（用户："先把没改完的问题先改完"）。

统一成一条规则：**`reid_enabled: true` 时，`vx` 非 0 ⟺ 画面里有已验证的注册目标。**
转不受影响（原地转照旧）。

#### 14.1 改动

| 位置 | 改动 |
|---|---|
| `rgbd_distance.py` | 身份要求从"已下达才生效"扩大到**只要部署了锁就生效**：选择分支由"锁定优先，否则走老路"改成"有匹配就是它，没有就没有"。`source` 分四类（均为纯数据）：`reid-inactive`（没人说过"跟着我"）、`reid-unverified`（说了但还没验证出来）、`reid-no-match`（锁定了、人不在）、`reid-unavailable`（要锁但 OSNet 没起来） |
| `core.py` / manifest | 新增 `distance_identity_max_age_s: 0.25` |
| `ros_node.py` | `_expire_distance_measurement` 按开关选窗口；OSNet 加载失败的报错补上后果 |
| `validate_manifest_consistency.py` | 新增一致性检查：有锁时 identity 窗口不得大于普通窗口 |

**相机停摆的尾巴**（13.3 第 1 条）：原来最后一条已验证测量按普通容差
`distance_measurement_max_age_s = 0.60` 存活 —— 相机一停，车还带着 0.30m 的惯性往前。
有锁时改按 `distance_identity_max_age_s` 过期，这个窗口当时定在 0.25s，
理由是"覆盖 OSNet 一次前向（实机 ~60ms）+ 抖动"。

> ⚠️ **这个理由只算了一半**：估计算子是 HOG（中位 185ms）**先跑**、OSNet（~60ms）后跑，
> 而测量的年龄从**帧到达**起算 —— 结果落地时已经 ~0.21s 旧，0.25s 的窗口在用之前就过期了。
> 后果见 **Step 17**：`vx` 恒为 0，车能转不能走。现已改为 0.60s。

#### 14.2 行为变化（两处，都是要的）

- **没人说"跟着我" → 车不前进**（仍可原地转）。这就是"画面里没有我"的字面实现。
- **"别跟了" → 停车**，不再回到"见人就跟"。想回到旧跟随，唯一开关是 `reid_enabled: false`
  （那条路一个字节没动，仍有独立用例守着）。

#### 14.3 离线证据（门禁：9 个阶段）

`scripts/_diag_no_identity_no_vx.py` 重写：**参数直接读 `robonix_manifest.yaml`**（不再在脚本里抄一份），
真估计器 → 真 `FollowDistanceRuntime` → 真 `FixedDistanceController`，相机 30Hz、下发 50Hz、
OSNet 推理延迟 60ms 显式建模：

| 阶段 | 构造 | vx 允许非 0 | 实测（vx≠0 的下发次数 / 61） |
|---|---|---|---|
| A | 锁未部署（`reid_enabled: false`） | 全部 | 58（老回路原样） |
| B | 部署了锁，没人说过"跟着我" | **0** | 0 ✅ |
| C | 说了，画面里没人 | **0** | 0 ✅ |
| D | 说了，第 1 帧有人形框 | **0** | 0 ✅ |
| E | 已锁定，人在画面里 | 全部 | 51 ✅ |
| F | 已锁定，人走出画面 | 只在窗口内 | 14，最后一帧 #23（0.2s 画面 + 0.25s 窗口）✅ |
| G | 注册失败，画面仍有人形框 | **0** | 0 ✅ |
| H | 已锁定，相机整个停摆 | 只在窗口内 | 46，最后一帧 #55 ✅ |
| I | 部署了锁但 OSNet 没起来 | **0** | 0 ✅ |

#### 14.4 验证

`bash scripts/validate_offline.sh`：**187 + 51 全绿** + 9 阶段门禁 OK + manifest 一致性 OK（52 键）。
用例 183 → 187：改写 2 个语义已变的旧用例（`test_unlocking_does_not_reopen_plain_association`、
`test_cancelling_the_command_stops_the_follow_loop`），新增 4 个
（`test_nobody_has_asked_yet_produces_no_target`、`test_a_lock_with_no_model_behind_it_produces_no_target`、
`test_without_reid_the_plain_loop_is_byte_for_byte_unchanged`、`test_identity_measurement_window_is_bounded`）。

> 注意：本地若开着 HTTP 代理（`http_proxy`），`semantic_intent_router` 的 HTTP 自测会拿到 502
> 而不是 404（`urllib` 把 127.0.0.1 也送去代理了）。跑门禁前 `no_proxy=127.0.0.1,localhost`。

---

### Step 15: 视觉后端切到本机 TensorRT 10.7（DINOv3 + SigLIP）✅

起因：用户问"为什么走 torch 而不是 TensorRT"，并指出 10.7 已经装过。

#### 15.1 先核对事实（这里有一次我自己的误判，一并更正）

| 事实 | 证据 |
|---|---|
| 系统 TensorRT 是 **10.3.0.30**（JetPack 6.2.2 / L4T R36.5） | `dpkg`: `libnvinfer10 10.3.0.30-1+cuda12.5`；`trtexec` → `TensorRT v100300`；容器内 `import tensorrt` → 10.3.0 |
| 10.3 建不出 DINO engine | `logs/build-dino.log`（2026-09-01）：`[E] IIfConditionalOutputLayer inputs must have the same shape. Shapes are [2] and [1]`（node 81 `[If]`，`/m/rope_embeddings/If_output_0`）→ `Parsing model failed` → `&&&& FAILED`。SigLIP 无此节点，同一次构建 **PASSED** |
| 10.7 **确实装在容器里**，且不覆盖系统 10.3 | `/root/tensorrt10.7/TensorRT-10.7.0.23/`（`lib/libnvinfer.so.10.7.0`、`bin/trtexec`、`python/*cp310*.whl`）。装在独立目录 + 靠 `LD_LIBRARY_PATH` 区分，回滚＝去掉环境变量 |
| 两份 engine 就是**本机用 10.7 建的** | 用 10.7 的 `trtexec --loadEngine --skipInference`：DINO 45 MiB、SigLIP 795 MiB，**双双 `&&&& PASSED [TensorRT v100700] [b23]`**；与安装记录 §9.3 的 46M/796M 对得上 |
| 真正缺的是 10.7 的 **python 绑定** | 绑定当初装进 `/root/minicpm-venv`，该 venv（连同另一个）因 `libcudnn.so.9` 缺失被删；镜像里现已无任何 venv → `--dino_backend trt_direct` 无法 `import tensorrt` |

> ⚠️ 更正：排查中我先用**系统的 10.3** `trtexec` 去加载这两份 engine，报
> `Serialization assertion safeVersionRead == kSAFE_SERIALIZATION_VERSION failed`
> （`Serialized Engine Version: 239`），据此推断"engine 是别处构建的、违反文档 §56"。
> **这个结论是错的** —— engine 与 TRT 版本严格绑定，用 10.7 的库读就是好的。
> 启动器里 `--dino_backend torch --siglip_backend torch` 是从 Go2 参考
> （`go2-reference/scripts/start_robottrack_inference_server.sh`，其文档也写"使用
> DINO/SigLIP torch backend"）**逐字沿用**的，不是我后来换掉的；缺绑定才是一直没切过来的原因。

#### 15.2 改动

| 位置 | 改动 |
|---|---|
| `scripts/start_robottrack_inference_server.sh` | 默认后端改 `trt_direct`；从镜像内 10.7 树解包 cp310 wheel 到 `${TMPDIR}/robottrack-trt107-cp310` 并用 `PYTHONPATH` 引入（容器每次从镜像重建，无残留）；`LD_LIBRARY_PATH` 前置 10.7 `lib/`（同名 `libnvinfer.so.10`，必须前置才拿到 10.7）；SigLIP token 数显式 **576**（本机 engine 是 pooled 24×24，server 默认值是 729）；缺 10.7 树时明确报错并提示回退 |

#### 15.3 基准（同进程背靠背，384×384，bf16，batch=1，15 次取中位数）

`scripts/_bench_vision_backends.py`：torch 塔用 server 自己的 `VisionFeatureCacher`，
TRT 塔按 `_maybe_install_*_trt` 的方式换成 `DinoTrtDirect` / `SigLipTrtDirect`。

| 塔 | torch | TRT 10.7 | 加速 | 输出 token 余弦 |
|---|---|---|---|---|
| DINOv3 ViT-S/16 | 33.6 ms | **16.1 ms** | ×2.09 | 0.99610 |
| SigLIP SO400M | 88.7 ms | **70.9 ms** | ×1.25 | 0.97042 |
| 合计 | 122.3 ms | **87.0 ms** | ×1.41 | — |

两边 token 形状一致（`(1,576,384)` / `(1,576,1152)`）。顺带把 SigLIP 的 fp32 权重
从显存里换掉（`del cacher.siglip`）。

#### 15.4 端到端（`/eval_dual`，合成帧，每轮换图避免命中特征缓存）

`scripts/_probe_vision_backend.sh trt|torch`：真启动器 → 真 server → POST 合成 JPEG。

| 后端 | 服务器横幅 | `/eval_dual` | 稳态 e2e（第 2/3 轮） |
|---|---|---|---|
| trt（默认） | `[dino-trt] loaded engine=…dino_patch_target_fp16.engine in=(1,3,384,384) out=(1,576,384)` + `DINO backend = TRT FP16`；SigLIP 同款，`out=(1,576,1152)` | 3/3 HTTP 200 | 223.7 / 237.4 ms |
| torch（回退） | 无任何 TRT 横幅（`vision_amp_dtype=torch.bfloat16` 即 torch 塔） | 3/3 HTTP 200 | 237.9 ms |

第 1 轮不计（建 engine/预热，`server_route_total_ms` 2949.7）。

**注意** 端到端只差 ~10 ms，因为 `/eval_dual` 的稳态延迟被 policy 前向（torch 塔 ~150 ms、
TRT 塔 ~207–222 ms）主导，视觉那 35 ms 不是整车节拍的决定项；要看的收益是 15.3 那张表。
两份 engine 都打印 `[TRT] [W] Using an engine plan file across different models of devices
is not recommended` —— 这是 Jetson 上同型号不同实例的常规告警，忽略。

#### 15.5 回退（不用改文件）

容器内（或 `follow.sh shell` 进去之后）：

```bash
ROBOTTRACK_VISION_BACKEND=torch bash scripts/start_follow.sh      # 两个塔都回 torch
ROBOTTRACK_DINO_BACKEND=torch bash scripts/start_follow.sh        # 只回一个
```

宿主机侧（`scripts/follow.sh start` 会把这三个变量显式 `-e` 转进容器）：

```bash
ROBOTTRACK_VISION_BACKEND=torch bash scripts/follow.sh start
```

或者干脆写进 `.env`（`start_follow.sh` 在容器内 `source` 它），这是持久做法。

⚠️ **不能**只在宿主机 `export` 之后跑 `follow.sh start`：`docker exec` 不继承宿主机
环境（见踩坑记录），变量会以空值到达容器，回退静默失效、栈照旧起在 TensorRT 上。
`follow.sh` 已经把这层转发补上了，但只有从它走才生效 —— 直接 `docker exec` 手敲
命令的场合仍然要靠 `-e` 或 `.env`。

原文件备份：`scripts/start_robottrack_inference_server.sh.bak-20260930-230617`。
镜像未改动、系统 10.3 未改动、venv 未重建。

#### 15.6 未做 / 边界

- 只用**合成帧**验证：SigLIP token 余弦 0.970、DINO 0.996，真人对跟随行为的影响**没量过**。
- 实机 live 未跑（人不在场）。切换默认值前建议先用真人帧比一次 `base_velocity`。
- engine 与「本机 + TRT 10.7 + MAXN」绑定：换镜像或升 JetPack 要按部署文档 §56 重建。

#### 15.7 起栈 / 关栈脚本的同步

后端换了之后，起栈脚本原来"没看见"的两件事变成了必须看见的：

| 脚本 | 同步内容 | 为什么 |
|---|---|---|
| `scripts/start_follow.sh` | 预检：`VISION_BACKEND != torch` 时先确认两份 engine 和容器内 10.7 树都在，缺就直接拒绝并给出回退命令 | `DinoTrtDirect` 是**第一帧**才构造的（server 的 `_ensure_vision_cache` 懒加载），缺 engine 会表现成"第一帧 500"而不是起栈失败 —— 等人站到车边上才发现 |
| `scripts/start_follow.sh` | 起完推理服务后等最多 8 s，只抓"启动器当场退出"这一类失败（TRT 树缺失、wheel 解包失败），死了就打印日志尾部并拒绝起栈 | 原来只 `&` 一下就走，启动器 exit 2 之后栈照样引导，跟随桥一条计划都拿不到，dry-run 下没有任何提示 |
| `scripts/start_follow.sh` | 启动横幅加 `vision=${VISION_BACKEND}` | 和 `mode=${INTENT_MODE}` 同理：把"这次实际选了什么"写进日志 |
| `scripts/follow.sh start` | 起栈预算 75 s → **85 s**（提示语同步改成"约 80 秒"）；三个后端开关用 `-e` 转进容器 | 多出来的 8 s 就是上面那次等待；预算不跟着改，健康的一次起栈会显示成 `OK 条数` 偏低，和真故障分不出来。`-e` 转发是因为 `docker exec` **不继承宿主机环境**，不转发则宿主机侧的回退命令会静默失效 |
| `scripts/follow.sh status` / `check` | 新增"视觉后端"一栏，读的是 **server 自己打的横幅**（`[server] DINO backend = TRT FP16`），不是启动器的 `backends=` | 后者只说明请求了什么；横幅才是"真的装上了"。0 个横幅 = torch 或还没热身，1 个 = 混着跑 |
| `.env.example` | 补 `ROBOTTRACK_VISION_BACKEND` / `ROBOTTRACK_DINO_BACKEND` / `ROBOTTRACK_SIGLIP_BACKEND` 三个开关 | 回退开关属于机器本地配置，不该藏在脚本注释里 |
| `README.md` | status/check 两行说明 + sidecar 段落补后端与回退 | — |

**没有改的**：`scripts/shutdown-follow.sh`。它按 `$RUN_DIR/inference.pid` 关服务
（pid 仍是 `exec` 后的 python 进程，写法没变），第 4 步的 PID 命名空间清扫也照样
覆盖推理服务；解包目录在容器的 `/tmp`，`docker restart` 会保留并复用、重建容器会
跟着消失，两种情况都不需要清理。

**注意（15.4 的补充）**：因为引擎是懒加载的，`/health` 绿 ≠ 引擎可用；`follow.sh check`
里"模型已加载"和"视觉后端"是两条独立结论，后者要跑过一帧才作数。

---

### Step 16: 时钟 / NTP —— 语音播报签名过期（`AuthFailure.SignatureExpire`）✅

#### 16.1 现象

网络问题解决后，播报改成报这个：

```
speaker failed: Tencent TTS AuthFailure.SignatureExpire: Signature expired.
Please use local time and enable NTP service for time synchronization to avoid
a deviation between `Timestamp` and the server time 1791533770
```

**不是密钥过期**（appid / SecretId / SecretKey 一个字没动，也没有任何 `SecretIdNotFound`
或 `SignatureFailure`）。腾讯对这个码的定义是固定的：请求里的 `Timestamp` 与服务端相差
**不得超过 5 分钟**。所以这是一条**时钟**报错，不是账号报错。

#### 16.2 根因（实测）

本机时钟比真实时间慢 **8 天 15 小时**：

| 来源 | 时间（UTC） |
|---|---|
| 本机 | 2026-09-30 16:59 |
| 阿里云 / 百度 / 腾讯（HTTP `Date` 头） | 2026-10-09 08:12 ~ 08:17 |
| NTP 三源（aliyun / cn.pool / ubuntu，手发 NTP 包） | 2026-10-09 08:17:37 |

差 746228 秒，所以每一次签名都必然被拒。**没有可靠时间源**：

- `hwclock -r` = `1970-01-01` —— RTC 电池没电/没有，**关机即丢时间**；
- `timedatectl`：`System clock synchronized: no`；`/etc/systemd/timesyncd.conf` 里
  `[Time]` 段是空的，于是用默认的 **`0.fr.pool.ntp.org`（法国）**，
  `Packet count: 0` —— 从来没同步成功过。

**NTP 网络本身是通的**：从本机手发 NTP 请求，`ntp.aliyun.com` / `cn.pool.ntp.org` /
`ntp.ubuntu.com` 全部应答真实时间。坏的只是**配置里选的那个服务器**。

#### 16.3 改动

只改宿主一个文件，原文件备份为 `/etc/systemd/timesyncd.conf.bak-20261001-010238`
（**文件名里的日期是旧钟时间**，别拿它当备份时间看）：

```ini
[Time]
NTP=ntp.aliyun.com cn.pool.ntp.org ntp.tencent.com
FallbackNTP=ntp.ubuntu.com time.google.com
```

`systemctl restart systemd-timesyncd` 后它自己 step 到位：
`Offset: +1w 1d 15h 18min` → `System clock synchronized: yes`，
Server `203.107.6.88 (ntp.aliyun.com)`。**不需要手动 `date -s`**。

#### 16.4 顺序（重要）

必须**先停栈再对时**：系统时间前跳 8 天会让正在跑的 ROS 时间戳/TF 直接炸
（TF 外推失败、消息过滤器失效）。实际执行：

```
follow.sh stop  →  改 NTP + 重启 timesyncd  →  follow.sh start
```

导航栈全程没碰（对时前它就是 `Exited (255) 7 days ago`，`follow.sh stop` 里那句
"导航容器不在了"的 WARN 是它自己本来就停着，不是本次动作造成的）。

#### 16.5 验证

| 项 | 结果 |
|---|---|
| 时钟 | 本机 2026-10-09 08:21 UTC；`System clock synchronized: yes` |
| TTS 合成 | `Tencent TTS request` → `TTS cache prewarmed for 2-character prompt` → `Backend status: … tts=Tencent Cloud TTS (OK)`；`SignatureExpire` **0** 条、`TTS synthesize failed` **0** 条 |
| 起栈 | 16/16 OK |
| 四检 | 4/4 OK（TRT 双塔 / 意图路由在线 / 相机 29~30 Hz / `/cmd_vel` 发布者 1） |

#### 16.6 边界

- **RTC 仍然是坏的**：断电久了仍会从一个旧时间起步（这次就是），只是 timesyncd 现在
  能在开机后很快纠正。要根治得换 RTC 电池。
- 时间跳变会在 docker 的 uptime 上留痕（`Up 8 days` —— 容器的 `StartedAt` 还是旧钟时间），
  纯显示问题，不影响功能。
- **本文档里所有取自旧钟的日志时间戳都不可信**：它们整体**早了 8 天 15 小时**
  （写在文档里的"2026-09-30"实际约为 10-09）。做时间线分析时要换算。

---

### Step 17: 身份窗口 0.25s → 0.60s —— 定距延迟大于窗口，`vx` 恒为 0 ✅

**症状**：说了"跟着我"、人也站在画面里，车**只原地打转、不前进**（重说一次也一样）。

#### 17.1 定位

| 观测 | 结果 |
|---|---|
| `/ranger/robottrack/cmd_vel_raw` 连续 8s / 371 帧 | `vx` **全 0**，`wz` 全非 0（夹在 ±0.300） |
| MCP `distance {operation: get}` | `status=no_measurement`、`has_measurement=false`，而 `enabled/active` 都是 true |
| 注册本身 | **没问题**。离线拿真 HOG + 真 OSNet + manifest 参数跑：12 帧内 `locked`，之后每帧 `source=reid`、余弦 0.96~0.99（阈值 0.70） |
| 深度配对 | **没问题**。color / depth 的 header 时间戳差 0.000s（阈值 0.20s） |
| HOG 检出 | **没问题**。20/20 帧都有人形框 |

链路本身是通的：`_select_target` 能选到人、`estimate()` 能出 `valid=True status=ok dist=1.41m`
（目标 1.0m ⇒ 期望 `vx≈0.2`）。所以问题不在"认没认出来"，而在**测量还没被用就已经过期**。

#### 17.2 根因：窗口是按 OSNet 一个环节定的，而估计算子是两段

容器内用**真 HOG + 真 OSNet + manifest 参数**分段实测：

| 环节 | 中位 | 最大 |
|---|---|---|
| HOG 检测（`detectMultiScale`，384~512 宽） | **185 ms** | 344 ms |
| OSNet 特征 | 60 ms | — |
| **整条 `estimate()`** | **210 ms** | **419 ms** |

关键在**年龄从哪算**：`estimate()` 的 `frame_timestamp_s` 用的是 `pair.rgb.received_monotonic`
（**帧到达**的时刻），不是结果落地的时刻。所以一条测量算出来的时候**已经 ~0.21s 旧**。

而 `ros_node._dispatch()` 每 20ms 的顺序是**先过期、后使用**：

```
_expire_distance_measurement(now)   # 超窗 → _measurement = None
select_distance_dispatch(...)       # 拿到 None
```

0.25s 的窗口碰上 0.21s（中位）/ 0.42s（最大）的延迟 ⇒ 绝大多数测量在 `_expire` 那一步就被清掉 ⇒
`_measurement` 恒为 `None` ⇒ `distance_control.apply()` 走 `no_measurement` 分支，返回
`vx=0, wz=模型 wz` —— **转向原样透传，前进被掐掉**。这与实测的 0 次非零 `vx` 完全吻合。

**为什么"以前能动"**：加锁（Step 14）之前上限是 `distance_measurement_max_age_s = 0.60`，够覆盖
这两段延迟。0.25s 是随锁一起引入的，而它只按 OSNet 的 ~60ms 估——HOG 在它前面，贵 3 倍。

#### 17.3 改法

| 位置 | 改动 |
|---|---|
| `robonix_manifest.yaml` | `distance_identity_max_age_s: 0.25` → **`0.60`**；上方注释改写为实测延迟与失效机理 |

- **0.60 就是上限**：`validate_manifest_consistency.py` 有约束「有锁时 identity 窗口不得大于普通窗口」，
  而普通窗口 `distance_measurement_max_age_s` 本来就是 0.60。想再放大就必须连着放大普通窗口，
  那会把 13.3 拿身份窗口治好的"相机停摆后带惯性前冲"重新放回来 —— 不划算，没必要。
- **包默认值不动**：`core.py` 里 `distance_identity_max_age_s` 的默认值仍是 0.25（它只在 manifest
  没写这个键时生效）。`go2_robottrack` 是与上游共享的代码，改默认值等于制造分叉；这里改的是**部署**的值。

#### 17.4 验证

| 项 | 结果 |
|---|---|
| `validate_manifest_consistency.py` | 52 个配置键全认；`specified-person lock ON … (identity window 0.6s)` |
| `validate_offline.sh` | 全绿（含 identity gate A–I 九阶段） |
| 起栈 / 四检 | **待重启后填**（见 17.5） |

> 跑 `validate_offline.sh` 时本机的 `http_proxy=127.0.0.1:7897` 会把 localhost 也代理了，
> intent router 的 4 个用例假报 502。用 `env -u http_proxy -u https_proxy` 跑就全过 —— 与本次改动无关。

#### 17.5 重启复验

**待执行**：`bash scripts/follow.sh stop` → `start`（只动 `robonix_follow_dev`）。
期望：起栈 **16/16 OK**、`check` **4/4 OK**；导航容器当时本就没在跑，不受影响。

#### 17.6 边界

- **估计算子仍是 ~5Hz**：worker 是单线程、只保留最新一帧（`submit()` 不撤销在跑的测量），
  一帧 210ms ⇒ 30Hz 相机里每 6 帧丢 5 帧。放宽窗口**不改变**这一点，只是让算出来的那 5Hz 真被用上。
- **相机停摆的惯性**：上限从 0.25s 回到 0.60s，即回到 Step 14 之前的数值（当时是刻意的取舍）。
  真要更紧，得先降 HOG 延迟（换检测器 / 降分辨率），而不是继续勒窗口。
- **日志判不了注册成败**：代码只在注册**停滞**时 `logger.warning`（每 2s），成功和失败都不打。
  想知道锁状态就查 MCP `distance {operation: get}` 的 `status`，或跑离线探针，别去翻日志。

---

### Step 18: 人一离开就"乱开" —— 收紧假匹配的两条通道 ✅

**症状**（Step 17 修好之后才出现，因为之前 `vx` 恒为 0 根本走不了）：跟随正常，但**人离开后有时停、有时满速乱开**。

#### 18.1 为什么"前进"只可能来自匹配

排查顺序与结论：

| 假设 | 结论 |
|---|---|
| mux 会退给 Nav2 开 | **否**。`source_mux.output()` 过期时返回 `ZERO_TWIST`（`selected_source_stale`），不切源 |
| 模型的 `vx` 漏过来 | **否**。模型 `vx` 一直是丢弃的，`vx` 只来自定距控制器 |
| 定距控制器自己会动 | **否**。无测量时返回 `VelocityCommand(vx=0.0, wz=模型wz)` —— 只转不走 |

所以**非零 `vx` ⟺ 存在测量 ⟺ ReID 判定"匹配上了"**。人不在场还满速前进，只可能是**假匹配**，而且匹配到的通常是**远处的背景块**（距离误差大 ⇒ `vx` 直接顶到 `distance_max_forward_mps`）。

#### 18.2 三个代码层面的口子

| # | 位置 | 问题 |
|---|---|---|
| 1 | `_select_target`（`rgbd_distance.py`） | **判定是单帧的**。`locked_target = max(locked_matches, key=locked_rank)` —— 一帧相似度过阈值就成了目标，下一帧不合格就丢。`detection_confirmation_frames: 2` 只作用在**不装锁**的老路径上 |
| 2 | `reid_min_bbox_height_px` | **只在注册时生效**（`_enroll_sample_index`）。匹配时 `extract()` 对**任何**非空裁剪都出特征（只有完全出界才置零），20px 的墙角照样被拉成 256×128 送 OSNet |
| 3 | `OpenCvHogPersonDetector` | `hit_threshold=0.0` ⇒ HOG 接受**每一个** SVM margin > 0 的窗口；又因 confidence = `sigmoid(margin)`，**所有返回框的置信度都 > 0.5**。而 ReID 路径下 HOG 置信度**完全不参与判定**（`_association_rank` 只用 IoU + 中心位移），所以 0.01 分的垃圾框和 0.99 分的真人框平权 |

本轮**只修 (3) + 收紧相似度阈值**；(1)(2) 是逻辑层，见 18.5。

#### 18.3 改动

| 位置 | 改动 |
|---|---|
| `rgbd_distance.py` | `RgbdDistanceConfig` 新增 `detector_min_confidence`（默认 `0.0` = 不过滤，**行为与改动前完全一致**）；在 `_select_target` 构造 `usable` 的**唯一漏斗**处过滤 |
| `core.py` | `RuntimeConfig` 新增 `detector_min_confidence`（边界 0..1）；`distance_identity_max_age_s` 的注释改为实测口径 |
| `ros_node.py` | 透传给估计器 |
| `robonix_manifest.yaml` | 新增 `detector_min_confidence: 0.60`；`reid_match_threshold: 0.70 → 0.85`（随后回调至 **0.80**，见 18.6） |
| `config/go2_robottrack.yaml` / `README.md` | 补新键 |
| `scripts/probe_person_detection.py` | 报告改为打印**全部**框的置信度分布（原来只打选中那个），并读 manifest 的 floor —— 定 `detector_min_confidence` 就靠它 |
| `tests/test_person_reid_lock.py` | +8 用例（`DetectorConfidenceFloorTests` / `DetectorFloorConfigTests`），共 195 |

**为什么把 floor 放在 `usable` 而不是检测器里**：`usable` 是注册与匹配**共用**的候选漏斗，放这里两个入口一起管住；而且**将来换掉 HOG 也不会把这个门槛一起换掉**（这正是 Step 18 讨论过的"要不要上 YOLO11"的前置条件）。

**为什么 `reid_match_threshold` 定 0.85**：本机实测**真匹配是 0.96~0.99**。0.70 留了 0.26 的带宽给噪声往上爬；0.85 距实测真匹配下限仍有 ~0.11 余量。**（该值随后被回调到 0.80，理由见 18.6。）**

**为什么 `detector_min_confidence` 先定 0.60**：它和 `hit_threshold=0.0` 是同一把尺子（confidence = sigmoid(margin)，margin 0.405 ↔ confidence 0.60），所以 0.60 砍掉的正是"勉强判正"的那一段——垃圾框的产地。**这是保守的第一步**，因为真人检测的 confidence 分布**尚未实测**（见 18.5），定高了会直接"找不到人"。阶梯：0.60 → 0.70 → 0.80。

#### 18.4 验证

| 项 | 结果 |
|---|---|
| `validate_manifest_consistency.py` | 53 个键全认（新增 1 个） |
| `validate_offline.sh` | 全绿：go2_robottrack **195** 用例、意图路由 51 用例、identity gate A–I 九阶段 |
| 默认值不回归 | `detector_min_confidence` 默认 `0.0`，用例 `test_floor_zero_keeps_every_detection` 显式守住"不过滤 = 老行为" |
| 实机 | **未验证**，见 18.5 |

#### 18.5 还没做的 / 已知边界

- **两个值都还没有实机依据**。`detector_min_confidence=0.60` 是**推断**不是实测：真人框的 confidence 分布没量过。上机后**先跑** `scripts/probe_person_detection.py`（打印全部框的置信度），看**真人**落在哪一档——真人贴着 floor 就往下调，垃圾框远高于 floor 就往上调。
- **(1) 单帧判定没修**。这是"人一离开"之外更根本的假匹配通道：**路人长得像、或者只露出半个身子**，一帧就能顶掉目标。修法是加连续帧确认（比如连续 N 帧过阈值才认），属于匹配层逻辑，与检测器无关。
- **(2) 匹配时的最小框高限制没修**。`reid_min_bbox_height_px` 仍然只管注册。匹配时加上这个条件可以顺带砍掉大量小垃圾框。**注意它会把"人走远到框 <64px"也一起拒掉 —— 车会停**，所以和 18.3 的 floor 一样，得实测后再定。
- **`reid_match_threshold` 的风险**：如果注册的人**背对镜头、走进阴影、或大半出画**，相似度可能掉到阈值以下 ⇒ 锁被丢 ⇒ 车停。**先怀疑这个值，再怀疑别的**；回退就是一个数字。（该值已从 0.85 回调到 0.80，见 18.6。）
- **Step 17 的窗口与本次改动的相互作用**：identity 窗口从 0.25 放宽到 0.60 是"车能走"的前提，但它同时把**每次假匹配的滑行时间放大了 2.4 倍**。本轮收紧 floor + 阈值就是在补这个副作用。**真正干净的解法是让估计算子变快**（HOG 中位 185ms 是主因），那时窗口才能收回去。

#### 18.6 阈值回调：`reid_match_threshold` 0.85 → 0.80

**改了什么**：只动 manifest 里的一个数，`reid_match_threshold: 0.85` → `0.80`。`core.py` / `rgbd_distance.py` 的**包默认值仍是 0.70，不动**（那里只是代码缺省，部署值始终由 manifest 决定）。

**为什么回调**：0.85 是我在第 18.3 步根据"实测真匹配 0.96~0.99"**单方面收紧**的，收紧方向有代价——它同时把"人稍微侧身 / 光线稍暗 / 半个身子出画"这些**真匹配掉分**的情况也一起推到了阈值以下，代价是**锁被丢、车停**。0.80 距实测真匹配下限（0.96）仍有 **~0.16** 余量，比 0.85 的 ~0.11 更宽，对真人掉分更宽容；同时仍把 0.70 那条 0.26 的噪声爬升带砍掉大半。

| 值 | 距真匹配下限（0.96） | 噪声带宽 | 取向 |
|---|---|---|---|
| 0.70 | 0.26 | 0.26 | 太松：垃圾框容易爬进来 |
| **0.80** | **0.16** | **0.16** | **当前值**：两边都留了余量 |
| 0.85 | 0.11 | 0.11 | 偏紧：真人掉分就被丢锁 ⇒ 车停 |

**验证**：manifest 一致性 + 离线全量用例重跑（值本身不参与用例断言，改的是部署参数，不影响任何测试结论）。

**这一步其实有实测旁证（Step 12.2）**：更早的离线实验里，用"两个人各 5 帧交替注册"污染模板后，同一人的相似度掉到 **0.837**——**0.80 放行，0.85 拒绝**。也就是说 0.85 **在实测中已经拒掉过一次真实的同一人**（虽然是降级模板）。0.80 恰好卡在这个实测值下方，这是它比 0.85 更站得住的地方。

**仍未验证**：和 18.5 一样，0.80 在**干净模板 + 真机**下的表现**没有实机依据**——"注册的人侧身多少度会掉到 0.80 以下"没量过。上机后若出现**车找不到人（停）**，先回调这个数；若**还是乱开**，说明问题在 `detector_min_confidence`（弱框）而不在这里。

---

### Step 19: 每次回答都多一句"这句话不匹配跟随距离指令；未调用任何能力" ✅

#### 19.1 现象

不管任务**成功还是失败**，客户的最终回答末尾都会多出这一句。它**不来自** Pilot 或 Executor，而是语义路由 sidecar 说的话。

#### 19.2 定位过程

`pilot.log` 是决定性证据。每一次语音指令都留下**两轮**规划，第二轮永远是同一句：

```
[pilot/rtdl] task_update goal='设置跟随距离为 2 米' status='in_progress'
[pilot/rtdl] -- plan start plan_id=3 round=0 calls=1 --      ← 真调用
[pilot/rtdl]   [1] do op_id=5 cap=go2_robottrack.follow_distance args={"meters":2.0,...}
[pilot/forest] plan_id=3 complete state=Succeeded; replanning
[pilot/rtdl] -- plan start plan_id=4 round=1 calls=0 --      ← 多出来的这一轮
[pilot/rtdl] rtdl_plan_description='follow distance intent not recognized'   ← 就是那句
```

`follow distance intent not recognized` 正是 `server.py` 里那条兜底分支的 `description`。

#### 19.3 根因：Pilot 注入的"范围行"被当成了用户指令

Pilot 一轮计划跑完后会 `replanning`（再问一次模型"做完了吗/还要做什么"），这一问会把结果灌回历史。**问题在于它灌的是好几条 user 角色消息**，而路由 sidecar 是靠"找最后一条 user 消息"来认指令的（`current_turn`）：

| Pilot 注入的 user 消息（`system/pilot/src/planner.rs`） | 前缀 |
|---|---|
| 叶子结果 | `Executor feedback for the current RTDL leaf (not a new user request): ` |
| **每棵树结果前的范围行** | **`Executor feedback scope: `** ← 元凶 |
| 图像观察备注（`history.rs`） | `Executor feedback: the previous capability call returned this image.` |
| 计划控制纠正 | `Pilot harness feedback: ` |

sidecar 的 `_is_feedback()` **只认第一条前缀**。于是范围行（Pilot 把它 push 在叶子结果**之前**，位于用户原话**之后**）成了"最后一条 user 消息"⇒ 被当成指令 ⇒ 两个意图解析器都不认 ⇒ 走兜底 ⇒ 回那句"这句话不匹配跟随距离指令"。

注意这解释了为什么**成功和失败都出现**：它不是结果判断出了问题，而是**第二轮规划**这个问题本身必然发生，与第一轮成败无关。

离线复现（构造 Pilot 的真实消息顺序，喂给 `decide(..., execution_mode="live")`）：

```
[system, user"跟着我。", assistant, user"Executor feedback scope: ...", user"Executor feedback for the current RTDL leaf..."]
  → 修复前: 这句话不匹配跟随距离指令；未调用任何能力     ← 与线上日志逐字一致
  → 修复后: 锁定当前看到的人已受理：collecting 5 samples
```

#### 19.4 改法

只改 `packages/semantic_intent_router/semantic_intent_router/server.py`，**只动"认指令"这一步，不动"收结果"**：

| 改动 | 内容 |
|---|---|
| 新增 `_HARNESS_LABELS = ("Executor feedback", "Pilot harness feedback")` | 按**标签**匹配而不是整句，Pilot 改后半句也不会失效 |
| 新增 `_is_harness_message()` | 任意 Pilot 注入的 user 消息（`_is_feedback` 的超集） |
| `current_turn()` 找指令时改用 `_is_harness_message` | 范围行/纠错行不再能冒充指令 |
| `_is_feedback()` **保持原样** | 它仍然只认那条带 `leaf_result` 的叶子反馈，`leaf_results()` 的切片长度因此不受影响 |

**为什么拆成两个判定**：如果直接把 `_is_feedback` 放宽，范围行会一并进 `turn.feedback_messages`，被 `leaf_results()` 按错误的偏移量切片——那是拿一个 bug 换另一个 bug。拆开之后：**找指令**看宽集合，**收结果**看窄集合。

**安全性没有变化**：反馈存在时 `_follow_distance_decision` / `_person_lock_decision` 一律走终态分支、**不下发新叶子**，所以"相对指令被重放导致又加一米"这条老防线仍然成立（有用例 `test_harness_line_never_triggers_a_second_capability_call` 守着）。

#### 19.5 验证

| 项 | 结果 |
|---|---|
| `semantic_intent_router` 离线用例 | **58 全过**（原 51，+7：范围行/纠错行/图像行不算指令、纯注入历史无指令、真话不会被误判为注入、成功与失败两端的终态回答、注入行不触发第二次调用） |
| 现场消息顺序复现 | 修复前复现线上那句坏话；修复后同一输入得到 `已受理` / `已完成` |
| `validate_offline.sh` | 全绿（路由 58 + go2_robottrack 195 + identity gate A–I + manifest 53 键） |

#### 19.6 生效条件与边界

- **要重启栈才生效**：sidecar 是常驻进程，代码虽经 `/mnt/ssd/wxf/jiaofu/skill-follow-person-rbnx -> /root/deploy` 绑定挂载进容器，但已加载的模块不会热更。
- **本步只改了"回答里多一句"**，**没有**改任何决策、阈值或速度通路。
- 若 Pilot 未来新增**别的前缀**的注入行，同一个病会以同样的方式复发——判据是"最后一条 user 消息是否是用户说的"。

---

### Step 20: 底盘断电后 CAN 帧"检测不到" —— 判据看的是网卡，不是控制器 ✅

**现象**：每次关掉 Ranger 再开机，`setup-hw.sh` 都报
`[WARN] can_ranger 上没有底盘帧 —— 跳过控制模式断言`，
而接口自己是 `UP / LOWER_UP / carrier=1 / bitrate 500000`，看上去完全正常。

**实机观测（2026-10-15）**：

| 观测 | 值 | 说明 |
|---|---|---|
| `ip -details link show can_ranger` | `can state ERROR-PASSIVE` | 同一行里网卡仍是 `state UP` |
| `error-pass 769389` / `rx_packets 772524` | 同速上涨 ~2091/s | 驱动只在 `can_id & CAN_ERR_FLAG` 时调 `gs_update_state()` ⇒ **涨的是错误帧**，不是底盘数据 |
| 裸 socket（空 `CAN_RAW_FILTER` + 全 1 `CAN_RAW_ERR_FILTER`） | 4 s 收到 0 帧 | 不是 candump 参数语义的问题（candump 默认收不到错误帧） |
| 手工 `configure_can`（down→bitrate→up）之后 | 控制器回 `ERROR-ACTIVE`，`rx_packets 772524 → 772524` | 重置有效，但**线上确实一帧都没有** |
| `/proc/net/softnet_stat` | `dropped=0`，`processed` 同速上涨 | 不是内核 backlog 丢包 |
| `dmesg` 本次开机 | `Configuring for 1 interfaces` 出现 **5 次** | 适配器被 USB 重新枚举过 4 次 |

每次重新枚举都是同一串：RX URB 刷错（`gs_usb_receive_bulk_callback: N callbacks suppressed`，
即修补版驱动那条 `receive URB failed: %pe, resubmitting`）→ `Couldn't shutdown device (err=-19)`
（ENODEV，设备已经没了）→ 重新枚举 → `renamed from can2`。
**底盘断电的瞬态顺着 CAN 线把适配器从 USB 上打掉了**；它回来时控制器留在 ERROR-PASSIVE，
而名字与波特率随后被脚本配好 ⇒ 表面上永远健康。

**根因（脚本侧）**：`can_up_at_bitrate()` 只看网卡：

```bash
can_up_at_bitrate() { ip -details link show "$1" | grep -q "state UP" && grep -qw "bitrate 500000"; }
```

`state UP` 是**网卡**的标志，和 CAN 控制器能不能收帧无关。于是

- 快捷分支（原 267 行）判定 "already UP"，把整个探测块（含 `can_frames`）全部跳过；
- 唯一会重置链路的 TX 探针（原 347 行）只在 `cansend` 报 ENOBUFS（gs_usb echo-slot 泄漏）时才触发。

两条合起来：**被断电留在 ERROR-PASSIVE 的控制器，脚本一次都不会修**。

**改法**（两处，"静默总线不是致命错误"这个既有判断不动）：

1. 新增 `can_ctrl_state()` / `can_ctrl_ok()`（读 `ip -details` 的 `can state`），
   在快捷分支和 configure 的触发条件里各加一个 `can_ctrl_ok`：
   控制器不健康 ⇒ 当成"没配好" ⇒ 走 `configure_can` 重置，然后**照样做流量探测**
   （这一块以前是被跳过的）。空字符串（接口 DOWN / 不是 CAN 口）算不健康 —— fail closed。
2. §2b 的"0 帧"分支先看控制器状态：**只在不是 ERROR-ACTIVE 时**重置一次再重试。
   有意不做无条件 reset：① 它会打断正在跑的 `ranger_base_node` 的 socket（与 TX 探针同一 caveat）；
   ② **重置变不出帧** —— 本机实测：控制器健康的静默总线，重置后 `rx_packets` 依然不动。
   所以"健康但静默 = 查底盘侧"这条结论不变。

**验证**（把 `setup-hw.sh` 226-424 行原文抽出来单独跑，不碰雷达/CDI 段；
`cansend` 打桩，避免为了测试真往底盘发 0x421 切控制模式）：

| 场景 | 结果 |
|---|---|
| 真机现况（ERROR-ACTIVE + 静默） | `[ OK ] can_ranger already UP at 500000bps (controller ERROR-ACTIVE)`；§2b 打印"控制器 ERROR-ACTIVE = 它听得见总线…底盘确实没在发"；RC=0 |
| 先把链路置 DOWN | 快捷分支不再命中 → `gs_usb owns can_ranger` → `configure_can` → `[ OK ] … (controller ERROR-ACTIVE)`；跑完链路恢复 UP/ERROR-ACTIVE |
| `can_ctrl_ok` 打桩为"不健康"（改动副本） | 打印 `[WARN] can_ranger UP@500000 但控制器是 … —— 重新初始化`；§2b 打印 `没有底盘帧，且控制器是 … —— 重置链路后重试`；RC=0，无未绑定变量/语法错误 |

"重置后收到底盘帧"那条分支（`[ OK ] 重置链路后收到底盘帧 —— 断的是适配器侧`）需要底盘真的在发帧，
本机没有这个条件，**未实测**。

**结论**：这台机器上"检测不到"的直接原因是**底盘侧没在发帧**（控制器重置成 ERROR-ACTIVE 后
`rx_packets` 4 秒一帧不涨）—— 要查上电 / 急停 / CAN 线两端。脚本这一侧补的是：
**下次适配器再被断电打掉，它会自己重置，而不是只看一眼网卡就放过。**

---

### Step 21: 再加一级"摘挂 USB" —— 软复位治不了的那种适配器 ✅

**起因**：Step 20 之后还剩一种情况。`configure_can`（down→bitrate→up）只重启 CAN **控制器**，
换不掉 USB 会话本身（同一个固件实例、同一批 URB、同样的 echo-slot 泄漏）。
"每次重设的时候把那个 USB-CAN 小盒子重启一下"——对，这活 `configure_can` 干不了。

**新增**（`setup-hw.sh`）：

- `usb_dev_of_can()`：从 `/sys/class/net/<if>/device` 反推 USB 设备路径（本机实测 `can_ranger` → `1-4.1`）
- `usb_resync_can()`：`unbind` → 1 s → `bind`（软件等价于拔插一次），再等 netdev 回来。
  回来时内核给的是**新名字**（本机 on-board 的 `can0/can1` 占着槽位 ⇒ 通常回来是 `can2`），
  所以它会找回 `can_ranger` 这个名字并重配链路。**每次运行最多一次**（`CAN_USB_RESYNCED`），
  且只在控制器不健康时触发 —— 它会打断正在跑的 `ranger_base_node` 的 socket，
  而且**它变不出帧**（Step 20 已实测：复位后 `rx_packets` 依然不动）。它是兜底，不是解药。
- 阶梯：§2 控制器不健康 → `configure_can`；**仍**不健康 → `usb_resync_can`。
  §2b「0 帧 + 不健康」→ 软复位；**仍**不健康 → 同一级（被守卫拦住，不重复摘挂）。
- `can_rx_rate()`（1 秒 `rx_packets` 增量）：控制器不是 `ERROR-ACTIVE` 时打印
  "适配器这 N 秒收到了 X 帧、candump 解不出一条 ⇒ 全是错误帧"，把**"没有有效帧"和"线上真安静"分开**。
  §2 那句"适配器 / USB / 驱动都正常"也改成不把话说满（当时那个结论是错的）。

**补充观测（2026-10-15 15:40）**：同一台机器上，一会儿是 `ERROR-PASSIVE` + `rx_packets` ~2090/s
（全是错误帧），一会儿底盘又正常发帧 ⇒ 这个"检测不到"是**间歇**的。
Step 20 里"直接原因是底盘侧没在发帧"只在那一刻成立。

**验证（2026-10-15，离线抽 §2 段落跑；`cansend` 打桩，USB 摘挂路径重定向到临时目录）**：

| 场景 | 结果 |
|---|---|
| 真机现状（当日下午再跑） | 底盘帧**有**了：`0x211 byte1=01` ⇒ 底盘在发，且已在 CAN 控制模式；`usb_dev_of_can can_ranger` → `1-4.1` |
| 强制「控制器 ERROR-PASSIVE + 0 帧 + rx 2090/s」 | 两级都走到：`[OK] 摘挂回来：can_ranger 已重新出现` → `[OK] 摘挂后控制器 …` → §2b 第二次触发被守卫拦住（**一次性生效**）→ 打印三条错误帧判据 |
| **真摘挂**（真写 `/sys/bus/usb/drivers/usb/unbind`） | **未实测** —— 离线验证把这两个路径换成了临时文件，真适配器没动过 |

**验证里的坑（记下来别再踩）**：离线抽段落跑是**非 root**，`ip link set …` 全报
`RTNETLINK answers: Operation not permitted` ⇒ `configure_can` 返回 1 ⇒ 打出
`[FAIL] can_ranger did not come UP`。那是权限造成的假象，不是脚本缺陷（正式跑时脚本已 `exec sudo`）。
**离线验证只能验控制流与消息，验不了真实链路操作。**

**另一条实测**：`${can_ctrl_now:-无状态}` 一度被我写成 `$(…)`（把默认值当命令执行，
报 `can_ctrl_now:-无状态: 未找到命令`）—— 就是靠离线跑一遍才发现的。

---

## 修改记录

| 时间 | 文件 | 改动 | 说明 |
|---|---|---|---|
| 2026-09-29 | `docs/specified-person-follow.md` | 新增 | 本文档 |
| 2026-09-29 | `go2_robottrack/person_reid.py` | 新增 | OSNet 封装 + 锁状态机 |
| 2026-09-29 | `go2_robottrack/rgbd_distance.py` | 修改 | `_select_target` 插 ReID 门 |
| 2026-09-29 | `go2_robottrack/core.py` | 修改 | RuntimeConfig 新增 5 字段 |
| 2026-09-29 | `go2_robottrack/ros_node.py` | 修改 | 加载 OSNet + 传锁状态 |
| 2026-09-29 | `go2_robottrack/provider.py` | 修改 | 加 lock/unlock operation |
| 2026-09-29 | `tests/test_person_reid_lock.py` | 新增 | 23 个用例 |
| 2026-09-29 | `semantic_intent_router/person_lock.py` | 新增 | 语音意图解析 |
| 2026-09-29 | `semantic_intent_router/server.py` | 修改 | 接 lock/unlock 路由 |
| 2026-09-29 | `semantic_intent_router/tests/test_person_lock.py` | 新增 | 13 个用例 |
| 2026-09-29 | `go2_robottrack/tests/test_provider_contract.py` | 修改 | +4 个 lock/unlock 契约用例 |
| 2026-09-29 | `go2_robottrack/start.sh` | 修改 | `ROBONIX_TORCH_LIBRARY_PATH` → `LD_LIBRARY_PATH` |
| 2026-09-29 | `robonix_manifest.yaml` | 修改 | env + 5 个 reid 配置项 |
| 2026-09-29 | `go2_robottrack/CAPABILITY.md` | 修改 | 声明 lock/unlock |
| 2026-09-29 | `go2_robottrack/capabilities/.../FollowDistance.srv` | 修改 | 仅注释，类型未变 |
| 2026-09-29 | `robonix_manifest.yaml` | 修改 | 经 `set_follow_mode.sh dry-run` 切到 dry-run（原 live，已自动备份） |
| 2026-09-29 | `robonix_manifest.reid-test.yaml` | 新增 | 临时最小 profile（相机+robottrack），雷达坏了时的测试脚手架 |
| 2026-09-30 | `go2_robottrack/person_reid.py` | 修改 | 新增 `note_enroll_wait()` + `waiting_s`/`prompts`；注册改为不过期、每 2s 播报 |
| 2026-09-30 | `go2_robottrack/rgbd_distance.py` | 修改 | 门拆出 `enroll_stalled` 分支，接 `note_enroll_wait()` |
| 2026-09-30 | `go2_robottrack/tests/test_person_reid_lock.py` | 修改 | +7 用例（`EnrollWaitPromptTests`），共 167 |
| 2026-09-30 | `scripts/_diag_enroll_throttle.py` | 新增 | 等待/终止路径的节流离线诊断（常备） |
| 2026-09-30 | `docs/specified-person-follow.md` | 修改 | 新增 Step 11 |
| 2026-09-30 | `go2_robottrack/rgbd_distance.py` | 修改 | 注册盯第一人（`_enroll_sample_index` + 锚点）；锁定期间按相似度选人；`reid_blocked` 提前于光流桥 |
| 2026-09-30 | `go2_robottrack/rgbd_distance.py` | 修改 | 抽出 `_association_rank()`，供关联与注册锚点共用 |
| 2026-09-30 | `go2_robottrack/person_reid.py` | 修改 | 新增 `match_features()`；`add_enroll_feature()` 返回 bool + 一致性地板 0.50；`note_enroll_wait(reason=)` |
| 2026-09-30 | `go2_robottrack/tests/test_person_reid_lock.py` | 修改 | +10 用例（`EnrollFirstPersonTests` / `IdentityBeatsPositionTests` / `LockedBridgeTests`），共 177 |
| 2026-09-30 | `docs/specified-person-follow.md` | 修改 | 新增 Step 12 |
| 2026-09-30 | `go2_robottrack/person_reid.py` | 修改 | 新增 `_engaged` / `is_engaged()` / 快照 `engaged`；注册失败不解除 |
| 2026-09-30 | `go2_robottrack/rgbd_distance.py` | 修改 | 新增 `reid_unverified` 门：已下达未验证 → 不产出目标，`source="reid-unverified"` |
| 2026-09-30 | `go2_robottrack/tests/test_person_reid_lock.py` | 修改 | +6 用例（`UnverifiedIdentityBlocksVxTests` 等），共 183 |
| 2026-09-30 | `scripts/_diag_no_identity_no_vx.py` | 新增 | A–F 六阶段实测 vx（估计器→runtime→控制器） |
| 2026-09-30 | `scripts/validate_offline.sh` | 修改 | 新增 identity gate 门禁 |
| 2026-09-30 | `docs/specified-person-follow.md` | 修改 | 新增 Step 13 |
| 2026-09-30 | `go2_robottrack/rgbd_distance.py` | 修改 | 身份要求扩大到"只要部署了锁"；`reid-inactive` / `reid-unavailable` 两个原因 |
| 2026-09-30 | `go2_robottrack/core.py` | 修改 | 新增 `distance_identity_max_age_s`（默认 0.25，边界 0.05..2.0） |
| 2026-09-30 | `go2_robottrack/ros_node.py` | 修改 | 测量过期按 identity 窗口；OSNet 失败报错补后果 |
| 2026-09-30 | `robonix_manifest.yaml` | 修改 | +`distance_identity_max_age_s: 0.25`；ReID 注释改写为新规则 |
| 2026-09-30 | `go2_robottrack/config/go2_robottrack.yaml` / `README.md` | 修改 | 补新键说明 |
| 2026-09-30 | `go2_robottrack/tests/test_person_reid_lock.py` | 修改 | 改写 2 个语义已变用例，+3 用例，共 186 |
| 2026-09-30 | `go2_robottrack/tests/test_core.py` | 修改 | +1 用例（identity 窗口边界），共 187 |
| 2026-09-30 | `scripts/_diag_no_identity_no_vx.py` | 修改 | 改为读 manifest 参数 + 建模推理延迟；扩到 9 阶段 |
| 2026-09-30 | `scripts/validate_manifest_consistency.py` | 修改 | +identity 窗口一致性检查与说明行 |
| 2026-09-30 | `docs/specified-person-follow.md` | 修改 | 新增 Step 14 |
| 2026-09-30 | `scripts/start_robottrack_inference_server.sh` | 修改 | 视觉后端默认切 TRT 10.7（`trt_direct`）；回退开关 `ROBOTTRACK_VISION_BACKEND=torch`；原文件备份 `.bak-20260930-230617` |
| 2026-09-30 | `scripts/_probe_vision_backend.sh` | 新增 | 后端 A/B 端到端探针（起真启动器→POST 合成帧→比对；不碰硬件） |
| 2026-09-30 | `scripts/_bench_vision_backends.py` | 新增 | 两塔 torch/TRT 同进程基准 + 输出 token 余弦 |
| 2026-09-30 | `docs/specified-person-follow.md` | 修改 | 新增 Step 15 |
| 2026-09-30 | `scripts/start_follow.sh` | 修改 | 起栈前校验 engine + 10.7 树（缺则拒绝并给回退命令）；起完推理服务等最多 8 s 抓当场退出；横幅加 `vision=` |
| 2026-09-30 | `scripts/follow.sh` | 修改 | `status` / `check` 增加"视觉后端"栏（读 server 横幅）；起栈预算 75→85 s；三个后端开关 `-e` 转进容器；起栈失败提示改为兼顾推理服务启动失败 |
| 2026-09-30 | `.env.example` | 修改 | 补 `ROBOTTRACK_VISION_BACKEND` 等三个后端开关说明 |
| 2026-09-30 | `README.md` | 修改 | status/check 说明行 + sidecar 段落补后端与回退 |
| 2026-10-09 | （宿主）`/etc/systemd/timesyncd.conf` | 修改 | NTP 源从默认的 `0.fr.pool.ntp.org` 改为 `ntp.aliyun.com cn.pool.ntp.org ntp.tencent.com`（+Fallback）；原文件备份为同目录 `timesyncd.conf.bak-<时间戳>` |
| 2026-10-09 | `docs/specified-person-follow.md` | 修改 | 新增 Step 16（时钟 / NTP 与 TTS 签名过期） |
| 2026-10-09 | `robonix_manifest.yaml` | 修改 | `distance_identity_max_age_s` **0.25 → 0.60**；上方注释改写为实测延迟（HOG 185ms + OSNet 60ms）与"过期早于使用"的失效机理 |
| 2026-10-09 | `docs/specified-person-follow.md` | 修改 | 新增 Step 17；并改正 Step 14 里"0.25s 覆盖 OSNet ~60ms"这条只算了一半的理由 |
| 2026-10-09 | `go2_robottrack/rgbd_distance.py` | 修改 | 新增 `detector_min_confidence`（默认 0.0）+ 在 `_select_target` 的 `usable` 漏斗处过滤；`__post_init__` 加 [0,1] 校验 |
| 2026-10-09 | `go2_robottrack/core.py` | 修改 | `RuntimeConfig` 新增 `detector_min_confidence`；`distance_identity_max_age_s` 默认值不变但注释改为实测口径（HOG 185ms + OSNet 60ms） |
| 2026-10-09 | `go2_robottrack/ros_node.py` | 修改 | 透传 `detector_min_confidence` |
| 2026-10-09 | `robonix_manifest.yaml` | 修改 | 新增 `detector_min_confidence: 0.60`；`reid_match_threshold` **0.70 → 0.85** |
| 2026-10-09 | `go2_robottrack/config/go2_robottrack.yaml` / `README.md` | 修改 | 补 `detector_min_confidence` 说明 |
| 2026-10-09 | `scripts/probe_person_detection.py` | 修改 | 报告从"只打选中框的置信度"改为**打全部框的分布** + 读 manifest 的 floor（定门槛的依据） |
| 2026-10-09 | `go2_robottrack/tests/test_person_reid_lock.py` | 修改 | +8 用例（`DetectorConfidenceFloorTests` / `DetectorFloorConfigTests`），共 195 |
| 2026-10-09 | `docs/specified-person-follow.md` | 修改 | 新增 Step 18（假匹配的两条通道：HOG 弱框 + 阈值） |
| 2026-10-10 | `robonix_manifest.yaml` | 修改 | `reid_match_threshold` **0.85 → 0.80**（回调：放宽对真人掉分的容忍，旁证见 Step 12.2 实测 0.837） |
| 2026-10-10 | `docs/specified-person-follow.md` | 修改 | 新增 Step 18.6（阈值回调 0.85 → 0.80 的原因与旁证）；18.3 / 18.5 交叉引用 |
| 2026-10-10 | `semantic_intent_router/server.py` | 修改 | 新增 `_HARNESS_LABELS` + `_is_harness_message()`；`current_turn()` 找指令改用后者（`_is_feedback` 保持不变，`leaf_results` 不受影响）—— 修掉"每次回答多一句'这句话不匹配跟随距离指令'" |
| 2026-10-10 | `semantic_intent_router/tests/test_server.py` | 修改 | +7 用例（`PilotHarnessTurnTests`），路由共 58 |
| 2026-10-10 | `docs/specified-person-follow.md` | 修改 | 新增 Step 19（Pilot 注入的 user 消息被当成指令） |
| 2026-10-10 | `semantic_intent_router/README.md` | 修改 | 新增"Not every user-role message is the user"一节（四类注入前缀与各自用途） |
| 2026-10-13 | （宿主）docker 镜像 | 新增 | `robonix:humble-orin-follow` **252982a64e66 → 6bb7ba364760**（commit `robonix_follow_dev`，可写层 87.9MB）；旧镜像保留为 `.bak-20261013` |
| 2026-10-13 | （宿主）docker 镜像 | 新增 | `robonix:humble-orin-deps-slim` **636160a0e541 → dbe4676eb7f3**（commit `robonix_ros_dev`，可写层 729MB）；旧镜像保留为 `.bak-20261013`。注意容器仍在跑**旧**镜像，`docker commit` 不改运行中的容器 |
| 2026-10-13 | （宿主）镜像离线包 | 新增 | `/mnt/ssd/backups/docker-images/robonix-humble-orin-follow+deps-slim-20261013.tar` —— 一条 `docker save` 同时导出上述两个镜像，**22,816,996,352 B（21.2 GiB）**，`manifest.json` 双条目；共用的 18.3G 底座 `31812ff4e227` 只存一份（tar 内共 5 层；分开存约 40.9 GiB）。`sha256=46be7952fb2a89d4582deafc731c1c505fa4980476d6e08b4c9b223cc7e184f2`。离线恢复：`docker load -i <tar>`（一次导入两个镜像；**容器本身不在包里**，需按原 `docker run` 参数重建）。**已实测**：tar 内 14 个 blob 的内容 sha256 与其文件名逐一相符（0 处不符）；`docker load` 实跑通过，两个 tag 都回来；5 层 diffID 与本机镜像 `RootFS.Layers` 完全一致，配置 blob = `6bb7ba364760` / `dbe4676eb7f3`，故本机 load 复用已有层、`/mnt/ssd` 占用不变 |

| 2026-10-15 | `setup-hw.sh` | 修改 | 新增 `can_ctrl_state()` / `can_ctrl_ok()`（读 `ip -details` 的 `can state`）。`can_up_at_bitrate()` 只反映网卡，`ERROR-PASSIVE` 的控制器照样算"已就绪"；快捷分支与 configure 触发条件各加一个控制器健康判据，接口 DOWN / 非 CAN 口算不健康（fail closed） |
| 2026-10-15 | `setup-hw.sh` | 修改 | §2b 的"0 帧"分支改为：控制器不是 `ERROR-ACTIVE` 时先 `configure_can` 重置一次再重试，然后才下"底盘侧"的结论；消息里带上控制器状态 + `rx_packets` 复核命令。理由见 Step 20 |
| 2026-10-15 | `docs/specified-person-follow.md` | 修改 | 新增 Step 20（底盘断电后 CAN 帧检测不到：判据看的是网卡，不是控制器） |
| 2026-10-15 | `setup-hw.sh` | 修改 | 新增 `usb_dev_of_can()` / `usb_resync_can()`（USB 摘挂 ≈ 拔插一次）与 `can_rx_rate()`；§2 与 §2b 各加一级"仍不健康就摘挂"的阶梯，`CAN_USB_RESYNCED` 保证每次运行最多摘挂一次。**真摘挂未实测**，见 Step 21 |
| 2026-10-15 | `setup-hw.sh` | 修改 | §2 的"适配器 / USB / 驱动都正常"改为不把话说满；§2b 在控制器非 `ERROR-ACTIVE` 时打印 1 秒 `rx_packets` 增量，区分"没有有效帧"与"线上真安静" |
| 2026-10-15 | `docs/specified-person-follow.md` | 修改 | 新增 Step 21（摘挂 USB 这一级：为什么软复位不够、验证怎么做、哪一项没实测） |

**改动统计**：新增 5 个文件（`person_reid.py`、`person_lock.py`、2 个测试文件、本文档），修改 10 个原有文件。

**核心结论**：`FollowDistance.srv` 本来就用 `string operation`，所以 `lock` / `unlock` **不需要新合约、新 IDL、新 codegen，也不需要重建 ROS2 overlay**。

---

## 依赖

- OSNet 模型：`models/osnet/osnet.py` + `osnet_x1_0_market1501_softmax.pth`（已下载，已验证）
  - 容器内 strict load：0 missing / 0 unexpected keys
  - 512 维特征，FP16 on Orin 33.15 ms/帧
- CUDA 环境：由 `ROBONIX_TORCH_LIBRARY_PATH` 提供，见 Step 6
- 视觉后端 TensorRT（Step 15）：容器内 `/root/tensorrt10.7/TensorRT-10.7.0.23`（lib + cp310 wheel + trtexec）
  + 两份本机建的 engine `upstream/…/realworld/trt_artifacts/{dino_patch_target_fp16,siglip_pooled_target_maxn_fp16}.engine`。
  系统 TRT 仍是 10.3（未动），靠 `LD_LIBRARY_PATH` 区分。
- 时间源（Step 16）：宿主 `/etc/systemd/timesyncd.conf` 指向 `ntp.aliyun.com` / `cn.pool.ntp.org`。
  本机 RTC 无电池（`hwclock` = 1970），**时间完全依赖 NTP**；腾讯 ASR/TTS 签名要求
  本机时间与服务端相差 <5 分钟，时钟一偏，语音整条链就静默失效。

---

## 踩坑记录（汇总）

| 问题 | 原因 | 解法 |
|---|---|---|
| `TypeError: only length-1 arrays...` | `global_avgpool` 输出 `(N,512,1,1)` | `pooled.flatten(1)` |
| `ImportError: libcudnn.so.9 not found` | 镜像 `LD_LIBRARY_PATH` 为空 | 指到 pip 装的 cuDNN 目录 |
| `sudo` 3 次密码错误 | heredoc 抢了管道 stdin | 脚本写文件后 `docker cp` 进去 |
| router 测试 502 Bad Gateway | `http_proxy` 把 localhost 也代理了 | 跑测试时清空 proxy 变量 |
| "别跟了" 匹配不到 | 模式要求 `跟着/跟随` | 放宽到 `跟`，且先判 unlock |
| "别跟着我了" 被判成锁定 | unlock 没优先判 | unlock 先测 |
| `_leaves()` 找不到 leaf | RTDL leaf 用的是 `cap`/`args` 不是 `capability_name` | 改走 `envelope["rtdl"]["children"][0]["args"]` |
| 等待播报连珠炮（间隔 0.42/0.63/0.27s） | `add_enroll_feature()` 复位了节流窗口，HOG 一闪就重开 | 只在 `request_enroll()`/`clear()` 复位，间隔是硬上限 |
| 宿主 `trtexec` 读本机 engine：`Serialized Engine Version: 239` 不匹配 | engine 与 TensorRT 版本严格绑定；宿主是 10.3，engine 是 10.7 建的，**不是**"别处构建的坏文件" | 用建它的那套 10.7 `trtexec --loadEngine` 读 |
| 用 TRT 10.3 建 DINO engine 失败 | 10.3 的 ONNX importer 拒收 DINOv3 rope 里的 `If` 节点（`IIfConditionalOutputLayer inputs must have the same shape. Shapes are [2] and [1]`） | 用容器内的 10.7 建（`/root/tensorrt10.7`） |
| 引擎/绑定都在，`--dino_backend trt_direct` 还是起不来 | 10.7 的 **python 绑定**装在 `/root/minicpm-venv`，该 venv 因 `libcudnn.so.9` 缺失被删 | 从镜像里的 wheel 解包到 `/tmp` 走 `PYTHONPATH`（见 15.2） |
| TRT 塔跑起来但特征网格不对 | 本机 SigLIP engine 是 pooled 24×24 = 576 token，server 默认 `--siglip_trt_output_tokens=729` | 启动器显式传 576 |
| `cannot assign 'DinoTrtDirect' as child module 'dino'` | cacher 是 `nn.Module`，不能把子模块属性重绑成普通对象 | 先 `del cacher.dino` 再赋值（server 的 `_maybe_install_*_trt` 就是这么做的） |
| `/health` 绿，但视觉引擎其实没起来 | 视觉缓存是第一帧才建的（`_ensure_vision_cache` 懒加载），`/health` 只反映模型 | 判后端要看 server 的横幅；起栈脚本改在第一帧之前就把 engine 存在性查掉 |
| 宿主机 `export ROBOTTRACK_VISION_BACKEND=torch` 后跑 `follow.sh start`，栈照样起在 TRT | `docker exec` 不继承宿主机环境，变量以**空值**到达容器（实测：`[\${VAR}]` → `[]`），`${VAR:-trt_direct}` 的空值兜底又把它当未设置 | `follow.sh start` 用 `-e` 显式转发三个后端开关；或写进 `.env`（容器内会 source） |
| TTS 报 `AuthFailure.SignatureExpire … server time 1791533770`，以为密钥过期 | **本机时钟慢 8 天 15 小时**：RTC 是 `1970`（没电池，关机丢时间），`timesyncd.conf` 的 `[Time]` 是空的 → 用默认法国源 `0.fr.pool.ntp.org`，`Packet count: 0` 从未同步。腾讯签名要求 `Timestamp` 与服务端差 <5 分钟 | 换 NTP 源为 `ntp.aliyun.com cn.pool.ntp.org ntp.tencent.com` + 重启 timesyncd。**NTP 网是通的**（手发 NTP 包三源都应答），坏的只是配置里那台服务器。**必须先停栈再对时**，前跳 8 天会让 ROS 时间戳/TF 直接炸 |
| 搞不清"以前通、现在不通"的边界 | 本机时钟不可信时，**日志时间戳也一起错**（整体早 8 天 15 小时），跨天比对会得出错误结论 | 先核时钟（`date -u` 对 HTTP `Date` 头 / NTP），再谈时间线 |
| 说了"跟着我"、人在画面里，车**能转不能走**，`vx` 恒 0 | 测量年龄从**帧到达**起算，而估计算子是 HOG（~185ms）**先跑** + OSNet（~60ms）；`distance_identity_max_age_s` 只按 OSNet 的 60ms 定为 0.25s ⇒ 结果落地时已过期，`_dispatch()` 又"先过期后使用" ⇒ `_measurement` 恒 `None` | 窗口必须覆盖**整条 `estimate()`**，不是只覆盖 OSNet。改为 0.60s（= 加锁前数值，也是校验器允许的上限） |
| 想知道"注册到底成没成"，翻日志翻不出来 | 代码只在注册**停滞**时 `logger.warning`，成功/失败都不打日志；且包内没有 `basicConfig`，路由本身也不确定 | 查 MCP `distance {operation: get}` 的 `status`（`no_measurement` / `locked` / `enroll_failed` …），或跑 `scripts/_diag_no_identity_no_vx.py` |
| 人离开后有时停、有时**满速乱开** | `vx` 非零 ⟺ 有测量 ⟺ 匹配上了。而判定是**单帧**的，且 HOG（`hit_threshold=0.0`）返回的每个框 confidence 都 >0.5、匹配时又**不看** HOG 置信度 —— 一个远处背景垃圾块对上模板，测出 4~5m，`vx` 直接顶满 | 加 `detector_min_confidence` 砍弱框 + 抬 `reid_match_threshold`（见 Step 18）。**根因其实是单帧判定**，那两条只是把噪声压下去 |
| `sigmoid(SVM margin)` 看着像概率，其实卡不动 | HOG 的 `hit_threshold=0.0` ⇒ 只有 margin>0 的窗口才返回 ⇒ **confidence 恒 >0.5**，看着有分数其实没有区分度 | 要卡就得卡 HOG 自己的 `hitThreshold`，或者在候选漏斗上按 confidence 过滤（本仓库的做法） |
| "定距匹配的阈值"和"注册的门槛"混着调 | 两者是**不同**的门：`reid_match_threshold` 管匹配，`reid_min_bbox_height_px` **只管注册**（匹配时小框照样被 OSNet 打分） | 调之前先确认改的是哪一道，见 Step 18 的表格 |
| 每次回答都多一句"这句话不匹配跟随距离指令；未调用任何能力"，**成功失败都有** | 路由 sidecar 靠"最后一条 user 消息"认指令，而 Pilot 把结果灌回历史时会 push **多条 user 角色消息**，其中 `Executor feedback scope: ...` 这条（push 在叶子结果**之前**）前缀不在 sidecar 的白名单里，于是被当成用户指令 ⇒ 解析失败 ⇒ 兜底话术 | 认指令时按**标签**排除所有 Pilot 注入行（`_HARNESS_LABELS`），但**收结果**仍只认叶子反馈前缀，见 Step 19 |
| 以为"多一句废话"是 Pilot/Executor 说的 | 它来自**语义路由 sidecar**（Pilot 的 VLM upstream）；判据在 `pilot.log` 的 `rtdl_plan_description` 字段，那里写着 sidecar 给的 `description` | 先看 `rbnx-boot/logs/pilot.log` 的 round=1 那一轮，再看 `semantic_intent_router` 里对应的兜底分支 |
| 后台 `docker save` 报 `permission denied ... /var/run/docker.sock`，**任务却报退出码 0** | 两件事叠加：① `PATH=/tmp/sudoshim:$PATH docker …` **不提权** —— 那个 shim 只是一个名叫 `sudo` 的脚本，只有命令字面是 `sudo` 时才生效，直接跑 `docker` 仍是以本人身份连 socket；② 命令末尾挂了 `; echo exit=$?; ls`，`ls` 成功了 ⇒ 后台任务报 0，把 docker 的失败盖掉 | 提权一律用字面 `sudo`：`(echo <pw> \| sudo -S docker …) > x.tar`；**不要**在末尾拼无关命令掩盖退出码 |
| 半截的 `docker save` 产物看不出是坏的 | 写到一半被强杀（如机器重启）只留下一个尺寸不对的 tar，文件名和好备份一样 | 先写 `x.tar.partial`，成功才 `mv` 成正式名；判完整性用**字节数**（本次 22,816,996,352 B）+ `tar -xOf … manifest.json` |
| 底盘断电后 `setup-hw.sh` 一直报"没有底盘帧"，而接口看着完全正常（`UP`/`LOWER_UP`/`carrier=1`/`bitrate 500000`） | 判据 `can_up_at_bitrate()` 只看网卡 `state UP` + bitrate，**不看 CAN 控制器状态**。断电把适配器从 USB 上打掉（dmesg：5 次重新枚举 + RX URB 报错 + `Couldn't shutdown device (err=-19)`），回来时控制器留在 `ERROR-PASSIVE`，而名字/波特率已被脚本配好 ⇒ "already UP" 快捷分支把探测整块跳过，唯一的重置路径只认 ENOBUFS | 加 `can_ctrl_ok()` 参与判定（Step 20）。判断"线上到底有没有帧"看 `rx_packets` 3 秒涨不涨，**别信 candump**（默认收不到错误帧，会把满屏错误帧显示成"没有帧"） |
| 把"没有有效帧"当成"总线上什么都没有" | `candump` 默认**收不到错误帧**，而错误帧照样计入 `rx_packets`。`can state ERROR-PASSIVE` + `rx_packets` ~2000/s ⇒ 线上是**满屏错误帧**，不是安静 | 判"有没有帧"看 `rx_packets` 增量（脚本里是 `can_rx_rate()`），**不要**拿 candump 的"0 帧"当结论；错误帧 ⇒ 物理层（线 / 接头 / 终端电阻 / 对面节点状态） |
| 适配器"半死"时，`ip link set down/up` 重置不回来 | 软复位只重启 CAN **控制器**，USB 会话（固件实例 / URB / echo-slot 泄漏）原封不动 | `usb_resync_can()`：`unbind` → 1 s → `bind`，等价于拔插一次（Step 21）。注意回来时网卡名会变（`can2`），且会打断正在跑的底盘节点的 socket |

---

## 已知边界 / 未做

- **重启即忘**：ROS runtime 停掉时 `_stop_runtime()` 会 `clear()` 锁并置 `available=False`。
  重启后必须重新说一次"跟着我"。（有意为之：跨光照/姿态复用一个旧模板更容易跟错人）
- **纯视觉，无避障**：沿用原跟随栈的安全边界，不接雷达/Nav2 避障，也不看地图。
  但**"目标还在不在场"现在有闸了**（Step 13/14）：没有已验证的注册目标就没有 `vx`。
  雷达坏掉不影响这一段（它本来就不读雷达）。
- **解锁后重新捕获有延迟**：原来的关联逻辑需要等上次 bbox 过期（`association_memory_frames`）。
  已验证这是**改动前就有的行为**，不是本次引入的回归。
- **锁定期间丢帧即停车**（Step 12）：锁定后光流桥被关掉，目标被 HOG 漏检时不再"续框"。
  这是为了不把目标交给"走到同一位置的人"，代价是短暂的停走。
- **vx 的闸门 = "画面里有已注册的我"**（Step 13 + 14）：只要部署了锁，从"没人说过跟着我"、注册、
  等待、注册失败、锁定后丢人，到相机停摆，一律不产出目标 → `vx=0`，`wz` 不管。
  代价是**没人说"跟着我"时车不前进**（可原地转），**"别跟了"等于停车**；
  要旧的"见人就跟"就 `reid_enabled: false`。
- **注册不过期，改为循环播报**（Step 11）：画面里没人时说"跟着我"，锁保持
  `enrolling_no_person`，每 2 秒播报一次"仍在等待"，直到有人进入画面才完成注册。
  **不再有超时失败** —— 这是要的行为，不是缺陷。
- **"画面里完全没人"本房间测不到**（Step 11.5）：相机前始终有东西被检测到，实机只覆盖了
  "抖动/半可见"，"完全无人"由 `scripts/_diag_enroll_throttle.py` 的 `empty` 路径离线覆盖。
- **全量栈起不来与环境无关**：宿主 `eno1` DOWN → Mid360 不可达 → `mid360_lidar` 失败
  → rbnx 原子回滚。修雷达后才能跑全量 profile。
- **尚未做**：真人端到端（真人注册 + 陌生人出现的防切换实测）；真机 live 联调。

---

## 备注

- 方案 A（进程内）：`person_reid.py` 被 `rgbd_distance.py` 直接 import，无 HTTP 开销
- 阈值 0.7 可调（`reid_match_threshold`）：误认相似穿着的人 → 调高；目标被丢 → 调低。
  **只在模板是单一身份时才有意义**（Step 12 之前模板可能被平均成两个人，调阈值治不了）
- 线程模型：估计器跑在专有 latest-only 工作线程，provider 在另一线程 → 共享锁状态全部 `RLock` 保护
- 锁定前 OSNet **零开销**：不锁就不跑前向，加载也只在启动时做一次
