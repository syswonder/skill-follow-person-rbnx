# 跟随功能 · 用户手册

**skill-follow-person-rbnx** — AgileX Ranger Mini v3 人员跟随

---

## 〇、在哪里输入命令

**全部命令都在同一个地方敲：机器人主机的一个终端窗口。**

判断方法：看提示符是不是这样

```
wangxiaofeng@ubuntu:/mnt/ssd/wxf/jiaofu/skill-follow-person-rbnx$
                  ^^^^^^ 机器人主机名        ^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 正确的目录
```

只要你在**这台机器**上、并且已经 `cd` 进了 `skill-follow-person-rbnx` 目录，就对了。

### 每次都先做这两下

```bash
cd /mnt/ssd/wxf/jiaofu/skill-follow-person-rbnx
sudo -v
```

- 第一条：进到正确目录（**每条命令都从这儿出发**）
- 第二条：输入一次密码。**屏幕不显示是正常的**，输完回车即可。5 分钟内不用再输

> 为什么总要 sudo：脚本要操作 docker 容器，需要管理员权限。

---

## 一、起服务

### 1.1 先确认导航已停

跟随和导航抢同一个底盘总线，**不能同时开**。

```bash
bash scripts/follow.sh status
```

输出最后两行必须是：

```
  跟随栈的底盘进程: 0
  导航栈的底盘进程: 0        ← 必须是 0
```

**如果不是 0**：导航还在跑，先停导航：

```bash
cd /mnt/ssd/wxf/jiaofu/robot-agilex-ranger_mini_v3
sudo bash shutdown-robonix.sh
cd /mnt/ssd/wxf/jiaofu/skill-follow-person-rbnx     # 再回来
```

### 1.2 （可选）决定用哪种模式

| 模式 | 车会动吗 | 什么时候用 |
|---|---|---|
| **dry-run（默认）** | ❌ 不动 | 第一次调相机、试链路 |
| **live** | ✅ 会动 | 真正跟随 |

**第一次建议先 dry-run 走一遍**，确认相机能看到人，再切 live。

要切 live 的话，**在起服务之前**敲：

```bash
bash scripts/follow.sh live
```

### 1.3 起服务

```bash
bash scripts/follow.sh start
```

**等约 75 秒。** 成功标志是最后一行：

```
  OK 条数: 16 （应为 16）
```

**不是 16** 就说明有组件没起来，看：

```bash
sudo docker exec robonix_follow_dev bash -lc \
  'tr "\r" "\n" < /root/deploy/logs/start_follow.log | grep FAIL'
```

---

## 二、检查服务状态

### 2.1 四项检查（起完服务、动车之前必跑）

```bash
bash scripts/follow.sh check
```

四项**必须全绿**：

```
[ OK ] 推理服务已加载模型
[ OK ] 意图路由在线
[ OK ] /camera_435i/color/image_raw  29.951 Hz
[ OK ] /camera_435i/depth/image_raw  29.875 Hz
[ OK ] /camera_435i/color/camera_info  30.176 Hz
[ OK ] /cmd_vel 发布者 1 个 —— live，车可以动
```

最后一行：
- `0 个` = 空挡（dry-run），车不会动
- `1 个` = 挂挡（live），车可以动
- **大于 1 个 = 危险，立刻 `bash scripts/follow.sh stop`**

### 2.2 随时看总体状态

```bash
bash scripts/follow.sh status
```

显示：容器在不在跑、当前是什么模式、底盘被谁控制。

---

## 三、使用

### 3.1 开始跟随

**让被跟随的人站到车正前方 6 米处。**

车会立刻前进，追到 5 米停下。

> ⚠️ **人必须站得比目标距离远，车才往前开。**
> 目标 5 米 → 人站 6 米。人若站 2 米，车会**停住不动**——
> 它想退到 5 米但被禁止倒车。**这是设计，不是故障。**

### 3.2 跟随中会发生什么

| 人的动作 | 车的反应 |
|---|---|
| 往后退 | 追上去（越远越快，最快 0.5 米/秒） |
| 往前靠近 | 减速，到 5 米停下 |
| 继续靠近 | **停住不动**（不倒车） |
| 左右移动 | 转向跟住，同时保持距离 |
| 走出画面 | 停止 |

### 3.3 实时监控（建议开着）

**另开一个终端**（`Ctrl+Shift+T` 新建标签页），在里面敲：

```bash
cd /mnt/ssd/wxf/jiaofu/skill-follow-person-rbnx
sudo -v
bash scripts/follow.sh watch
```

每秒刷新一张表，显示指令流经的 5 个环节和当前速度。`vx` 就是车的前进速度。

**另一路画面**：浏览器打开 `http://192.168.1.50:5801/web`
（能看到摄像头画面、AI 识别框）

### 3.4 改跟随距离

改配置文件：

```bash
nano robonix_manifest.yaml
```

找到这一行（约第 188 行），改成想要的值（**允许 0.5 ~ 7.0 米**）：

```yaml
      target_distance_m: 5.0
```

保存：`Ctrl+O` → 回车 → `Ctrl+X`。

然后**必须重启才生效**：

```bash
bash scripts/follow.sh stop
bash scripts/follow.sh start
```

---

## 四、安全停止跟随

**分两种情况，别搞混：**

### 4.1 只想让车停下，但服务继续跑

| 方法 | 操作 | 说明 |
|---|---|---|
| **遥控器**（最快） | 按遥控器上的停止/急停键 | 任何时候都能用，推荐 |
| **人走出画面** | 被跟随的人走出摄像头视野 | 车会自动停（内置保护） |

> **注意**：目前**没有**"暂停跟随但不关服务"的软件命令。
> 要立刻让车停，**用遥控器**。

### 4.2 完全停止跟随（关掉服务）

见下一节。

---

## 五、安全关闭服务

### 5.1 推荐顺序

**第一步**：先确认车已经停了（用遥控器停住）。

**第二步**：切回空挡（这样下次启动默认是安全的）：

```bash
bash scripts/follow.sh dry-run
```

**第三步**：关服务：

```bash
bash scripts/follow.sh stop
```

**成功标志**（最后一行）：

```
[ OK ] follow stack shutdown complete — can_ranger + ports released
```

看到这行就说明**底盘总线已释放**，可以安全地起导航了。

### 5.2 确认关干净了

```bash
bash scripts/follow.sh status
```

看 `/cmd_vel` 那行应该显示 `Publisher count: 0`，且两个底盘进程都是 0。

### 5.3 恢复导航

```bash
cd /mnt/ssd/wxf/jiaofu/robot-agilex-ranger_mini_v3
bash start-no-arm.sh
```

---

## 六、速查表

**位置：机器人主机，`/mnt/ssd/wxf/jiaofu/skill-follow-person-rbnx` 目录**

| 我要… | 输入 |
|---|---|
| 看总体状态 | `bash scripts/follow.sh status` |
| **起服务** | `bash scripts/follow.sh start` |
| **检查服务（四项）** | `bash scripts/follow.sh check` |
| 看实时速度 | `bash scripts/follow.sh watch`（新终端） |
| 切挂挡 | `bash scripts/follow.sh live` 然后 stop → start |
| 切空挡 | `bash scripts/follow.sh dry-run` 然后 stop → start |
| **安全停止跟随** | 遥控器按停 |
| **安全关闭服务** | `bash scripts/follow.sh dry-run` → `bash scripts/follow.sh stop` |
| 进容器终端 | `bash scripts/follow.sh shell` |

---

## 七、故障速查

| 现象 | 原因 / 处理 |
|---|---|
| **车只转圈不前进** | 人比目标距离近。让人退到 5 米以外 |
| 车完全不动 | 跑 `check` 看哪项没过 |
| `start` 报端口占用 | 服务已在跑，先 `stop` 再 `start` |
| `OK 条数` 不是 16 | 看启动日志里的 `FAIL` 行 |
| 雷达 FAILED | 主机雷达网口地址不对，跑导航仓 `setup-hw.sh` |
| 车不停 | **按遥控器急停**，然后 `follow.sh stop` |
| 网页打不开 | 用 `192.168.1.50`，地址要带 `/web` |
| 命令报 sudo 错 | 回目录敲 `sudo -v` 输密码 |

**深入排查**见 `docs/RUNBOOK.md`；验收测试见 `docs/ACCEPTANCE.md`。
