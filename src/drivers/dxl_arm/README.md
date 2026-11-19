# dxl_arm：CUAV 7-Nano 三舵机控制

基于 PX4 官方 `release/1.18`（`8303efda43`），实现三台 DYNAMIXEL XL430-W250-T 的串口诊断、同步位置控制和遥控三档动作。串口通信、三档动作和开机自启已在台架验证；未进行飞行验证。

## 改动位置

| 文件 | 作用 |
| --- | --- |
| [`dxl_arm.cpp`](dxl_arm.cpp) | 打开串口，按 DYNAMIXEL Protocol 2.0 收发报文（CRC16、字节填充、超时处理）；同步写入目标、读取反馈并实现遥控后台任务 |
| [`rc_motion.hpp`](rc_motion.hpp) | 定义三档动作、回中门槛和正弦轨迹 |
| [`CMakeLists.txt`](CMakeLists.txt)、[`Kconfig`](Kconfig)、[板级配置](../../../boards/cuav/7-nano/default.px4board) | 注册 `dxl_arm` 并加入 `cuav_7-nano_default` 固件 |
| [板级启动脚本](../../../boards/cuav/7-nano/init/rc.board_extras) | 开机执行 `dxl_arm start`，无需在 MAVLink Console 手动启动 |

## 接线与配置

- **TELEM1**（`/dev/ttyS5`）→ TTL 半双工转换板（本项目使用微雪串行总线舵机驱动板）→ 三台舵机，ID `1/2/3`，`57600 baud`、`8N1`，使用位置控制模式。舵机独立供电并与飞控共地。
- **TELEM2**（`/dev/ttyS3`）→ ELRS/CRSF 接收机。PX4 参数 `RC_PORT_CONFIG` 选 `TELEM 2`（`102`）；遥控器将三段拨杆 `SC` 映射到未占用的 `CH7`。实测上、中、下档分别为 `2000 / 1503 / 999`，与舵机位置单位无关。
- 两个串口都不能同时分配给 MAVLink 或其他服务；仅需取消实际占用这两个口的 `MAV_x_CONFIG` 分配，不用停用 USB 上的 QGC 连接。

## 编译与使用

```bash
git clone --recursive --branch dxl-arm https://github.com/supsmwhite/PX4-Dynamixel-Arm.git
cd PX4-Dynamixel-Arm
make cuav_7-nano_default
make cuav_7-nano_default upload
```

生成的固件位于 `build/cuav_7-nano_default/cuav_7-nano_default.px4`。拆桨、固定并支撑机构，保留可随时切断的舵机电源。上电后保持飞控未解锁，接通遥控，把 `SC` 放在中档。后台检查三台舵机的模式、限位和初始位置，需要时开启力矩。**等三台实际回到 `2000` 附近，再拨上档或下档**；快速经过中档不会放行。

| `SC` 档位 | 三台舵机的动作 |
| --- | --- |
| 上 | 一次同步下发 `2800`，由舵机内部的位置控制执行 |
| 中 | 一次同步下发 `2000`；编码器反馈确认到位后才允许切入外档 |
| 下 | 从 `2000` 平滑接入 `1900～2500` 的正弦往复，约 7 秒一周期 |

台架复验通过：切到上档时直接下发 `2800`，切回中档时直接下发 `2000`，三台舵机均按目标动作。

遥控失联或飞控解锁后，后台停止下发目标；恢复后须回中才能重新进入外档。**停止下发不等于切断力矩**，舵机可能继续执行最后收到的目标。动作值按当前台架设定，换机构或舵机后应重新验证行程。

## 控制台诊断

飞控开机自动启动遥控后台并占用舵机串口。在 QGC 的 MAVLink Console 执行 `dxl_arm stop`，用 `dxl_arm status` 确认 `stopped` 后，再跑手动串口命令。调试结束后把 `SC` 放回中档，执行 `dxl_arm start` 恢复遥控。

| 命令 | 功能 |
| --- | --- |
| `dxl_arm status` / `dxl_arm stop` / `dxl_arm start` | 查询、停止或启动遥控后台任务；`stop` 后等状态为 `stopped` 再操作串口 |
| `dxl_arm rc_test` | 60 秒只读档位诊断，不向舵机发送指令 |
| `dxl_arm ping` / `dxl_arm read` | 点名三台设备，或同步读取实际位置（地址 `132`） |
| `dxl_arm read --timing` | 只发一轮实际位置查询，记录各台回包及三台收齐的耗时，不发运动指令 |
| `dxl_arm move <p1> <p2> <p3>` | 开启力矩，同步写入并读回目标位置（地址 `116`）；读回一致不代表已经到位 |
| `dxl_arm move --timing <p1> <p2> <p3>` | 台架计时：只发一次新目标，记录目标读回与到位观测时间 |
| `dxl_arm rc_live` | 手动启动 60 秒遥控台架测试，正常使用无需调用 |

## 台架计时

保持飞控未解锁、拆桨并支撑机构。停止遥控后台后，按需要执行以下测试。字段定义、原始读数和统计结果见[台架延迟实测](LATENCY.md)。

### 读取位置耗时

执行 `dxl_arm read --timing`，无需先移动舵机。`pos` 是编码器原始位置，`read_us` 是包含回包的耗时（微秒）。需要三台位置都可用时，看 `Read timing all 3`；任一台缺包或报错时不会输出该行。

### 动作计时

先使能力矩并回中，等三台停稳并确认实际位置均在 `2000 ±15`，再发新目标：

```text
dxl_arm move 2000 2000 2000
dxl_arm read
dxl_arm move --timing 2800 2800 2800
```

`goal_us` 是首次读回新目标的时间；`last_out_us` 和 `first_in_us` 用于估计进入目标容差的区间；`settled_us` 是连续两次在范围内的第二次确认时间。位置轮询设定为 `10 ms`，收齐上一轮回包后才会发下一轮。比较多轮时，每轮先回中并保持负载、供电一致。
