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
| 上 | 逐步抬升到 `2800` |
| 中 | 逐步回到 `2000`；编码器反馈确认到位后才允许切入外档 |
| 下 | 从 `2000` 平滑接入 `1900～2500` 的正弦往复，约 7 秒一周期 |

遥控失联或飞控解锁后，后台停止下发目标；恢复后须回中才能重新进入外档。**停止下发不等于切断力矩**，舵机可能继续执行最后收到的目标。动作值按当前台架设定，换机构或舵机后应重新验证行程。

## 控制台诊断

| 命令 | 功能 |
| --- | --- |
| `dxl_arm status` / `dxl_arm stop` / `dxl_arm start` | 查询、停止或启动遥控后台任务；`stop` 后等状态为 `stopped` 再操作串口 |
| `dxl_arm rc_test` | 60 秒只读档位诊断，不向舵机发送指令 |
| `dxl_arm ping` / `dxl_arm read` | 点名三台设备，或同步读取实际位置（地址 `132`） |
| `dxl_arm move <p1> <p2> <p3>` | 开启力矩，同步写入并读回目标位置（地址 `116`）；读回一致不代表已经到位 |
| `dxl_arm rc_live` | 手动启动 60 秒遥控台架测试，正常使用无需调用 |

执行 `ping`、`read`、`move`、`rc_live` 前先停止后台任务。通信延迟和动作到位耗时的专项测量尚未实现。
