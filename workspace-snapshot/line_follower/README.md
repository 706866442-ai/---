# STM32F103C8T6 循迹小车固件

接线以 [接线总表](../接线总表.md) 为准；本工程没有改动已有引脚。旧 STM32 曾冒烟，不能拿它继续测试。新板已有旧版短程循迹固件烧录记录；本次新增代码的烧录状态以最新构建和上传记录为准，电脑测试不能当作实车验证。

## 已实现的功能

- 上电先停机；USER 短按切换模式，按约 1 秒松开测试左、右轮，按至少 2 秒松开启动，运行时按键停机。
- 模式 0：灰度比例短程循迹，最长 10 秒；灰度通信失败、全白或全黑时停机。
- 模式 1：从 A 沿黑线到 B，在接近 1 米处连续检测两次全黑交叉标记后停机，最长 10 秒。
- 模式 2：四边循迹，转角用编码器控制约 90°并重新捕获黑线，第四段回到 A 后停机，最长 30 秒。
- 模式 3/4：从实测固定起点分别倒库、侧停；编码器反馈控制轮程、故障停机。停车路径是几何初值，尚无车身碰撞检测。
- 可选双轮速度 PID 内环；灰度偏差给左右轮不同速度目标。默认关闭，待编码器与速度标定后才可调参启用。

待机 PA4 每 2 秒闪 1～5 次对应模式 0～4；运行时快闪。故障时每 4 秒显示一次 0.8 秒长亮加 N 次短闪：1=灰度握手失败，2=读取失败，3=全白，4=全黑，5=配置未验收，6=路线/停车/反馈故障。故障由按键确认后回待机。

## 配置锁与实车边界

配置都在 `include/RobotConfig.h`。目前 `ENCODER_INPUTS_ENABLED=false`、`ENCODERS_VERIFIED=false`、`SPEED_PID_ENABLED=false`、`ROUTE_START_POSE_VERIFIED=false`、`PARKING_START_POSE_VERIFIED=false`，左右每计数毫米数和停车参数仍为 0。故当前可运行的是原短程循迹；自动到 B、绕圈、速度 PID 和停车会拒绝启动。不要凭估计解除这些锁。

已有用户反馈：A 通道带左轮、B 通道带右轮，两轮正转向前；灰度 1 号探头在车头朝前时位于左侧。四根反馈线已获用户确认接入 E1A→PB13、E1B→PB14、E2A→PB15、E2B→PA8；**编码器信号电压、脉冲、E1/E2 左右归属及正负号尚未验证**。轮径记录 65 mm，轮距 120 mm 只是估计。路线的全黑角点规则也还没有赛道实测。

模式 1 不需要确认拐弯方向；模式 2 还要求 `ROUTE_TURN_SIGN` 为 +1（左）或 -1（右）。停车目前从标定起点单独运行，还未实现正式发车后循迹到车位起点再自动停车。没有视觉模块，所以罗马数字、颜色和形状识别尚不能实测或启用；MPU6050 暂非必需。

现场读数和动作按 [现场操作清单](现场操作清单.md) 逐项完成，把结果交给我填配置、重烧录。旧版固件或电脑测试的成功，不代表新版已完成车辆验收。

## 构建与测试

PlatformIO 环境 `bluepill_f103c8`，`pio run` 构建，`pio run -t upload` 通过 ST-LINK 烧录。电脑端测试位于 `tests/parking_test.cpp`、`tests/route_test.cpp`、`tests/wheel_speed_test.cpp`；涵盖路径状态、编码器故障和速度控制计算，但不能代替电气与实车测试。


## LinePid 外环整合（2026-10-15）

外环位于 `include/LinePid.h`，参数在 `include/RobotConfig.h`。默认 `LINE_PID_ENABLED=false`、增益为零，保留原比例循迹；启用但参数无效会拒绝启动。引脚及硬件验收锁保持原值。

- Follow：仅启用外环时输出 PWM 修正；同时启用速度内环时，将外环输出除以 `LINE_MAX_CORRECTION` 归一化，再乘 `SPEED_LINE_CORRECTION_MMPS` 转换为左右目标速度差，由原速度内环执行。
- Route：直线段使用外环 PWM 修正，转角仍由原路线控制器执行；Route 尚未接速度内环。
- 使用实际控制周期，超过 150 ms 停机；停止、故障、重新启动和路线转角时清除外环历史。微分低通、积分限幅和抗饱和由 LinePid 提供。
- 新增 `tests/line_pid_test.cpp`；先调 Kp，再少量 Kd，Ki 通常从零开始。PWM 输出上限与速度修正比例需分别标定。

本次只合并外环，未采用附件中额外的 I²C 寄存器修改、路线新接口或历史实测记录；未烧录、未实车验收。合并前文件保存在 `before_line_pid/`。

验证结果：LinePid、Parking、Route、WheelSpeedControl 四组主机测试全部通过；PlatformIO `bluepill_f103c8` 编译通过（RAM 2372 B，Flash 34948 B）。

本机复现命令（PowerShell，在本工程目录运行）：

```powershell
./tests/run_host_tests.ps1 -Compiler 'D:/桌面/西电/Dev-Cpp/MinGW64/bin/g++.exe'
$env:PLATFORMIO_CORE_DIR='C:/Users/0/AppData/Local/Temp/pio_core_car_task'
& 'C:/Users/0/.platformio/penv/Scripts/pio.exe' run
```

这里显式设置 core 目录，避免本机全局环境变量指向另一处受限缓存。
