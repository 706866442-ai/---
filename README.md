# 电赛-小车

归档日期：2026-10-15。STM32F103C8T6 / PlatformIO / Arduino 小车项目源码与资料快照。

## 目录与来源

- `gray_i2c_probe/`：迁移后当前目录中的灰度通信、独立编码器诊断、轮速控制库与测试。
- `development/`：迁移后的开发分支完整源码与场地图 PDF；`line_follower/` 为交接文档指定的主工程，另含 `gray_reader/` 与开发版灰度工程。
- `workspace-snapshot/`：原工作区保留的版本，包含不同的循迹/PID实现、测试和 `before_line_pid` 备份；与 development 不自动合并。
- `workspace-snapshot/mpu6050_sw_i2c/`：仅有 PlatformIO 配置，尚无传感器实现源码。MPU6050 接入规划见 [Codex项目交接.md](Codex项目交接.md)。
- `history/stm32_blink/`：历史 PA4 闪灯工程及当时的文档。

## 接线和状态

当前接线以根目录 [接线总表.md](接线总表.md) 为主表。workspace-snapshot 与 history 中的接线表仅为原版本快照，不作为当前接线依据。

原有 AGENTS.md 中关于主程序和 PID 的状态描述与较新的交接文档存在差异，归档保留原文；主工程及未验收项目请结合最新接线主表、交接文档和具体代码核对。

本次只归档，未修改控制代码、未重新编译、未烧录，也未执行硬件测试。原文中的历史验证记录不代表本次验证。

## 使用

安装 PlatformIO 后，在对应工程目录运行 `pio run` 编译。各工程配置保留了本机 `D:/car-data/...` 的 core_dir、build_dir 等路径；换电脑使用前需调整为自己的有效路径，Windows 下建议使用纯英文构建路径。

已排除工具链、.pio 编译缓存、测试可执行文件、编辑器本机配置、临时渲染文件和本机迁移脚本。源码及现有文档保留原始内容，PDF 保留原始字节。

## 2026-10-15 C 盘循迹程序更新

按本次指定的 C 盘 `line_follower` 工程完成程序工作，更新位于 [workspace-snapshot/line_follower](workspace-snapshot/line_follower)。上述“本次只归档”描述对应初始归档；本次新增控制代码及验证如下。`development` 仍保留另一套源码，两者未自动合并，测试结论不可互用。

- 完成内外环配合、独立调度与保护，补充泊车/出库任务状态机、视觉帧解析、限量诊断输出和调参工具。
- 11 项主机测试通过；默认 STM32 固件 Flash 41016 B / RAM 3556 B；隔离启用任务与视觉的验证配置 Flash 44228 B / RAM 3024 B，均编译成功。详见 [验证日志](workspace-snapshot/line_follower/verification/2026-10-15)。
- 未烧录、未实车验收；完整任务和视觉默认关闭，方向、路径和通信协议需实测确认。隔离测试配置使用模拟标定值，不可烧录上车。
- [已完成清单](workspace-snapshot/line_follower/docs/01_已完成清单.md)
- [还需完成清单](workspace-snapshot/line_follower/docs/02_还需完成清单.md)
- [所需实物数据清单](workspace-snapshot/line_follower/docs/03_所需实物数据清单.md)

根目录接线主表仍为当前实物记录；本次 C 盘的软件配置说明保存在对应快照接线表中，未改变实际接线。

## 编码器双路原始计数显示

C 盘快照中的 `gray_i2c_probe/encoder_probe` 已增加 PA2 串口输出：每 100 ms 提供 E1/E2 累计计数、增量、无效跳变及 A/B 电平。此探针已编译，尚未烧录；须先核对编码器供电和 3.3 V 逻辑电平。详情见 [探针说明](workspace-snapshot/gray_i2c_probe/编码器测试说明.md)。
