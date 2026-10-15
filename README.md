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
