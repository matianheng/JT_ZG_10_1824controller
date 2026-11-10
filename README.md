# JT_ZG_10_1824controller

骏通 JT_ZG_10 配套 1824 控制器**自研固件**工程（替代原厂 dl1824.bin）。
平台：STM32F4 级 Cortex-M4F 裸机（型号待芯片丝印确认，当前占位 STM32F407VE）；
链接基址 0x08010000（前 64KB 为原厂 bootloader），目标产物为可经原 bootloader 刷入的裸 bin。

## 构建（Mac + VS Code 路线）

1. 装 ARM 工具链：`brew install --cask gcc-arm-embedded`（或 developer.arm.com 官方 pkg）
2. VS Code 扩展：C/C++、CMake Tools、Cortex-Debug
3. 本目录执行 `./build.sh` → 产物 `build/firmware.bin`，脚本尾部自动做 bin 头 8 字节自检

## 当前状态

- 模板 v1：仅验证「编译 → 格式正确的 bin」链路，**不可刷任何板子**
- 业务开发待：芯片型号确认、引脚映射恢复（bin 反汇编进行中）、屏↔控制器协议（已从安卓源码恢复）
