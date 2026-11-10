# 自研固件模板工程 v1（VS Code + ARM GCC 链，占位型号 STM32F407VE）

## 前置
1. 装 ARM 工具链（任一）：
   - Homebrew：`brew install --cask gcc-arm-embedded`，装完新开终端验证 `arm-none-eabi-gcc --version`
   - 或官网 pkg：developer.arm.com → Arm GNU Toolchain → macOS x86_64 版（免账号）
2. VS Code 扩展（扩展面板搜名字装）：`C/C++`（Microsoft）、`CMake Tools`（Microsoft）、`Cortex-Debug`（marus25，以后 ST-Link 调试用）

## 编译
VS Code 打开本文件夹 → 终端执行 `./build.sh`
产物：`build/firmware.bin`，脚本结尾自动打印 bin 头 8 字节自检：
- 前 4 字节应为 `00 00 02 20`（栈顶 0x20020000）
- 后 4 字节低字节为奇数、且落在 0x0801xxxx 段（复位地址）——与 dl1824.bin 同格式即算链路通。

## 注意
- 本模板只验证「能编译出格式正确的 bin」，**不要拿它刷任何板子**（芯片型号/引脚都未确认）。
- 芯片丝印确认后要改：链接脚本 FLASH/RAM 大小、启动文件换官方版、再开始写业务代码。
