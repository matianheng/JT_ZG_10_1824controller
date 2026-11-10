# 开发手册：环境搭建 + 编译打包（实录版）

> 本文档记录的是 2026-11-10 在马哥的 Mac（2018 款 Intel i7 / 16GB）上**实际走通**的一套流程和命令，
> 不是通用教程。换一台新电脑从零搭建，照第一节从头做一遍即可；日常开发只看第二节。
> 工程位置（本机）：`~/Documents/JUNTONG/JT_ZG_10_1824controller`
> 仓库：`github.com/matianheng/JT_ZG_10_1824controller`

---

## 一、环境搭建（一次性，从零到能编译）

### 0. 前提

- 有 VS Code（已装）、Git、Homebrew（已装）。
- 终端凡是要访问 GitHub（含 brew 拉配方），**先挂代理**，否则 `raw.githubusercontent.com` 会被连接重置：

```bash
export http_proxy=http://127.0.0.1:7897
export https_proxy=http://127.0.0.1:7897
```

（7897 是本机代理口；或开代理软件的全局/TUN 模式，二选一。）

### 1. 装 VS Code 扩展（4 个）

扩展面板搜 ID 安装，或终端一条命令（见下方注意）：

| 扩展 | ID | 用途 |
|---|---|---|
| C/C++ | `ms-vscode.cpptools` | 代码补全/跳转/报错 |
| CMake Tools | `ms-vscode.cmake-tools` | 在 VS Code 里配置与构建 CMake 工程 |
| CMake | `twxs.cmake` | CMakeLists 语法高亮（注意发布者是 twxs，不是 ms-vscode，那个不存在） |
| Cortex-Debug | `marus25.cortex-debug` | ST-Link 单步调试（有调试器时用） |

终端安装命令（本机 `code` 命令默认不在 PATH，要用完整路径）：

```bash
/Applications/Visual\ Studio\ Code.app/Contents/Resources/app/bin/code --install-extension ms-vscode.cpptools --install-extension ms-vscode.cmake-tools --install-extension twxs.cmake --install-extension marus25.cortex-debug
```

> 想以后直接用 `code` 命令：VS Code 里 `Cmd+Shift+P` → 搜 `Shell Command: Install 'code' command in PATH` 执行一次。
> 注意：这条命令是敲在**终端**里的，不是敲在命令面板（Cmd+Shift+P）里。

### 2. 装 ARM 工具链（编译器）

```bash
brew install --cask gcc-arm-embedded
```

- 过程中会用 `sudo` 要一次 Mac 登录密码，**输密码时屏幕不回显任何字符，属正常**，打完回车即可。
- 装完**新开一个终端标签**（或先 `hash -r`），验证：

```bash
arm-none-eabi-gcc --version
# 本机实际结果：arm-none-eabi-gcc (Arm GNU Toolchain 14.2.Rel1) 14.2.1
```

### 3. 装 CMake（构建工具）

```bash
brew install cmake
```

- 本机实际：CMake **4.4.5**。注意本机 macOS 在 Homebrew 属 Tier 3、无预编译瓶，brew 会**从源码编译**，
  十几到几十分钟、风扇转、长时间无输出都是正常现象，不要中断。
- 不想等源码编译的替代路线：到 cmake.org/download 下 macOS universal 的 dmg，拖进“应用程序”后执行：

```bash
sudo "/Applications/CMake.app/Contents/bin/cmake-gui" --install
```

- 验证：`cmake --version`

### 4. 拉取工程并首次编译（验证环境）

```bash
cd ~/Documents/JUNTONG/JT_ZG_10_1824controller
git pull            # 或首次：git clone https://github.com/matianheng/JT_ZG_10_1824controller.git
chmod +x build.sh   # 只需一次：从 GitHub 拉下来的 build.sh 可能没有可执行权限
./build.sh
```

看到 `build/dl1824.bin` 生成、且末尾自检通过（见第二节第 3 条），环境即搭建完成。
**本机已于 2026-11-10 实测通过。**

### 5.（可选，暂缓）STM32CubeIDE / STM32CubeProgrammer

- 官网：https://www.st.com/en/development-tools/stm32cubeide.html（Get Software 选 macOS x86_64 包，免费注册账号）
- 2026-11-10 当天 ST 登录服务 503（服务器维护），账号已注册成功，等恢复后补装即可，不影响现行 VS Code 路线。

---

## 二、日常编译打包（每次改完代码）

### 1. 一键流程

```bash
cd ~/Documents/JUNTONG/JT_ZG_10_1824controller
./build.sh
```

产物在 `build/` 目录：

| 文件 | 说明 |
|---|---|
| `dl1824.bin` | **最终交付物**：裸 bin，基址 0x08010000，将来走原厂 bootloader 刷入 |
| `firmware`（.elf） | 带调试符号的可执行文件，ST-Link 调试时用 |
| `firmware.map` | 链接映射表，查每个函数/变量的地址时看它 |

### 2. build.sh 内部到底做了什么（等价手动命令）

```bash
# 第 1 步 配置（生成构建系统；改了 CMakeLists/增删源文件后才需要重跑，build.sh 每次都会跑）
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake

# 第 2 步 编译 + 链接（按 STM32F407VE_APP.ld 链接，FLASH 起始 0x08010000）
cmake --build build

# 第 3 步 生成 bin 与看体积（已写在 CMakeLists 的 POST_BUILD 里，编译时自动执行）
arm-none-eabi-objcopy -O binary build/firmware build/dl1824.bin
arm-none-eabi-size build/firmware
```

VS Code 图形操作与上面等价：打开工程文件夹 → 左侧 CMake 面板选本工程的工具链文件
（`cmake/arm-none-eabi.cmake`）→ 点 Build。日常推荐还是终端 `./build.sh`，输出最全。

### 3. 打包前自检（必做）

build.sh 末尾会自动打印 bin 头 8 字节，逐项核对：

```bash
xxd -l 8 build/dl1824.bin
# 本机模板实测：00000000: 0000 0220 cd04 0108
```

| 字节 | 应为 | 含义 |
|---|---|---|
| 前 4 字节 | `00 00 02 20`（即 0x20020000） | 初始栈顶，必须落在 RAM 区 |
| 后 4 字节 | 0x0801xxxx 且**最低位为 1**（奇数，如 cd04 0108 → 0x080104CD） | 复位入口，必须在应用区 0x08010000 起的范围内、Thumb 地址为奇数 |

两条都对，格式才与原厂 dl1824.bin 一致。**任一不对不要刷，先查链接脚本。**

### 4. 干净重编（怀疑缓存/改了链接脚本时）

```bash
rm -rf build
./build.sh
```

### 5. 正常现象，不是报错

- 链接时刷 `_close/_read/_write/_lseek is not implemented and will always fail` 警告：
  裸机 + nano 库的系统调用桩未实现，本工程不用文件 IO，忽略。
- `This is a Tier 3 configuration`：Homebrew 对本机 macOS 版本的支持等级提示，忽略。

### 6. 安全线（刷写）

- 当前模板/自研 bin **只许刷备用板验证**，bootloader 是否对 bin 有额外校验尚未实测；
  在役车的唯一一块控制器**不要试刷**。刷写方法等备用板到手、整片备份原厂 Flash 后再补进本手册。

---

## 三、故障速查（本机实际踩过的坑）

| 现象 | 原因 | 解法 |
|---|---|---|
| 把 `code --install-extension ...` 敲进命令面板没反应 | 那是终端命令，命令面板只认 VS Code 功能 | 开内置终端（菜单：终端→新建终端）再敲 |
| `bash: code: command not found` | `code` 未注册到 PATH | 用完整路径命令（见一.1），或执行一次 Install 'code' command in PATH |
| `Extension 'ms-vscode.cmake' not found` | ID 写错，微软没有这个扩展 | 装 `twxs.cmake` |
| `arm-none-eabi-gcc: command not found` | 工具链未装或 PATH 未刷新 | 按一.2 安装；已装则新开终端标签或 `hash -r` |
| `./build.sh: Permission denied` | 拉下来的 build.sh 无可执行位 | `chmod +x build.sh`（只需一次）或 `bash build.sh` |
| `./build.sh:4: permission denied: cmake` | 本机没装 CMake | 按一.3 安装 |
| brew 下载 `Connection reset by peer` | 终端没走代理，GitHub raw 域名被重置 | 按一.0 给终端挂代理（7897）或开全局/TUN 再重试 |
| brew 装 cmake 长时间停在 `make` | Tier 3 无预编译瓶、正在源码编译 | 正常，等它编完；或 Ctrl+C 改走 dmg 路线（一.3） |
| brew 装工具链时卡在 `Password:` | 在等 sudo 密码且不回显 | 输入 Mac 登录密码后回车 |
