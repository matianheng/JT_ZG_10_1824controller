#!/bin/zsh
# 用法：在本目录执行  ./build.sh
# 有 arm-none-eabi-gcc 时：CMake 交叉编译 + 填充至 189551 字节，产物 build/dl1824.bin。
# 无交叉工具链时（本机）：只做宿主 cc 严格语法检查（与用户 Mac 同等严格度），
# 不产生可刷 bin，并在最后明确提示。
set -e

if ! command -v arm-none-eabi-gcc >/dev/null 2>&1; then
    echo "--- 本机无 arm-none-eabi-gcc，改做宿主 cc 严格语法检查（不产生可刷 bin）---"
    # 只查 src/main.c：-Wno-int-to-pointer-cast 用于屏蔽 64 位宿主机的已知
    # 假告警（32 位寄存器地址转指针在宿主机上宽窄不等，目标机上不存在）；
    # -Werror=implicit-function-declaration / -Werror=implicit-int 仍为错误。
    # src/startup.c 不参与宿主检查：它是 freq_demo 已验证的逐字复制件，
    # 其向量表指针初始化在 64 位宿主机上必然报“initializer element is not
    # constant”，在 32 位目标机上不存在；真正的编译由交叉工具链完成。
    cc -std=c11 -Wall -Wextra -Werror=implicit-function-declaration -Werror=implicit-int \
       -Wno-int-to-pointer-cast -fsyntax-only src/main.c
    echo "宿主语法检查通过：src/main.c（-Werror=implicit-function-declaration -Werror=implicit-int）"
    echo "注意：要得到可刷的 build/dl1824.bin，请在装有 arm-none-eabi-gcc 的机器上再跑 ./build.sh"
    exit 0
fi

cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake > /dev/null
cmake --build build
# mimic: objcopy 产物填充到与原厂 dl1824.bin 相同的 189551 字节（不足填 0xFF，
# 超长视为异常直接报错），使镜像在“大小”上与原厂一模一样，排查验收条件
python3 - << 'PYEOF'
import os, sys
path = "build/dl1824.bin"
target = 189551
size = os.path.getsize(path)
if size > target:
    sys.exit("错误：%s 已 %d 字节，超过原厂大小 %d，不填充" % (path, size, target))
if size < target:
    with open(path, "ab") as f:
        f.write(b"\xFF" * (target - size))
    print("已填充：%d -> %d 字节（0xFF）" % (size, target))
else:
    print("无需填充：已是 %d 字节" % target)
PYEOF
echo "--- bin 头 8 字节自检（应为 0x20006538 小端：38 65 00 20 + 0x0801xxxx 奇数地址）---"
xxd -l 8 build/dl1824.bin
ls -lh build/dl1824.bin
echo "填充后文件大小：$(wc -c < build/dl1824.bin) 字节（原厂 dl1824.bin 同为 189551 字节）"
