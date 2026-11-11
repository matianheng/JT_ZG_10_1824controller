#!/bin/zsh
# 用法：在本目录执行  ./build.sh
set -e
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake > /dev/null
cmake --build build
echo "--- bin 头 8 字节自检（应为 0x20006538 小端（38 65 00 20）+ 0x0801xxxx 奇数地址）---"
xxd -l 8 build/dl1824.bin
echo "--- 填充至原厂镜像大小 189551 字节（0xFF）---"
python3 - << 'PYEOF'
import sys
p = "build/dl1824.bin"
d = open(p, "rb").read()
if len(d) > 189551:
    sys.exit("错误：产物 %d 字节已超过原厂镜像大小 189551" % len(d))
if len(d) < 189551:
    open(p, "ab").write(b"\xff" * (189551 - len(d)))
print("填充后大小:", len(open(p, "rb").read()))
PYEOF
ls -lh build/dl1824.bin
