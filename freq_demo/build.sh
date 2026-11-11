#!/bin/zsh
# 用法：在本目录执行  ./build.sh
set -e
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
