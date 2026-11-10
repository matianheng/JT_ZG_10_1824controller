#!/bin/zsh
# 用法：在本目录执行  ./build.sh
set -e
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake > /dev/null
cmake --build build
echo "--- bin 头 8 字节自检（应为 0x20020000 小端 + 0x0801xxxx 奇数地址）---"
xxd -l 8 build/firmware.bin
ls -lh build/firmware.bin
