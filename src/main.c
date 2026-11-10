/* 自研固件主入口
 * 当前阶段：台架 Demo（模拟量输入控制指示灯，见 demo_main.c / demo/README.md）。
 * 后续力限算法在此处接入，替换/扩展 demo_main 的调用。
 */
#include "demo.h"

int main(void)
{
    demo_main();
    for (;;) { }
    return 0;
}
