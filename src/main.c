/* 占位主程序：仅验证编译链。正式工程在此接入自研力限算法。 */
#include <stdint.h>
volatile double g_demo;   /* 顺带验证硬件浮点（Cortex-M4F）已启用 */
int main(void)
{
    double a = 3.14159, b = 2.0;
    for (;;) { g_demo = a * b; a += 0.000001; if (a > 4.0) a = 3.14159; }
    return 0;
}
