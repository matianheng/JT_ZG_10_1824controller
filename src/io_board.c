/* 板级 IO 实现（占位版）
 *
 * 本文件是自研固件与真实硬件之间唯一需要按板子填写的文件。
 * 当前为占位实现：保证工程完整编译、链接出 dl1824.bin；
 * 三个函数均为安全空实现（读数 0、输出关、软件计数计时），
 * 【不可直接上板当真实功能使用】。
 *
 * 待确认后填写（依赖：主芯片型号 + 连接器针脚到 MCU 引脚的板内对应）：
 *  1) io_read_bench_voltage：
 *     - J1-1 对应 ADC 通道采样（多次平均在逻辑层已做，这里给单次换算值）；
 *     - 脚电压换算：pin_v = adc_raw / ADC_FULL * AI_FULLSCALE_V
 *       （AI_FULLSCALE_V 取决于模块 AI 前端分压，须台架标定：
 *        给 J1-1 加已知电压、读原始 AD，反推满量程）；
 *     - 台架电压换算：台架侧满量程 24V，电位器供电端串 7.5K 后脚压最高 9.6V，
 *       bench_v = pin_v * (24.0 / 9.6) = pin_v * 2.5。
 *  2) io_write_lamp：J1-17 对应 DOH 驱动输出置 1/0（高边 24V）。
 *  3) io_millis：用 SysTick 1kHz 中断累加（需在启动代码向量表挂 SysTick_Handler，
 *     并按确认后的主频配置重装值）；当前占位版每次调用自增，节拍不准，仅保链接。
 */
#include "io.h"

float io_read_bench_voltage(void)
{
    /* TODO(板级): ADC 采样 J1-1，按上方注释换算后返回台架侧电压 */
    return 0.0f;
}

void io_write_lamp(int on)
{
    /* TODO(板级): J1-17 DOH 输出 on=1 输出 24V / on=0 关断 */
    (void)on;
}

unsigned long io_millis(void)
{
    /* TODO(板级): 改为 SysTick 1kHz 累加值 */
    static unsigned long cnt = 0;
    return ++cnt;
}
