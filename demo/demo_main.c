/* Demo 主逻辑：台架模拟量输入 -> 阈值判断（带回差）-> 输出点灯
 *
 * 行为：
 *   输入电压 >= 12.0V -> 灯亮（J1-17 输出 24V）
 *   输入电压 <= 10.0V -> 灯灭
 *   10~12V 之间保持上一状态（回差，防止阈值附近抖动反复开关）
 * 采样：每 10ms 采一次，5 点滑动平均后再判阈值。
 * 编译位置：正式工程中替代模板 main 的逻辑入口；当前模板的 main.c 不动，
 * 本文件先独立存放，io_board.c 补齐后一起并入构建。
 */
#include "io.h"

#define SAMPLE_PERIOD_MS   10u
#define AVG_COUNT          5u
#define TH_ON_V            12.0f
#define TH_OFF_V           10.0f

static float avg_push(float *buf, unsigned *idx, float v)
{
    buf[*idx] = v;
    *idx = (*idx + 1u) % AVG_COUNT;
    float s = 0.0f;
    for (unsigned i = 0; i < AVG_COUNT; i++) s += buf[i];
    return s / (float)AVG_COUNT;
}

int demo_main(void)
{
    float buf[AVG_COUNT] = {0};
    unsigned idx = 0;
    int lamp = 0;
    unsigned long last = io_millis();

    for (;;) {
        unsigned long now = io_millis();
        if ((now - last) < SAMPLE_PERIOD_MS) continue;
        last = now;

        float v = avg_push(buf, &idx, io_read_bench_voltage());

        if (!lamp && v >= TH_ON_V)      lamp = 1;
        else if (lamp && v <= TH_OFF_V) lamp = 0;

        io_write_lamp(lamp);
    }
    return 0;
}
