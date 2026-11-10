/* Demo 主逻辑 v2：台架 0-24V 模拟量输入 -> 控制指示灯闪烁频率 / 明暗
 *
 * 两种效果模式，改 EFFECT_MODE 一个宏切换（默认闪烁频率模式，台架上最直观）：
 *   EFFECT_BLINK：输入 0V -> 灭；电压越高闪得越快（0.5Hz 慢闪 ~ 10Hz 快闪，50% 占空）
 *   EFFECT_DIM  ：PWM 调光，输入 0V -> 灭，24V -> 常亮，中间连续忽明忽暗
 *                  （软件 PWM 200Hz，人眼看不到闪烁，只看到亮度变化）
 * 采样：每 10ms 采一次，5 点滑动平均，旋钮拧动时效果平滑跟随、不跳变。
 * 输入输出脚（连接器级，DL-1018246 针脚表）：入 J1-1（AI/DIH 空脚），
 * 出 J1-17（DOH/PWM 空脚，高边 24V）。板级实现见 io.h，上板时补 io_board.c。
 */
#include "io.h"
#include "demo.h"

#define EFFECT_BLINK  1
#define EFFECT_DIM    2
#define EFFECT_MODE   EFFECT_BLINK

#define SAMPLE_PERIOD_MS  10u
#define AVG_COUNT         5u
#define IN_SPAN_V         24.0f    /* 台架输入满量程（io 层已换算回台架侧电压） */
#define BLINK_MIN_HZ      0.5f
#define BLINK_MAX_HZ      10.0f
#define DIM_PWM_PERIOD_MS 5u       /* 200Hz 软件 PWM */

static float avg_push(float *buf, unsigned *idx, float v)
{
    buf[*idx] = v;
    *idx = (*idx + 1u) % AVG_COUNT;
    float s = 0.0f;
    for (unsigned i = 0; i < AVG_COUNT; i++) s += buf[i];
    return s / (float)AVG_COUNT;
}

static float clamp01(float x)
{
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

int demo_main(void)
{
    float buf[AVG_COUNT] = {0};
    unsigned idx = 0;
    unsigned long last_sample = io_millis();

    for (;;) {
        unsigned long now = io_millis();
        static float x = 0.0f;                 /* 归一化输入 0~1 */
        if ((now - last_sample) >= SAMPLE_PERIOD_MS) {
            last_sample = now;
            x = clamp01(avg_push(buf, &idx, io_read_bench_voltage()) / IN_SPAN_V);
        }

        int on = 0;
#if EFFECT_MODE == EFFECT_BLINK
        if (x > 0.02f) {
            float hz = BLINK_MIN_HZ + (BLINK_MAX_HZ - BLINK_MIN_HZ) * x;
            unsigned long period = (unsigned long)(1000.0f / hz);
            on = ((now % period) < (period / 2u)) ? 1 : 0;
        }
#elif EFFECT_MODE == EFFECT_DIM
        {
            unsigned long phase = now % DIM_PWM_PERIOD_MS;
            unsigned long on_ms = (unsigned long)(x * (float)DIM_PWM_PERIOD_MS + 0.5f);
            on = (phase < on_ms) ? 1 : 0;
        }
#endif
        io_write_lamp(on);
    }
    return 0;
}
