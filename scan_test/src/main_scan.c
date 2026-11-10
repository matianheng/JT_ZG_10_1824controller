/* 输出扫描测试固件（找灯脚专用，不是正式 Demo）
 *
 * 原理：原厂固件实际使用的 50 个 GPIO 输出脚已从 bin 恢复（见
 * reference/pin_recovery_v1.md 第 5 节）。本固件依次把候选脚逐个置高
 * 1 秒（其余保持低），灯接在哪个连接器输出上，轮到驱动它的那只脚时
 * 灯就会亮 —— 记下是第几步亮的，查 README 对照表即知是哪只芯片脚。
 *
 * 候选 45 脚：50 个输出脚中排除 PA15（初始高、疑板级使能）、PA12
 * （专用翻转、疑心跳）、PE12/13/14（3 位编码组）——这 5 个留第二轮。
 * 每轮开始有标记动作：全部候选脚齐闪 3 次，便于分辨轮次与计数。
 *
 * 安全：仅限台架、连接器上只接灯与电位器、无阀类负载时使用；
 * 发现异响/异味/异常发热立即断电。
 * 时序：SysTick 轮询计时（不依赖中断向量），HSI 16MHz 默认时钟。
 */
#include <stdint.h>

#define REG32(a) (*(volatile uint32_t *)(a))
#define GPIOA_BASE 0x40020000u
#define GPIOB_BASE 0x40020400u
#define GPIOC_BASE 0x40020800u
#define GPIOD_BASE 0x40020C00u
#define GPIOE_BASE 0x40021000u
#define RCC_AHB1ENR REG32(0x40023830u)
#define STK_CTRL REG32(0xE000E010u)
#define STK_LOAD REG32(0xE000E014u)
#define STK_VAL  REG32(0xE000E018u)

typedef struct { uint32_t base; uint8_t pin; } Pin;
static const Pin kPins[] = {
    /*  1- 8 */ {GPIOA_BASE,0},{GPIOA_BASE,1},{GPIOA_BASE,2},{GPIOA_BASE,3},
                {GPIOA_BASE,8},{GPIOA_BASE,9},{GPIOA_BASE,10},{GPIOA_BASE,11},
    /*  9-19 */ {GPIOB_BASE,3},{GPIOB_BASE,4},{GPIOB_BASE,5},{GPIOB_BASE,6},
                {GPIOB_BASE,7},{GPIOB_BASE,8},{GPIOB_BASE,9},{GPIOB_BASE,10},
                {GPIOB_BASE,11},{GPIOB_BASE,14},{GPIOB_BASE,15},
    /* 20-26 */ {GPIOC_BASE,6},{GPIOC_BASE,7},{GPIOC_BASE,8},{GPIOC_BASE,9},
                {GPIOC_BASE,13},{GPIOC_BASE,14},{GPIOC_BASE,15},
    /* 27-38 */ {GPIOD_BASE,2},{GPIOD_BASE,3},{GPIOD_BASE,4},{GPIOD_BASE,5},
                {GPIOD_BASE,6},{GPIOD_BASE,7},{GPIOD_BASE,10},{GPIOD_BASE,11},
                {GPIOD_BASE,12},{GPIOD_BASE,13},{GPIOD_BASE,14},{GPIOD_BASE,15},
    /* 39-45 */ {GPIOE_BASE,0},{GPIOE_BASE,1},{GPIOE_BASE,2},{GPIOE_BASE,3},
                {GPIOE_BASE,4},{GPIOE_BASE,7},{GPIOE_BASE,15},
};
#define PIN_COUNT (sizeof(kPins) / sizeof(kPins[0]))

static void delay_ms(uint32_t ms)
{
    while (ms--) {
        STK_VAL = 0;
        while ((STK_CTRL & (1u << 16)) == 0) { }  /* COUNTFLAG */
    }
}

static void pin_mode_output(const Pin *p)
{
    volatile uint32_t *moder = (volatile uint32_t *)(p->base + 0x00u);
    uint32_t v = *moder;
    v &= ~(3u << (p->pin * 2u));
    v |=  (1u << (p->pin * 2u));
    *moder = v;
}

static void pin_write(const Pin *p, int on)
{
    volatile uint32_t *bsrr = (volatile uint32_t *)(p->base + 0x18u);
    *bsrr = on ? (1u << p->pin) : (1u << (p->pin + 16u));
}

static void all_write(int on)
{
    for (unsigned i = 0; i < PIN_COUNT; i++) pin_write(&kPins[i], on);
}

int main(void)
{
    RCC_AHB1ENR |= 0x1Fu;  /* GPIOA-E 时钟 */
    for (unsigned i = 0; i < PIN_COUNT; i++) {
        pin_mode_output(&kPins[i]);
        pin_write(&kPins[i], 0);
    }
    STK_LOAD = 16000u - 1u;  /* 1ms @16MHz */
    STK_VAL = 0;
    STK_CTRL = (1u << 2) | 1u;  /* CLKSOURCE=内核, ENABLE, 不开中断 */

    for (;;) {
        for (int k = 0; k < 3; k++) {  /* 轮次标记：齐闪 3 次 */
            all_write(1); delay_ms(150);
            all_write(0); delay_ms(150);
        }
        delay_ms(1000);  /* 标记后停顿 1 秒再开始计数 */
        for (unsigned i = 0; i < PIN_COUNT; i++) {
            pin_write(&kPins[i], 1);
            delay_ms(1000);
            pin_write(&kPins[i], 0);
            delay_ms(250);
        }
    }
    return 0;
}
