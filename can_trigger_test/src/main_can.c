/* AI 触发 CAN 发送测试固件（台架验证用，不是正式 Demo）
 *
 * 行为：
 *  - 持续全量扫描模拟输入：外部多路复用 8 个槽（PE12-14 选通，编码=槽号
 *    比特反序）× 12 路 ADC 通道（原厂已恢复的 10 路 + PA4/PA5），每轮
 *    取最大值及其位置；
 *  - 最大值 >= THRESH_RAW（默认 2048 = 半量程，对应台架刻度 12V，
 *    按“连接器 24V → ADC 满量程”假定）时，以 50ms 周期持续发送
 *    触发帧 ID 0x7A1，直到信号回落；
 *  - 未触发时每 1 秒发一帧心跳 ID 0x7A0（便于先确认监听链路），
 *    不需要心跳可把 ENABLE_HEARTBEAT 改 0。
 *
 * 顺带定位：触发帧带回最大值所在的 (槽号, ADC通道)，拧电位器时跟动
 * 的那一路即 J1-1 的真实通路；帧内原始计数可用于标定准确的 12V 阈值。
 *
 * 总线：CAN1（J3-1/2，线号 113/114），PD0/PD1 AF9，250 kbit/s
 * （HSI 16MHz，APB1=16MHz，BRP=4，BS1=13，BS2=2，采样点 87.5%）。
 * 注意：监听工具须处于正常模式（参与 ACK），只听模式下本节点会因
 * 无 ACK 反复重发并退出发送。
 */
#include <stdint.h>

#define REG32(a) (*(volatile uint32_t *)(a))
/* GPIO */
#define GPIOA_BASE 0x40020000u
#define GPIOB_BASE 0x40020400u
#define GPIOC_BASE 0x40020800u
#define GPIOD_BASE 0x40020C00u
#define GPIOE_BASE 0x40021000u
#define MODER(b)  REG32((b) + 0x00u)
#define ODR(b)    REG32((b) + 0x14u)
#define BSRR(b)   REG32((b) + 0x18u)
#define AFRL(b)   REG32((b) + 0x20u)
#define AFRH(b)   REG32((b) + 0x24u)
/* RCC */
#define RCC_AHB1ENR REG32(0x40023830u)
#define RCC_APB1ENR REG32(0x40023840u)
#define RCC_APB2ENR REG32(0x40023844u)
/* SysTick */
#define STK_CTRL REG32(0xE000E010u)
#define STK_LOAD REG32(0xE000E014u)
#define STK_VAL  REG32(0xE000E018u)
/* ADC */
#define ADC1_BASE 0x40012000u
#define ADC2_BASE 0x40012100u
#define ADC3_BASE 0x40012200u
#define ADC_CCR   REG32(0x40012304u)
#define ADC_SR(b)   REG32((b) + 0x00u)
#define ADC_CR1(b)  REG32((b) + 0x04u)
#define ADC_CR2(b)  REG32((b) + 0x08u)
#define ADC_SMPR1(b) REG32((b) + 0x0Cu)
#define ADC_SMPR2(b) REG32((b) + 0x10u)
#define ADC_SQR1(b) REG32((b) + 0x2Cu)
#define ADC_SQR3(b) REG32((b) + 0x34u)
#define ADC_DR(b)   REG32((b) + 0x4Cu)
/* CAN1 */
#define CAN1_BASE 0x40006400u
#define CAN_MCR  REG32(CAN1_BASE + 0x00u)
#define CAN_MSR  REG32(CAN1_BASE + 0x04u)
#define CAN_TSR  REG32(CAN1_BASE + 0x08u)
#define CAN_BTR  REG32(CAN1_BASE + 0x1Cu)
#define CAN_FMR  REG32(CAN1_BASE + 0x200u)
#define CAN_FM1R REG32(CAN1_BASE + 0x204u)
#define CAN_FS1R REG32(CAN1_BASE + 0x20Cu)
#define CAN_FFA1R REG32(CAN1_BASE + 0x214u)
#define CAN_FA1R REG32(CAN1_BASE + 0x21Cu)
#define CAN_F0R1 REG32(CAN1_BASE + 0x240u)
#define CAN_F0R2 REG32(CAN1_BASE + 0x244u)
#define CAN_TI0R  REG32(CAN1_BASE + 0x180u)
#define CAN_TDT0R REG32(CAN1_BASE + 0x184u)
#define CAN_TDL0R REG32(CAN1_BASE + 0x188u)
#define CAN_TDH0R REG32(CAN1_BASE + 0x18Cu)

#define THRESH_RAW       2048u   /* 12V 阈值（半量程假定，待台架标定） */
#define ENABLE_HEARTBEAT 1
#define ID_TRIGGER 0x7A1u
#define ID_HEARTBEAT 0x7A0u

/* PE12-14 选槽编码：槽号 3 位比特反序（已从原厂固件解码定案） */
static const uint8_t kMuxCode[8] = {0, 4, 2, 6, 1, 5, 3, 7};

typedef struct { uint32_t adc; uint8_t ch; uint8_t code; } AdcChan;
/* code 编码：(ADC序号<<4)|通道号，ADC1=0x0、ADC2=0x1、ADC3=0x2 */
static const AdcChan kChans[] = {
    {ADC1_BASE, 13, 0x0D}, {ADC1_BASE, 6, 0x06}, {ADC1_BASE, 15, 0x0F},
    {ADC1_BASE, 8, 0x08},  {ADC1_BASE, 9, 0x09}, {ADC1_BASE, 7, 0x07},
    {ADC1_BASE, 4, 0x04},  {ADC1_BASE, 5, 0x05},   /* PA4/PA5 未用对 */
    {ADC2_BASE, 14, 0x1E}, {ADC2_BASE, 12, 0x1C},  /* PC4 / PC2(压力) */
    {ADC3_BASE, 11, 0x2B}, {ADC3_BASE, 10, 0x2A},
};
#define NCH (sizeof(kChans) / sizeof(kChans[0]))

static void delay_ms(uint32_t ms)
{
    while (ms--) { STK_VAL = 0; while ((STK_CTRL & (1u << 16)) == 0) { } }
}
static void delay_us(uint32_t us)
{
    STK_LOAD = 16u - 1u;  /* 1us @16MHz */
    while (us--) { STK_VAL = 0; while ((STK_CTRL & (1u << 16)) == 0) { } }
    STK_LOAD = 16000u - 1u;
}

static void gpio_analog(uint32_t base, uint8_t pin)
{
    uint32_t v = MODER(base);
    v |= (3u << (pin * 2u));
    MODER(base) = v;
}
static void gpio_output(uint32_t base, uint8_t pin)
{
    uint32_t v = MODER(base);
    v &= ~(3u << (pin * 2u));
    v |= (1u << (pin * 2u));
    MODER(base) = v;
}

static void mux_select(uint8_t slot)
{
    uint32_t odr = ODR(GPIOE_BASE);
    odr &= ~(7u << 12);
    odr |= ((uint32_t)kMuxCode[slot] << 12);
    ODR(GPIOE_BASE) = odr;
}

static void adc_init(void)
{
    ADC_CCR &= ~(3u << 16);  /* ADCPRE /2 -> ADCCLK 8MHz */
    const uint32_t adcs[3] = {ADC1_BASE, ADC2_BASE, ADC3_BASE};
    for (int i = 0; i < 3; i++) {
        uint32_t b = adcs[i];
        ADC_SMPR1(b) = 0x24924924u;  /* 通道10-17 采样=56周期(码3) 近似铺满 */
        ADC_SMPR2(b) = 0x24924924u;  /* 通道0-9 同上 */
        ADC_SQR1(b) = 0;             /* 序列长度 1 */
        ADC_CR2(b) |= 1u;            /* ADON */
    }
}

static uint16_t adc_read(uint32_t base, uint8_t ch)
{
    ADC_SQR3(base) = ch;
    ADC_CR2(base) |= (1u << 30);  /* SWSTART */
    while ((ADC_SR(base) & (1u << 1)) == 0) { }  /* EOC */
    return (uint16_t)(ADC_DR(base) & 0xFFFu);
}

static void can_init_250k(void)
{
    /* PD0/PD1 -> AF9 */
    uint32_t v = MODER(GPIOD_BASE);
    v &= ~((3u << 0) | (3u << 2));
    v |= ((2u << 0) | (2u << 2));
    MODER(GPIOD_BASE) = v;
    AFRH(GPIOD_BASE) &= ~0xFFu;  /* 注：PD0/PD1 在 AFRL，见下行 */
    AFRL(GPIOD_BASE) = (AFRL(GPIOD_BASE) & ~0xFFu) | 0x99u;
    RCC_APB1ENR |= (1u << 25);  /* CAN1 时钟 */
    CAN_MCR |= 1u;  /* INRQ 进初始化 */
    while ((CAN_MSR & 1u) == 0) { }
    CAN_MCR &= ~(1u << 1);  /* 退出睡眠 */
    CAN_BTR = (3u << 24) | (1u << 20) | (12u << 16) | (3u);  /* SJW1,BS2=2,BS1=13,BRP=4 */
    /* 过滤器 0：全接收（仅为让控制器处于正常收发状态） */
    CAN_FMR |= 1u;
    CAN_FM1R &= ~1u;      /* 掩码模式 */
    CAN_FS1R |= 1u;       /* 32 位 */
    CAN_FFA1R &= ~1u;     /* FIFO0 */
    CAN_F0R1 = 0; CAN_F0R2 = 0;
    CAN_FA1R |= 1u;
    CAN_FMR &= ~1u;
    CAN_MCR &= ~1u;  /* 退出初始化 */
    while ((CAN_MSR & 1u) != 0) { }
}

static void can_send(uint16_t id, const uint8_t d[8])
{
    while ((CAN_TSR & (1u << 26)) == 0) { }  /* 等邮箱0空 */
    CAN_TDL0R = (uint32_t)d[0] | ((uint32_t)d[1] << 8) |
                ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
    CAN_TDH0R = (uint32_t)d[4] | ((uint32_t)d[5] << 8) |
                ((uint32_t)d[6] << 16) | ((uint32_t)d[7] << 24);
    CAN_TDT0R = 8u;  /* DLC=8 */
    CAN_TI0R = ((uint32_t)id << 21);
    CAN_TI0R |= 1u;  /* TXRQ */
}

typedef struct { uint16_t max; uint8_t slot; uint8_t code; uint16_t over; } Sweep;
static Sweep sweep_all(void)
{
    Sweep s = {0, 0, 0, 0};
    for (uint8_t slot = 0; slot < 8; slot++) {
        mux_select(slot);
        delay_us(100);  /* 多路复用切换稳定时间 */
        for (unsigned i = 0; i < NCH; i++) {
            uint16_t v = adc_read(kChans[i].adc, kChans[i].ch);
            if (v > s.max) { s.max = v; s.slot = slot; s.code = kChans[i].code; }
            if (v >= THRESH_RAW) s.over++;
        }
    }
    return s;
}

int main(void)
{
    RCC_AHB1ENR |= 0x1Fu | (1u << 7);   /* GPIOA-E + GPIOH 口时钟位无害 */
    RCC_APB2ENR |= (1u << 8) | (1u << 9) | (1u << 10);  /* ADC1/2/3 */
    /* 12 路模拟脚 */
    gpio_analog(GPIOA_BASE, 4); gpio_analog(GPIOA_BASE, 5);
    gpio_analog(GPIOA_BASE, 6); gpio_analog(GPIOA_BASE, 7);
    gpio_analog(GPIOB_BASE, 0); gpio_analog(GPIOB_BASE, 1);
    for (uint8_t p = 0; p <= 5; p++) gpio_analog(GPIOC_BASE, p);
    /* 多路复用选通脚 */
    gpio_output(GPIOE_BASE, 12); gpio_output(GPIOE_BASE, 13); gpio_output(GPIOE_BASE, 14);
    STK_LOAD = 16000u - 1u;
    STK_VAL = 0;
    STK_CTRL = (1u << 2) | 1u;
    adc_init();
    can_init_250k();

    uint8_t counter = 0;
    uint32_t hb_ms = 0;
    for (;;) {
        Sweep s = sweep_all();
        uint16_t volt_x100 = (uint16_t)(((uint32_t)s.max * 2400u) / 4095u);
        if (s.max >= THRESH_RAW) {
            uint8_t d[8];
            d[0] = (uint8_t)(s.max & 0xFF); d[1] = (uint8_t)(s.max >> 8);
            d[2] = s.slot; d[3] = s.code;
            d[4] = (uint8_t)(s.over & 0xFF);
            d[5] = counter++;
            d[6] = (uint8_t)(volt_x100 & 0xFF); d[7] = (uint8_t)(volt_x100 >> 8);
            can_send(ID_TRIGGER, d);
            delay_ms(50);
            hb_ms = 0;
        } else if (ENABLE_HEARTBEAT) {
            if (hb_ms == 0) {
                uint8_t d[8] = {0};
                d[0] = (uint8_t)(s.max & 0xFF); d[1] = (uint8_t)(s.max >> 8);
                d[2] = s.slot; d[3] = s.code;
                d[5] = counter++;
                d[6] = (uint8_t)(volt_x100 & 0xFF); d[7] = (uint8_t)(volt_x100 >> 8);
                can_send(ID_HEARTBEAT, d);
            }
            delay_ms(20);
            hb_ms += 20;
            if (hb_ms >= 1000) hb_ms = 0;
        } else {
            delay_ms(20);
        }
    }
    return 0;
}
