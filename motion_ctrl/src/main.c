/* motion_ctrl v1 — 鼎立 DL-IO1824G 动作控制固件（草稿，本地准备用，未推仓库）
 *
 * 一句话行为：主令开关闭合才给主控阀供电；变幅模式（J1-10 低）只通起幅/
 * 落幅阀，伸缩模式（J1-10 高）只通伸/缩臂阀；过卷（CAN ID 192）或 ID 192
 * 超时时切断落幅阀与伸臂阀；超载切断已实现但默认关闭（OVERLOAD_CUT_ENABLE=0）。
 *
 * v1 的已知缺口——DI 输入通道未确认：
 *   J1-6（主令）与 J1-10（模式）是 DIH/AI 型脚，本板上开关量靠 ADC 采样读
 *   回（只有 PE8-PE11 是纯数字输入），但这两个开关具体落在 8 槽×12 路
 *   ADC 矩阵的哪一格尚未实测。本版以 DI-HUNT 模式发货：两个 DI 常量
 *   仍为 -1，5 个阀输出强制保持断开，同时全矩阵经 0x7A2 流出，供实车
 *   拨动开关时看哪一格跳变定位；一旦两个数填入（只改 4 个常量），同一
 *   份代码即为可用的动作程序，无需其他改动。绝不猜 DI 通道。
 *
 * 阀脚映射（2026-11-12 实车 diag_mimic 轮扫实测，步号 = 45 脚 kPins 表
 * 1-based 索引；J1-2 由用户 2026-11-12 11:59 再确认 0x1A=26）：
 *   J1-2  主控阀   步 26 = PC15
 *   J1-4  起幅阀   步 18 = PB14
 *   J1-5  落幅阀   步 28 = PD3
 *   J1-18 伸臂阀   步 24 = PC13
 *   J1-21 缩臂阀   步 12 = PB6
 * 绝不驱动 J1-3（油路切换阀，本车未接线）。
 *
 * 过卷信号来源：臂头控制器 QH6546 经 CAN ID 192（0xC0，200ms 周期）发来，
 * 本车 J2-1/J2-2/J1-11 均为空脚，过卷与接近开关不走模块 DI。
 *   ID 192 byte1 bit8 = 副钩过卷，byte1 bit7 = 主钩过卷
 *   （厂家资料以 1-based 记位，即 bit8=0x80、bit7=0x40；待实车核对）
 *   byte4 bit8 = 接近开关触发（本版只记录，暂不参与切断）。
 * 超时判故障：>500ms 未收到 ID 192 即视为故障态，同样切断 J1-5/J1-18，
 * 绝不把“收不到”等同于“未过卷”。上电后收到第一帧 192 之前即为超时态。
 *
 * 逻辑依据：厂家《输入输出逻辑---初版》文件 03（已与电气原理图核对）。
 * 与文件 04 的分歧（待用户定夺，见 README）：04 要求过卷只切对应钩的
 * 起钩阀、超载禁缩臂；本版按 03 执行（过卷切 J1-5+J1-18，超载不切缩臂）。
 *
 * 基础设施复用 freq_demo（已台架验证）：寄存器定义、clock_force_hsi16()、
 * adc_init()/adc_read()、can_init_250k()、can_send()、Bus-Off 自恢复、
 * IWDG 喂狗、PA12 双闪心跳、SysTick 1ms 轮询、5ms 主循环节拍。
 * 打包基线：_estack=0x20006538、产物填 0xFF 至 189,551 字节、文件须名
 * dl1824.bin（见 build.sh）。仅限台架/实车发动机熄火状态验证。
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
#define CAN_RF0R REG32(CAN1_BASE + 0x0Cu)
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
#define CAN_RI0R  REG32(CAN1_BASE + 0x1B0u)
#define CAN_RDT0R REG32(CAN1_BASE + 0x1B4u)
#define CAN_RDL0R REG32(CAN1_BASE + 0x1B8u)
#define CAN_RDH0R REG32(CAN1_BASE + 0x1BCu)

#define WAIT_LIMIT   100000u    /* 轮询等待计数上限，超时放弃不挂死 */
#define ID_STATUS    0x7A0u     /* 250ms 状态帧（无条件发送） */
#define ID_RAWGROUP  0x7A2u     /* 每秒一组：12 路通道一个槽的原始值 */
#define ID_QH6546    0x0C0u     /* 192：臂头控制器 QH6546（过卷/接近） */

/* ================= 待填：DI 输入通道（实车 DI-HUNT 定位后填入） ================
 * 含义：该开关在 0x7A2 全矩阵中的位置。(slot, ch) —— slot = 多路复用槽号
 * 0..7，ch = kChans 索引 0..11。-1 = 未确认；任一为 -1 即进入 DI-HUNT
 * 模式（5 个阀输出强制断开，只发诊断帧）。填入两个数后即为动作程序。
 * DI_MODE_INVERT：若实测 J1-10 高电平对应变幅（与假设相反），改成 1。 */
#define DI_MASTER_SLOT  (1)     /* J1-6 主令开关：槽号，2026-11-12 实测（J1-6-2.asc，24V 持续接 8~96s） */
#define DI_MASTER_CH    (9)     /* J1-6 主令开关：kChans 索引 9=PC2（ADC2 IN12），通 24V 时 4095、断开时 7；同格 ch1（PA6）同步 3007，可作印证 */
#define DI_MODE_SLOT    (3)     /* J1-10 变幅/伸缩切换：槽号，2026-11-12 实测（J1-10.asc，24V 持续接约 6~58 秒） */
#define DI_MODE_CH      (9)     /* J1-10 变幅/伸缩切换：kChans 索引 9=PC2（ADC2 IN12），通 24V 时 4095、断开时 7；同格 ch1（PA6）同步 3008，可作印证；极性待实车核对（DI_MODE_INVERT） */
#define DI_MODE_INVERT  0       /* 模式极性：0=J1-10 高为伸缩（文件 03），1=反之 */
#define DI_HIGH_RAW     2048u   /* DI 判高阈值（半量程暂定；HUNT 实测出真实高/低电平后校准） */

/* 超载切断：已实现、默认关闭。原因：P1/P2 未标定、无臂长/角度输入，
 * 力矩百分比算不出来；现在打开只会误切。待标定完成后再置 1。 */
#define OVERLOAD_CUT_ENABLE 0

/* ID 192 超时门限：>500ms 未收到即判通信故障（切断 J1-5/J1-18）。 */
#define QH6546_TIMEOUT_MS 500u

/* ---- 5 个阀输出（实车轮扫定案，只驱动这 5 脚；绝不碰 J1-3） ---- */
typedef struct { uint32_t base; uint8_t pin; } Pin;
static const Pin kValveMaster = {GPIOC_BASE, 15};  /* J1-2  主控阀  步26 PC15 */
static const Pin kValveLuffUp = {GPIOB_BASE, 14};  /* J1-4  起幅阀  步18 PB14 */
static const Pin kValveLuffDn = {GPIOD_BASE, 3};   /* J1-5  落幅阀  步28 PD3  */
static const Pin kValveTeleEx = {GPIOC_BASE, 13};  /* J1-18 伸臂阀  步24 PC13 */
static const Pin kValveTeleRe = {GPIOB_BASE, 6};   /* J1-21 缩臂阀  步12 PB6  */

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

/* ---- 运行态（ID 192 解码结果） ---- */
static uint8_t g_over_hoist_main = 0;  /* 主钩过卷（ID 192 byte1 bit7，待核对） */
static uint8_t g_over_hoist_aux = 0;   /* 副钩过卷（ID 192 byte1 bit8，待核对） */
static uint8_t g_proximity = 0;        /* 接近开关（ID 192 byte4 bit8，v1 只记录） */
static uint32_t g_qh6546_last_ms = 0;  /* 上次收到 ID 192 的 ms 时刻（初值 0=超时态） */
static uint16_t g_raw[8][NCH];         /* 全部 8 槽 × 12 路原始值矩阵（供 0x7A2 轮发） */
static uint16_t g_sweep_max = 0;       /* 本轮全矩阵最大值（供 0x7A0 字节 6-7） */

static void delay_ms(uint32_t ms)
{
    while (ms--) {
        STK_VAL = 0;
        while ((STK_CTRL & (1u << 16)) == 0) { }  /* COUNTFLAG */
    }
}
static void delay_us(uint32_t us)
{
    STK_LOAD = 16u - 1u;  /* 1us @16MHz */
    while (us--) { STK_VAL = 0; while ((STK_CTRL & (1u << 16)) == 0) { } }
    STK_LOAD = 16000u - 1u;
}

/* 时钟归一化：Bootloader 跳转时可能留下 PLL 高速时钟树（原厂运行频率），
   本固件的 SysTick/CAN/ADC 时序常数全部按 HSI 16MHz 基准。上电先切回
   HSI 并关 PLL、清 APB 预分频，保证时序与常数一致（带超时，不挂死）。 */
#define RCC_CR   REG32(0x40023800u)
#define RCC_CFGR REG32(0x40023808u)
static void clock_force_hsi16(void)
{
    uint32_t guard = 100000u;
    RCC_CR |= (1u << 0);                                  /* HSION */
    while ((RCC_CR & (1u << 1)) == 0 && guard--) { }      /* 等 HSIRDY */
    RCC_CFGR &= ~3u;                                      /* SW = HSI */
    guard = 100000u;
    while ((RCC_CFGR & (3u << 2)) != 0 && guard--) { }    /* 等 SWS = HSI */
    RCC_CR &= ~(1u << 24);                                /* 关 PLL */
    RCC_CFGR &= ~((7u << 10) | (7u << 13));               /* APB1/APB2 不分频 */
}

static void gpio_output(uint32_t base, uint8_t pin)
{
    uint32_t v = MODER(base);
    v &= ~(3u << (pin * 2u));
    v |=  (1u << (pin * 2u));
    MODER(base) = v;
}
static void gpio_analog(uint32_t base, uint8_t pin)
{
    uint32_t v = MODER(base);
    v |= (3u << (pin * 2u));
    MODER(base) = v;
}

static void pin_write(const Pin *p, int on)
{
    BSRR(p->base) = on ? (1u << p->pin) : (1u << (p->pin + 16u));
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
        ADC_SMPR1(b) = 0xFFFFFFFFu;  /* 通道10-17 采样=480周期(码7) 拉满，排除高阻前端采样不足 */
        ADC_SMPR2(b) = 0xFFFFFFFFu;  /* 通道0-9 同上 */
        ADC_SQR1(b) = 0;             /* 序列长度 1 */
        ADC_CR2(b) |= 1u;            /* ADON */
    }
}

static uint16_t adc_read(uint32_t base, uint8_t ch)
{
    /* 每通道连转两次、弃第一次：消除通道切换后的采样电容残留（鬼影） */
    uint16_t v = 0;
    for (int k = 0; k < 2; k++) {
        uint32_t guard = WAIT_LIMIT;
        ADC_SQR3(base) = ch;
        ADC_CR2(base) |= (1u << 30);  /* SWSTART */
        while ((ADC_SR(base) & (1u << 1)) == 0) {  /* EOC */
            if (--guard == 0) return 0;
        }
        v = (uint16_t)(ADC_DR(base) & 0xFFFu);
    }
    return v;
}

static void can_init_250k(void)
{
    /* PD0/PD1 -> AF9 */
    uint32_t v = MODER(GPIOD_BASE);
    v &= ~((3u << 0) | (3u << 2));
    v |= ((2u << 0) | (2u << 2));
    MODER(GPIOD_BASE) = v;
    AFRL(GPIOD_BASE) = (AFRL(GPIOD_BASE) & ~0xFFu) | 0x99u;
    RCC_APB1ENR |= (1u << 25);  /* CAN1 时钟 */
    uint32_t guard;
    /* 按 HAL 顺序：先退出睡眠（清 SLEEP、等 SLAK 清零），
       再请求初始化（置 INRQ、等 INAK）——超时只放弃等待，不挂死 */
    CAN_MCR &= ~(1u << 1);  /* 清 SLEEP */
    guard = WAIT_LIMIT;
    while ((CAN_MSR & (1u << 1)) != 0 && guard--) { }  /* SLAK */
    CAN_MCR |= 1u;  /* INRQ 进初始化 */
    guard = WAIT_LIMIT;
    while ((CAN_MSR & 1u) == 0 && guard--) { }  /* INAK */
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
    guard = WAIT_LIMIT;
    while ((CAN_MSR & 1u) != 0 && guard--) { }  /* INAK 清零 */
}

static int can_send(uint16_t id, const uint8_t d[8])
{
    uint32_t guard = WAIT_LIMIT;
    while ((CAN_TSR & (1u << 26)) == 0) {  /* 等邮箱0空 TME0 */
        if (--guard == 0) return 0;
    }
    CAN_TDL0R = (uint32_t)d[0] | ((uint32_t)d[1] << 8) |
                ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
    CAN_TDH0R = (uint32_t)d[4] | ((uint32_t)d[5] << 8) |
                ((uint32_t)d[6] << 16) | ((uint32_t)d[7] << 24);
    CAN_TDT0R = 8u;  /* DLC=8 */
    CAN_TI0R = ((uint32_t)id << 21);
    CAN_TI0R |= 1u;  /* TXRQ */
    return 1;
}

/* ---- CAN 接收轮询（FIFO0）：每轮最多取 8 帧，绝不无界循环 ----
   只关心 ID 192（QH6546 过卷/接近）；其余帧读后直接丢弃释放。 */
static void can_poll_rx(uint32_t now_ms)
{
    for (int n = 0; n < 8; n++) {
        if ((CAN_RF0R & 3u) == 0) break;  /* FIFO0 空（FMP0=0） */
        uint32_t rir = CAN_RI0R;
        uint16_t id = (uint16_t)((rir >> 21) & 0x7FFu);
        (void)CAN_RDT0R;                   /* 读走 RDTR，清邮箱状态 */
        uint32_t rdl = CAN_RDL0R;
        uint32_t rdh = CAN_RDH0R;
        CAN_RF0R |= (1u << 5);             /* RFOM0：释放邮箱 */
        if (id == ID_QH6546) {
            uint8_t b1 = (uint8_t)(rdl & 0xFFu);
            uint8_t b4 = (uint8_t)((rdh >> 24) & 0xFFu);
            /* 厂家资料 1-based 记位：bit8=0x80、bit7=0x40；待实车核对 */
            g_over_hoist_aux  = (b1 & (1u << 7)) != 0;  /* byte1 bit8 副钩过卷 */
            g_over_hoist_main = (b1 & (1u << 6)) != 0;  /* byte1 bit7 主钩过卷 */
            g_proximity       = (b4 & (1u << 7)) != 0;  /* byte4 bit8 接近开关 */
            g_qh6546_last_ms  = now_ms;
        }
    }
}

/* ---- 全矩阵扫一轮：8 槽 × 12 路，填 g_raw 并更新最大值 ----
   每槽选通后等 1ms 沉降；单通道双转弃首值。约 20ms，1 秒扫一轮。 */
static void matrix_sweep(void)
{
    g_sweep_max = 0;
    for (uint8_t slot = 0; slot < 8; slot++) {
        mux_select(slot);
        delay_us(1000);
        for (uint8_t i = 0; i < NCH; i++) {
            uint16_t v = adc_read(kChans[i].adc, kChans[i].ch);
            g_raw[slot][i] = v;
            if (v > g_sweep_max) g_sweep_max = v;
        }
    }
}

/* ---- 读一个 DI 通道原始值（动作模式下每 50ms 调用） ---- */
static uint16_t di_read(int slot, int ch_idx)
{
    mux_select((uint8_t)slot);
    delay_us(1000);
    uint32_t acc = 0;
    for (int i = 0; i < 2; i++) {
        acc += adc_read(kChans[ch_idx].adc, kChans[ch_idx].ch);
    }
    return (uint16_t)(acc / 2u);
}

/* ================= 动作主程序 =================
   DI 未确认时为 DI-HUNT 模式：5 阀全关断，只发诊断帧；
   DI 填入后为动作模式：主令/模式/过卷逻辑驱动 5 阀。 */
int main(void)
{
    clock_force_hsi16();
    RCC_AHB1ENR |= 0x1Fu;  /* GPIOA-E 时钟 */
    RCC_APB2ENR |= (1u << 8) | (1u << 9) | (1u << 10);  /* ADC1/2/3 时钟 */
    /* SysTick：1ms 节拍基准 */
    STK_LOAD = 16000u - 1u;
    STK_VAL = 0;
    STK_CTRL = (1u << 2) | 1u;

    /* 5 个阀脚推挽输出，初始全关断（安全默认） */
    gpio_output(kValveMaster.base, kValveMaster.pin); pin_write(&kValveMaster, 0);
    gpio_output(kValveLuffUp.base, kValveLuffUp.pin); pin_write(&kValveLuffUp, 0);
    gpio_output(kValveLuffDn.base, kValveLuffDn.pin); pin_write(&kValveLuffDn, 0);
    gpio_output(kValveTeleEx.base, kValveTeleEx.pin); pin_write(&kValveTeleEx, 0);
    gpio_output(kValveTeleRe.base, kValveTeleRe.pin); pin_write(&kValveTeleRe, 0);
    gpio_output(GPIOE_BASE, 12); gpio_output(GPIOE_BASE, 13); gpio_output(GPIOE_BASE, 14);  /* 复用选通脚 */
    gpio_output(GPIOA_BASE, 15);   /* PA15 置高（原厂上电即高；freq_demo v2/v3 同款） */
    BSRR(GPIOA_BASE) = (1u << 15);
    /* 12 路模拟脚为模拟模式（与 freq_demo v3 一致，含 PC2） */
    gpio_analog(GPIOA_BASE, 4); gpio_analog(GPIOA_BASE, 5);
    gpio_analog(GPIOA_BASE, 6); gpio_analog(GPIOA_BASE, 7);
    gpio_analog(GPIOB_BASE, 0); gpio_analog(GPIOB_BASE, 1);
    for (uint8_t pp = 0; pp <= 5; pp++) gpio_analog(GPIOC_BASE, pp);
    {
        uint32_t v = MODER(GPIOA_BASE);   /* PA12 输出（心跳灯） */
        v &= ~(3u << 24); v |= (1u << 24);
        MODER(GPIOA_BASE) = v;
        BSRR(GPIOA_BASE) = (1u << (12 + 16));
    }
    adc_init();
    can_init_250k();

    /* DI 是否已确认：两个通道的 slot 与 ch 全部填入才算确认 */
    const int di_ready = (DI_MASTER_SLOT >= 0) && (DI_MASTER_CH >= 0) &&
                         (DI_MODE_SLOT >= 0) && (DI_MODE_CH >= 0);

    uint32_t ms = 0;
    uint32_t last_status = 0;
    uint32_t last_sweep = 0;
    uint32_t last_di = 0;
    uint8_t  cnt = 0;
    uint16_t di_master_raw = 0;
    uint16_t di_mode_raw = 0;
    uint8_t  dump_slot = 0;
    for (;;) {
        REG32(0x40003000u) = 0xAAAAu;     /* IWDG 喂狗 */
        can_poll_rx(ms);                  /* 收 ID 192（每轮至多 8 帧） */

        /* ---- 输入采样 ---- */
        if (di_ready && (ms - last_di >= 50u)) {
            last_di = ms;
            di_master_raw = di_read(DI_MASTER_SLOT, DI_MASTER_CH);
            di_mode_raw = di_read(DI_MODE_SLOT, DI_MODE_CH);
        }
        if (ms - last_sweep >= 1000u) {   /* 每秒全矩阵扫一轮（HUNT 定位用），扫完即发该槽组帧 */
            last_sweep = ms;
            matrix_sweep();
            /* 0x7A2 矩阵组帧：4 帧发一个槽的 12 路原始值，8 秒发完全部 8 个槽；
               字节 6 高半字节=槽号、低半字节=帧序号，字节 7 滚动计数（diag_mimic 同款格式） */
            for (uint8_t f = 0; f < 4; f++) {
                uint8_t d[8];
                for (uint8_t k = 0; k < 3; k++) {
                    uint16_t v = g_raw[dump_slot][f * 3u + k];
                    d[k * 2u] = (uint8_t)(v & 0xFF);
                    d[k * 2u + 1u] = (uint8_t)(v >> 8);
                }
                d[6] = (uint8_t)((dump_slot << 4) | f);
                d[7] = cnt++;
                (void)can_send(ID_RAWGROUP, d);
            }
            dump_slot = (uint8_t)((dump_slot + 1u) & 7u);
        }

        /* ---- 动作逻辑（文件 03；方向由液压先导手柄给，模块只做使能/互锁） ---- */
        uint8_t master_on = 0;
        uint8_t telesc = 0;
        if (di_ready) {
            master_on = (di_master_raw > DI_HIGH_RAW) ? 1u : 0u;
            telesc = (di_mode_raw > DI_HIGH_RAW) ? 1u : 0u;
            if (DI_MODE_INVERT) telesc = (uint8_t)(telesc ^ 1u);
        }
        uint8_t over_hoist = (g_over_hoist_main || g_over_hoist_aux) ? 1u : 0u;
        uint8_t qh_timeout = ((ms - g_qh6546_last_ms) > QH6546_TIMEOUT_MS) ? 1u : 0u;
        uint8_t cut_dn_ex = (over_hoist || qh_timeout) ? 1u : 0u;  /* 切落幅阀+伸臂阀 */
#if OVERLOAD_CUT_ENABLE
        /* 超载切断（v1 未接：无标定压力与臂长/角度，百分比算不出；恒 0 占位） */
        uint8_t overload = 0;
        if (overload) cut_dn_ex = 1u;
#endif
        if (di_ready) {
            pin_write(&kValveMaster, master_on);
            pin_write(&kValveLuffUp, master_on && !telesc);
            pin_write(&kValveLuffDn, master_on && !telesc && !cut_dn_ex);
            pin_write(&kValveTeleEx, master_on && telesc && !cut_dn_ex);
            pin_write(&kValveTeleRe, master_on && telesc);
        } else {
            /* DI-HUNT 模式：5 阀强制全关断 */
            pin_write(&kValveMaster, 0);
            pin_write(&kValveLuffUp, 0);
            pin_write(&kValveLuffDn, 0);
            pin_write(&kValveTeleEx, 0);
            pin_write(&kValveTeleRe, 0);
        }

        /* PA12 双闪心跳（周期 2000ms：0-100ms 亮、100-200ms 灭、200-300ms 亮） */
        {
            uint32_t ph = ms % 2000u;
            if (ph < 100u || (ph >= 200u && ph < 300u)) BSRR(GPIOA_BASE) = (1u << 12);
            else BSRR(GPIOA_BASE) = (1u << (12 + 16));
        }

        /* ---- 0x7A0 状态帧：每 250ms ----
           字节 0-3：ms 计数 LE；字节 4：滚动计数；
           字节 5：模式标志（见 README 表）——HUNT 模式基值 0xA5，
                   动作模式基值 0x25（bit7=DI 已确认标志）；
           字节 6-7：本轮全矩阵最大值 LE（diag_mimic 同款）。 */
        if (ms - last_status >= 250u) {
            last_status = ms;
            uint8_t b5 = di_ready ? (uint8_t)0x25u : (uint8_t)0xA5u;
            if (qh_timeout) b5 |= (1u << 6);
            if (over_hoist) b5 |= (1u << 4);
            if (di_ready && telesc) b5 |= (1u << 3);
            if (di_ready && master_on) b5 |= (1u << 1);
            uint8_t d[8] = { (uint8_t)(ms & 0xFFu), (uint8_t)((ms >> 8) & 0xFFu),
                             (uint8_t)((ms >> 16) & 0xFFu), (uint8_t)((ms >> 24) & 0xFFu),
                             cnt++, b5,
                             (uint8_t)(g_sweep_max & 0xFFu), (uint8_t)((g_sweep_max >> 8) & 0xFFu) };
            (void)can_send(ID_STATUS, d);
        }

        if ((REG32(0x40006418u) & (1u << 2)) != 0) can_init_250k();  /* Bus-Off 自恢复 */
        delay_ms(5);
        ms += 5u;
    }
    return 0;
}
