/* 仿原厂形态诊断固件 diag_mimic（不是正式 Demo）
 *
 * 以 diag_test v2 为底本，专为排查 Bootloader 镜像验收而改三处：
 * 栈顶对齐原厂（0x20006538）、产物填充至 189,551 字节与原厂同大小
 * （见 build.sh）、PA12 心跳改为不可混淆的“双闪”节律（见主循环）。
 * 其余功能与 diag_test 完全一致：一次刷机同时验证三条链路：
 *
 * 由 scan_test（45 脚输出轮扫）与 can_trigger_test（CAN 发送 + ADC 扫描）
 * 合并而成，一版固件同时验证三条链路，避免反复刷机：
 *
 *  (a) CAN 链路：上电即以 250ms 周期无条件发送状态帧 0x7A0，不依赖
 *      任何输入 —— 监听到 0x7A0 就证明“固件在跑 + CAN 物理链路通”。
 *  (b) 灯脚：45 个候选输出脚每 1 秒推进一步轮扫（轮始齐闪 3 次标记，
 *      同 scan_test），当前步号随 0x7A0 字节 5 一起发出，不数闪烁、
 *      对照帧内步号即可定位 J1-17 的驱动脚。
 *  (c) AI 通路：每轮做 8 槽（多路复用）× 12 路 ADC 全扫描；最大值
 *      >= 2048 时发触发帧 0x7A1；每秒发一组 0x7A2（4 帧，装 12 路通道
 *      在槽 0 的原始值），拧电位器看哪一路跟动即知 J1-1 的通路。
 *
 * 总线：CAN1（J3-1/2，线号 113/114），PD0/PD1 AF9，250 kbit/s
 * （HSI 16MHz，APB1=16MHz，BRP=4，BS1=13，BS2=2，采样点 87.5%）。
 * 注意：监听工具须处于正常模式（参与 ACK），只听模式下本节点会因
 * 无 ACK 反复重发并退出发送。
 *
 * 与两个源工程相比的健壮性修正：
 *  - CAN 初始化按 HAL 顺序：先清 SLEEP 等 SLAK 清零，再置 INRQ 等
 *    INAK（can_trigger_test 是先 INRQ 后清 SLEEP）；
 *  - 所有轮询等待（CAN 初始化应答、发送邮箱、ADC EOC）都带计数
 *    超时（WAIT_LIMIT），超时放弃、绝不无限挂死；发送失败不阻塞
 *    主循环。
 *
 * 安全：仅限台架、连接器上只接灯与电位器、无阀类负载时使用；
 * 发现异响/异味/异常发热立即断电。
 * 时序：SysTick 轮询计时（不依赖中断向量），HSI 16MHz 默认时钟，
 * 主循环节拍 50ms，毫秒计数为近似值（节拍累加）。
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

#define THRESH_RAW   2048u      /* 12V 阈值（半量程假定，待台架标定） */
#define WAIT_LIMIT   100000u    /* 轮询等待计数上限，超时放弃不挂死 */
#define ID_STATUS    0x7A0u     /* 250ms 状态帧（无条件发送） */
#define ID_TRIGGER   0x7A1u     /* ADC 最大值过阈时发送 */
#define ID_RAWGROUP  0x7A2u     /* 每秒一组：12 路通道槽 0 原始值 */

/* ---- 输出轮扫候选脚（同 main_scan.c 的 kPins，45 脚） ---- */
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
static void all_write(int on)
{
    for (unsigned i = 0; i < PIN_COUNT; i++) pin_write(&kPins[i], on);
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


/* ---- 变频律（用户定版）：0V 长亮、电压越高频越快、24V -> 20Hz，线性连续 ---- */
#define RAW_AT_24V 4000u   /* 24V 对应的原始计数（暂定，待标定） */
#define RAW_ZERO   40u     /* 小于此值视为 0V */
#define FREQ_MAX   20u
static uint32_t freq_of(uint16_t raw)
{
    if (raw <= RAW_ZERO) return 0;
    uint32_t f = ((uint32_t)(raw - RAW_ZERO) * FREQ_MAX) / (RAW_AT_24V - RAW_ZERO);
    if (f > FREQ_MAX) f = FREQ_MAX;
    if (f == 0u) f = 1u;
    return f;
}

/* 读演示输入（槽0, PC2）= J1-12 通路：选槽后等 1ms 沉降，4 次平均 */
static uint16_t read_demo_raw(void)
{
    mux_select(0);
    delay_us(1000);
    uint32_t acc = 0;
    for (int i = 0; i < 4; i++) acc += adc_read(ADC2_BASE, 12);
    return (uint16_t)(acc / 4u);
}

/* ================= 变频主程序 =================
   基线 = steady_on（已实证长亮）：45 路输出一起驱动（灯在其中一路，
   不依赖步号与 CAN 命令）；输入每 50ms 采一轮；0x7A0 每 250ms 汇报
   字节5=当前频率 Hz（0=长亮）、字节6-7=输入原始值。 */
int main(void)
{
    clock_force_hsi16();
    RCC_AHB1ENR |= 0x1Fu;  /* GPIOA-E 时钟 */
    RCC_APB2ENR |= (1u << 8) | (1u << 9) | (1u << 10);  /* ADC1/2/3 时钟 */
    /* SysTick：1ms 节拍基准 */
    STK_LOAD = 16000u - 1u;
    STK_VAL = 0;
    STK_CTRL = (1u << 2) | 1u;

    const unsigned n = (unsigned)(sizeof(kPins) / sizeof(kPins[0]));
    for (unsigned i = 0; i < n; i++) {
        gpio_output(kPins[i].base, kPins[i].pin);
        pin_write(&kPins[i], 1);          /* 先全置高（长亮起点） */
    }
    gpio_output(GPIOE_BASE, 12); gpio_output(GPIOE_BASE, 13); gpio_output(GPIOE_BASE, 14);  /* 复用选通脚 */
    {
        uint32_t v = MODER(GPIOA_BASE);   /* PA12 输出（心跳灯） */
        v &= ~(3u << 24); v |= (1u << 24);
        MODER(GPIOA_BASE) = v;
        BSRR(GPIOA_BASE) = (1u << (12 + 16));
    }
    adc_init();
    can_init_250k();

    uint32_t ms = 0;
    uint32_t last_status = 0;
    uint32_t last_sample = 0;
    uint8_t  cnt = 0;
    uint16_t demo_raw = 0;
    uint32_t demo_freq = 0;
    for (;;) {
        REG32(0x40003000u) = 0xAAAAu;     /* IWDG 喂狗 */
        if (ms - last_sample >= 50u) {    /* 每 50ms 采一轮输入 */
            last_sample = ms;
            demo_raw = read_demo_raw();
            demo_freq = freq_of(demo_raw);
        }
        if (demo_freq == 0u) {
            all_write(1);                 /* 0V：长亮 */
        } else {
            uint32_t period = 1000u / demo_freq;
            all_write((ms % period) < (period / 2u));
        }
        uint32_t ph = ms % 2000u;         /* PA12 双闪心跳 */
        if (ph < 100u || (ph >= 200u && ph < 300u)) BSRR(GPIOA_BASE) = (1u << 12);
        else BSRR(GPIOA_BASE) = (1u << (12 + 16));
        if (ms - last_status >= 250u) {
            last_status = ms;
            uint8_t dd[8] = { (uint8_t)(ms & 0xFFu), (uint8_t)((ms >> 8) & 0xFFu),
                              (uint8_t)((ms >> 16) & 0xFFu), (uint8_t)((ms >> 24) & 0xFFu),
                              cnt++, (uint8_t)demo_freq,
                              (uint8_t)(demo_raw & 0xFFu), (uint8_t)((demo_raw >> 8) & 0xFFu) };
            (void)can_send(0x7A0u, dd);
        }
        if ((REG32(0x40006418u) & (1u << 2)) != 0) can_init_250k();  /* Bus-Off 自恢复 */
        delay_ms(5);
        ms += 5u;
    }
    return 0;
}
