/* motion_ctrl v2.1 — 鼎立 DL-IO1824G 原厂逻辑版（v2 + 全输出显式置低）
 *
 * v2.1 相对 v2 的唯一行为变化：45 个 DO 脚每轮先全部置低，再按逻辑写
 * 5 个阀——其余 40 脚（卷扬/配重/推杆/油冷/空调/蜂鸣器等）不再悬空，
 * 排除“未驱动脚默认导通、把先导油偷走”这一干扰。针对 2026-11-12 实车
 * 现象：伸缩模式下手柄左推走副钩卷扬、伸缩不动（2.asc），而原厂同操作
 * 伸缩正常（2-2.asc，臂长 12.88→15.86→11.39m）。若 v2.1 伸缩仍不动，
 * 则伸缩阀 GPIO 脚认定（PC13/PB6）有误，用 tele_hunt 找真脚（见
 * tele_hunt/README.md），勿再猜脚。
 *
 * 一句话行为：v1.1 的 5 阀动作逻辑按文件 03 字面执行（主令只门控主控阀
 * J1-2；变幅/伸缩阀按模式常通，先导选方向；过卷 0x192 或超时切断落幅
 * 与伸臂阀；超载切断仍关闭），并增加原厂通讯骨架——NMT 状态机（0x001
 * 启动、0x708 心跳、8 秒主机心跳超时只报警不锁阀）、最小 SDO 服务
 * （0x608/0x588，厂家 0x43 读 / 0x2F-0x23 写方言）、100ms PDO（0x198
 * 臂长、0x1A8 角度、0x1B8 压力、0x1F8 IO 状态、0x298 吊重、0x2A8 幅度
 * 高度、0x2D8 故障位图），使 DIS101A 屏像认原厂一样认出本固件。
 * 0x7A0/0x7A2 自研诊断帧保留，供台架对照。
 *
 * 依据（2026-11-12 采集）：
 *   文件 03《输入输出逻辑---初版》58 行（阀逻辑字面为准）；
 *   厂家手册 V1.3.0（NMT 0x001、0x708/0x718 心跳、SDO 0x43/0x60 方言、
 *     0x2002 PDO 周期 n×10ms 默认 100ms）；
 *   upload-3.asc 原厂基线（0x708=[05 00 03] 运行态；0x1F8 位图；
 *     0x298 额定/显示/百分比×100/修正前；0x192 恒为 02 00 00 00 01 00
 *     02 00；屏每 ~2.5s 读 SDO 0x2010，屏不发 0x718 主机心跳）。
 *
 * v2 相对 v1.1 的唯一行为变化（除新增通讯外）：变幅/伸缩阀不再以主令
 * 为门控——文件 03 字面是“默认得电/失电”且无主令条件，v1 的主令门控
 * 属偏离原厂；主控阀 J1-2 仍由主令门控，无油压时阀得电亦不动。
 * 明天台架对照原厂验证：主令断开+变幅模式时 J1-4 是否 24V。
 *
 * 未做（待定，不猜）：
 *   卷扬/推杆/蜂鸣器/配重/油冷/空调输出脚步号未测（只确认 J1-2/4/5/
 *   18/21 五脚），一律不驱动；J1-7/J1-8/J1-14/J1-15/J2-12/J2-13 的
 *   DI 格子未定位，强制/超载解除逻辑未接；臂长/角度来源 record 11/12
 *   未定（0x392/0x492 已排除），PDO 里发 0；P1/P2 压力换算用假定
 *   （250Ω 采样、PT400 量程 0~400bar、4mA 活零点），台架核对；
 *   0x1B8 bytes4-7 未识别（基线恒为 596/36），v2 发 0；参数不存
 *   Flash（SDO 0x1010 只应答），掉电丢失；扩展 TPDO（0x1C8/0x1D8/
 *   0x2B8/0x2C8/0x388…0x4D8）暂不发送。
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
 * 过卷信号来源：臂头控制器 QH6546 经 CAN ID 0x192（40ms 周期）发来，
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
#define ID_QH6546    0x192u     /* 0x192：臂头控制器 QH6546（过卷/接近），实车总线核实 2026-11-12：40ms 周期；v1 误作 0xC0 全程超时 */

/* ---- 原厂通讯：节点与帧 ID（厂家手册 V1.3.0 + upload-3 基线） ---- */
#define NODE_ID        8u        /* 本节点 nodeID（SDO 0x2001 可改，v2 只存不换） */
#define ID_NMT         0x001u    /* 主机→节点：NMT 管理帧（DLC 2：cmd, nodeID） */
#define ID_HOST_HB     0x718u    /* 主机心跳 0x710+nodeID，1 字节 [0x01]，1000ms */
#define ID_NODE_HB     0x708u    /* 本机心跳 0x700+nodeID，3 字节 [status, com, hw_id=0x03] */
#define ID_SDO_RX      0x608u    /* SDO 请求 0x600+nodeID */
#define ID_SDO_TX      0x588u    /* SDO 回应 0x580+nodeID */
#define ID_PDO_LEN     0x198u    /* 臂长 0x190+nodeID（0.01m） */
#define ID_PDO_ANG     0x1A8u    /* 角度 0x1A0+nodeID（0.01°） */
#define ID_PDO_PRES    0x1B8u    /* 压力 0x1B0+nodeID（0.01MPa） */
#define ID_PDO_IO      0x1F8u    /* IO 状态 0x1F0+nodeID */
#define ID_PDO_WGT     0x298u    /* 吊重 0x290+nodeID（0.01t / 百分比×100） */
#define ID_PDO_AMP     0x2A8u    /* 幅度高度 0x2A0+nodeID（0.01m） */
#define ID_PDO_FLT     0x2D8u    /* 故障位图 0x2D0+nodeID（自研位定义，见下） */
#define HOST_HB_TIMEOUT_MS 8000u /* 8 秒无主机心跳判通信故障（手册），只报警不锁阀 */
#define PDO_PERIOD_DEFAULT 10u   /* 0x2002 默认值 10（×10ms = 100ms） */

/* NMT 状态（0x708 byte0；手册值） */
#define NMT_INIT  0x00u
#define NMT_STOP  0x04u
#define NMT_OPER  0x05u
#define NMT_PREOP 0x7Fu

/* 0x2D8 故障位图（自研定义；厂家位图无文档，基线全 0） */
#define FLT_QH_TIMEOUT (1u << 0) /* ID 0x192 超时（>500ms） */
#define FLT_HOST_TIMEOUT (1u << 1) /* 主机心跳超时（>8s） */
#define FLT_P1_OPEN    (1u << 2) /* P1 断线（原始值 <400，约 <2mA） */
#define FLT_P2_OPEN    (1u << 3) /* P2 断线 */

/* P1/P2 压力换算（假定，待台架核对）：采样电阻约 250Ω（5V→4060 锚点），
 * PT400 量程 0~400bar，4mA 活零点对应原始值 819（4mA×250Ω=1V）。
 * 0.01MPa 单位值 = (raw-819)×4000/3276；raw<400 判断线。 */
#define P_RAW_ZERO  819u
#define P_RAW_OPEN  400u
#define P_RAW_FULL  4095u
#define P_BAR_FULL  400u

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

/* 其余开关 DI 格子（未定位，-1；定位方法与 J1-6/J1-10 相同：拨动开关看
 * 0x7A2 哪格跳变。定位前相关逻辑不接线、不驱动） */
#define DI_FORCE_SLOT   (-1)     /* J1-7 强制开关（解除高度限位/消音） */
#define DI_FORCE_CH     (-1)
#define DI_OVLKEY_SLOT  (-1)     /* J1-8 超载钥匙（解除超载限制） */
#define DI_OVLKEY_CH    (-1)
#define DI_CW_FWD_SLOT  (-1)     /* J1-15 配重前移开关 */
#define DI_CW_FWD_CH    (-1)
#define DI_CW_BWD_SLOT  (-1)     /* J1-14 配重后移开关 */
#define DI_CW_BWD_CH    (-1)
#define DI_PSH_EXT_SLOT (-1)     /* J2-13 推杆伸到位 */
#define DI_PSH_EXT_CH   (-1)
#define DI_PSH_RET_SLOT (-1)     /* J2-12 推杆缩到位 */
#define DI_PSH_RET_CH   (-1)

/* P1/P2 压力输入格子（原厂固件逆向定案：共用 PC2，P1=槽0、P2=槽6） */
#define P1_SLOT 0
#define P1_CH   9
#define P2_SLOT 6
#define P2_CH   9

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

/* ---- 全部 45 个 DO 候选脚（同 diag_mimic 的 kPins；v2.1 起全部显式置低，
      只留 5 个阀按逻辑驱动，杜绝未驱动脚悬空默认导通干扰） ---- */
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

/* ---- 运行态（ID 192 解码结果） ---- */
static uint8_t g_over_hoist_main = 0;  /* 主钩过卷（ID 192 byte1 bit7，待核对） */
static uint8_t g_over_hoist_aux = 0;   /* 副钩过卷（ID 192 byte1 bit8，待核对） */
static uint8_t g_proximity = 0;        /* 接近开关（ID 192 byte4 bit8，v1 只记录） */
static uint32_t g_qh6546_last_ms = 0;  /* 上次收到 ID 192 的 ms 时刻（初值 0=超时态） */
static uint16_t g_raw[8][NCH];         /* 全部 8 槽 × 12 路原始值矩阵（供 0x7A2 轮发） */
static uint16_t g_sweep_max = 0;       /* 本轮全矩阵最大值（供 0x7A0 字节 6-7） */

/* ---- 原厂通讯运行态 ---- */
static uint8_t  g_nmt_state = NMT_INIT;   /* 上电先 INIT（发一次 0x00 心跳）后转 PREOP */
static uint8_t  g_com_ok = 0;             /* 主机心跳 8 秒内收到过（0x708 byte1） */
static uint32_t g_host_hb_last_ms = 0;    /* 上次收到 0x718 的 ms 时刻 */
static uint8_t  g_hb_init_sent = 0;       /* INIT 心跳已发过 */
static uint8_t  g_hb_stop_sent = 0;       /* STOP 心跳已发过 */
static uint8_t  g_pdo_period = PDO_PERIOD_DEFAULT; /* 0x2002：n×10ms，默认 10=100ms */
static uint32_t g_duty = 0x20u;           /* SDO 0x2010 工况（upload-3 实测屏读回值 0x20） */
static uint32_t g_struct = 0u;            /* SDO 0x2011 结构参数（v2 只存不应用） */
static uint32_t g_vehicle = 0x22u;        /* SDO 0x2022 车型（0x22=JTC12H6B；本车 JTC10H6B-1JC 实测为 0x22） */
static uint32_t g_boom_sec = 0u;          /* SDO 0x2028 臂节（0=五节臂，1=六节臂；默认 0，待定） */
static uint32_t g_baud = 3u;              /* SDO 0x2000 波特率（3=250K；v2 只存不换） */
static uint32_t g_nodeid = NODE_ID;       /* SDO 0x2001 节点号（v2 只存不换） */
static uint16_t g_p1_raw = 0;             /* P1 大腔原始值（slot0/ch9） */
static uint16_t g_p2_raw = 0;             /* P2 小腔原始值（slot6/ch9） */
static uint16_t g_p1_001mpa = 0;          /* P1 换算值（0.01MPa，假定换算） */
static uint16_t g_p2_001mpa = 0;          /* P2 换算值（0.01MPa，假定换算） */
static uint8_t  g_fault = 0;              /* 0x2D8 故障位图（自研位定义） */
static uint8_t  g_io_b0 = 0x80u;          /* 0x1F8 byte0：bit7=1（基线常量），bit0=主令输入 */
static uint8_t  g_io_b4 = 0x04u;          /* 0x1F8 byte4：bit2=1（基线常量），bit0=主控阀输出，bit4=模式 */

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
/* 全部 DO 脚置低（5 个阀随后按逻辑重写，其余 40 脚保持低电平） */
static void all_outputs_low(void)
{
    for (unsigned i = 0; i < PIN_COUNT; i++) pin_write(&kPins[i], 0);
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

/* ---- P1/P2 压力换算（假定，待台架核对）：原始值→0.01MPa ----
 * 4mA 活零点 raw=819 对应 0；20mA raw=4095 对应 400bar=4000（0.01MPa）。
 * raw<400 判断线（约 <2mA）；低于零点钳 0。 */
static uint16_t pressure_001mpa(uint16_t raw)
{
    if (raw <= P_RAW_ZERO) return 0;
    uint32_t v = ((uint32_t)(raw - P_RAW_ZERO) * (P_BAR_FULL * 10u)) /
                 (P_RAW_FULL - P_RAW_ZERO);
    return (uint16_t)(v > 4000u ? 4000u : v);
}

/* ---- 心跳帧 0x708：[status, com, hw_id]，1000ms 周期，各态都发 ----
 * INIT 态只发一次 [0x00]，STOP 态只发一次 [0x04]（手册：回复一次后总线关闭）。 */
static void send_heartbeat(uint8_t status)
{
    uint8_t d[8] = {status, g_com_ok ? 1u : 0u, 0x03u, 0, 0, 0, 0, 0};
    (void)can_send(ID_NODE_HB, d);
}

/* ---- PDO 周期帧（仅操作态发送；臂长/角度/吊重/幅度暂发 0=未知） ---- */
static void send_pdos(void)
{
    uint8_t d[8];
    /* 0x198 臂长：L1/L2（0.01m），record 11/12 未定，发 0 */
    d[0] = 0; d[1] = 0; d[2] = 0; d[3] = 0; d[4] = 0; d[5] = 0; d[6] = 0; d[7] = 0;
    (void)can_send(ID_PDO_LEN, d);
    /* 0x1A8 角度：A1/A2（0.01°），来源未定，发 0 */
    (void)can_send(ID_PDO_ANG, d);
    /* 0x1B8 压力：P1/P2（0.01MPa）；bytes4-7 未识别（基线恒 596/36），发 0 */
    d[0] = (uint8_t)(g_p1_001mpa & 0xFFu); d[1] = (uint8_t)(g_p1_001mpa >> 8);
    d[2] = (uint8_t)(g_p2_001mpa & 0xFFu); d[3] = (uint8_t)(g_p2_001mpa >> 8);
    d[4] = 0; d[5] = 0; d[6] = 0; d[7] = 0;
    (void)can_send(ID_PDO_PRES, d);
    /* 0x1F8 IO 状态（upload-3 推断位图；阀位在帧内不可见，见 README） */
    d[0] = g_io_b0; d[1] = 0; d[2] = 0x08u; d[3] = 0;
    d[4] = g_io_b4; d[5] = 0; d[6] = 0; d[7] = 0x05u;
    (void)can_send(ID_PDO_IO, d);
    /* 0x298 吊重：额定/显示/百分比×100/修正前（无力限模型，发 0） */
    d[0] = 0; d[1] = 0; d[2] = 0; d[3] = 0; d[4] = 0; d[5] = 0; d[6] = 0; d[7] = 0;
    (void)can_send(ID_PDO_WGT, d);
    /* 0x2A8 幅度/高度（0.01m），无臂长/角度，发 0 */
    (void)can_send(ID_PDO_AMP, d);
    /* 0x2D8 故障位图（自研位定义） */
    d[0] = g_fault; d[1] = 0; d[2] = 0; d[3] = 0; d[4] = 0; d[5] = 0; d[6] = 0; d[7] = 0;
    (void)can_send(ID_PDO_FLT, d);
}

/* ---- SDO 服务（厂家方言：读命令 0x43，回 0x60；写命令 0x2F/0x2B/0x2A/
 * 0x23，回 0x60；异常回 0x80 + 中止码）。仅预操作/操作态响应。 ---- */
#define SDO_ABORT_NOOBJ 0x06020000u  /* 对象不存在 */
#define SDO_ABORT_RDONLY 0x06010002u /* 写只读对象 */
#define SDO_ABORT_CMD  0x05040001u   /* 命令字无效 */
static void sdo_reply(uint8_t cmd, uint16_t idx, uint8_t sub,
                      const uint8_t v[4])
{
    uint8_t d[8] = {cmd, (uint8_t)(idx & 0xFFu), (uint8_t)(idx >> 8), sub,
                    v[0], v[1], v[2], v[3]};
    (void)can_send(ID_SDO_TX, d);
}
static void sdo_abort(uint16_t idx, uint8_t sub, uint32_t code)
{
    uint8_t v[4] = {(uint8_t)(code & 0xFFu), (uint8_t)((code >> 8) & 0xFFu),
                    (uint8_t)((code >> 16) & 0xFFu), (uint8_t)(code >> 24)};
    sdo_reply(0x80u, idx, sub, v);
}
/* SDO 对象表：读/写的值存在 RAM（v2 不存 Flash；0x1010 只应答）。 */
static int sdo_lookup(uint16_t idx, uint8_t sub, uint32_t *val, int *writable)
{
    switch (idx) {
    case 0x2000u: if (sub != 0) return 0; *val = g_baud; *writable = 1; return 1;
    case 0x2001u: if (sub != 0) return 0; *val = g_nodeid; *writable = 1; return 1;
    case 0x2002u: if (sub != 0) return 0; *val = g_pdo_period; *writable = 1; return 1;
    case 0x2010u: if (sub != 1) return 0; *val = g_duty; *writable = 1; return 1;
    case 0x2011u: if (sub != 1) return 0; *val = g_struct; *writable = 1; return 1;
    case 0x2022u: if (sub != 1) return 0; *val = g_vehicle; *writable = 1; return 1;
    case 0x2028u: if (sub != 0) return 0; *val = g_boom_sec; *writable = 1; return 1;
    default: return 0;
    }
}
static void sdo_store(uint16_t idx, uint8_t sub, uint32_t val)
{
    switch (idx) {
    case 0x2000u: g_baud = val & 0xFFu; break;      /* 只存不换（换波特率须重启） */
    case 0x2001u: g_nodeid = val & 0xFFu; break;    /* 只存不换 */
    case 0x2002u: /* PDO 周期：立即生效，钳 1..200（×10ms） */
        g_pdo_period = (uint8_t)(val < 1u ? 1u : (val > 200u ? 200u : val));
        break;
    case 0x2010u: g_duty = val; break;
    case 0x2011u: g_struct = val; break; /* 结构参数：v2 只存不应用 */
    case 0x2022u: g_vehicle = val & 0xFFu; break;
    case 0x2028u: g_boom_sec = val & 1u; break;
    default: break;
    }
    (void)sub;
}
static void sdo_handle(const uint8_t d[8])
{
    if (g_nmt_state != NMT_PREOP && g_nmt_state != NMT_OPER) return;
    uint8_t cmd = d[0];
    uint16_t idx = (uint16_t)(d[1] | ((uint16_t)d[2] << 8));
    uint8_t sub = d[3];
    uint8_t v[4] = {d[4], d[5], d[6], d[7]};
    if (cmd == 0x43u) {  /* 读 */
        uint32_t val = 0; int wr = 0;
        if (!sdo_lookup(idx, sub, &val, &wr)) { sdo_abort(idx, sub, SDO_ABORT_NOOBJ); return; }
        uint8_t r[4] = {(uint8_t)(val & 0xFFu), (uint8_t)((val >> 8) & 0xFFu),
                        (uint8_t)((val >> 16) & 0xFFu), (uint8_t)(val >> 24)};
        sdo_reply(0x60u, idx, sub, r);
        return;
    }
    if (cmd == 0x2Fu || cmd == 0x2Bu || cmd == 0x2Au || cmd == 0x23u) {  /* 写 */
        /* 0x1010 存参 / 0x1011 恢复：只认 "save"/"load"，应答但不落盘 */
        if (idx == 0x1010u && sub == 0) {
            if (d[4] == 's' && d[5] == 'a' && d[6] == 'v' && d[7] == 'e')
                sdo_reply(0x60u, idx, sub, v);
            else sdo_abort(idx, sub, SDO_ABORT_CMD);
            return;
        }
        if (idx == 0x1011u && sub == 0) {
            if (d[4] == 'l' && d[5] == 'o' && d[6] == 'a' && d[7] == 'd') {
                g_pdo_period = PDO_PERIOD_DEFAULT; g_duty = 0x20u; g_struct = 0u;
                g_vehicle = 0x22u; g_boom_sec = 0u;
                sdo_reply(0x60u, idx, sub, v);
            } else sdo_abort(idx, sub, SDO_ABORT_CMD);
            return;
        }
        uint32_t cur = 0; int wr = 0;
        if (!sdo_lookup(idx, sub, &cur, &wr)) { sdo_abort(idx, sub, SDO_ABORT_NOOBJ); return; }
        if (!wr) { sdo_abort(idx, sub, SDO_ABORT_RDONLY); return; }
        uint32_t val = (uint32_t)d[4] | ((uint32_t)d[5] << 8) |
                       ((uint32_t)d[6] << 16) | ((uint32_t)d[7] << 24);
        sdo_store(idx, sub, val);
        sdo_reply(0x60u, idx, sub, v);
        return;
    }
    sdo_abort(idx, sub, SDO_ABORT_CMD);
}

/* ---- NMT 管理帧 0x001（DLC 2：cmd, nodeID；nodeID 0=全部） ---- */
static void nmt_handle(const uint8_t d[8], uint8_t dlc)
{
    if (dlc < 2) return;
    uint8_t cmd = d[0];
    uint8_t nid = d[1];
    if (nid != 0 && nid != NODE_ID) return;
    switch (cmd) {
    case 0x01u: g_nmt_state = NMT_OPER; g_hb_stop_sent = 0; break;  /* 启动 */
    case 0x02u: /* 停止：发一次 [0x04] 心跳后总线关闭（仍收 NMT 以恢复） */
        g_nmt_state = NMT_STOP; g_hb_stop_sent = 0; break;
    case 0x80u: g_nmt_state = NMT_PREOP; g_hb_stop_sent = 0; break; /* 预操作 */
    case 0x81u: case 0x82u: break; /* 复位命令手册标未使用：忽略 */
    default: break;
    }
}

/* ---- CAN 接收轮询（FIFO0）：每轮最多取 8 帧，绝不无界循环 ----
   只关心 ID 192（QH6546 过卷/接近）；其余帧读后直接丢弃释放。 */
static void can_poll_rx(uint32_t now_ms)
{
    for (int n = 0; n < 8; n++) {
        if ((CAN_RF0R & 3u) == 0) break;  /* FIFO0 空（FMP0=0） */
        uint32_t rir = CAN_RI0R;
        uint16_t id = (uint16_t)((rir >> 21) & 0x7FFu);
        uint32_t rdt = CAN_RDT0R;          /* DLC 在低 4 位 */
        uint8_t dlc = (uint8_t)(rdt & 0xFu);
        uint32_t rdl = CAN_RDL0R;
        uint32_t rdh = CAN_RDH0R;
        CAN_RF0R |= (1u << 5);             /* RFOM0：释放邮箱 */
        uint8_t d[8] = {(uint8_t)(rdl & 0xFFu), (uint8_t)((rdl >> 8) & 0xFFu),
                        (uint8_t)((rdl >> 16) & 0xFFu), (uint8_t)((rdl >> 24) & 0xFFu),
                        (uint8_t)(rdh & 0xFFu), (uint8_t)((rdh >> 8) & 0xFFu),
                        (uint8_t)((rdh >> 16) & 0xFFu), (uint8_t)((rdh >> 24) & 0xFFu)};
        if (id == ID_NMT) { nmt_handle(d, dlc); continue; }
        if (id == ID_HOST_HB) { g_host_hb_last_ms = now_ms; g_com_ok = 1; continue; }
        if (id == ID_SDO_RX) { sdo_handle(d); continue; }
        if (id == ID_QH6546) {
            uint8_t b1 = d[0];
            uint8_t b4 = d[3];  /* byte4（1-based）= data[3] */
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
    /* v2.1：其余 40 个 DO 脚也配成输出并置低（不悬空），随后每轮重申 */
    for (unsigned i = 0; i < PIN_COUNT; i++) {
        gpio_output(kPins[i].base, kPins[i].pin);
        pin_write(&kPins[i], 0);
    }
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
    uint32_t last_hb = 0;
    uint32_t last_pdo = 0;
    uint8_t  cnt = 0;
    uint16_t di_master_raw = 0;
    uint16_t di_mode_raw = 0;
    uint8_t  dump_slot = 0;
    for (;;) {
        REG32(0x40003000u) = 0xAAAAu;     /* IWDG 喂狗 */
        can_poll_rx(ms);                  /* 收 NMT/主机心跳/SDO/ID 192（每轮至多 8 帧） */

        /* ---- 输入采样（50ms）：主令/模式/P1/P2 ---- */
        if (di_ready && (ms - last_di >= 50u)) {
            last_di = ms;
            di_master_raw = di_read(DI_MASTER_SLOT, DI_MASTER_CH);
            di_mode_raw = di_read(DI_MODE_SLOT, DI_MODE_CH);
            g_p1_raw = di_read(P1_SLOT, P1_CH);
            g_p2_raw = di_read(P2_SLOT, P2_CH);
            g_p1_001mpa = pressure_001mpa(g_p1_raw);
            g_p2_001mpa = pressure_001mpa(g_p2_raw);
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

        /* ---- 动作逻辑（文件 03 字面；方向由液压先导手柄给，模块只做使能/互锁） ----
         * v2 变化：变幅/伸缩阀不再以主令为门控。文件 03 对 J1-4/J1-5 写的是
         * “默认得电”、对 J1-18/J1-21 写的是模式切换得电，均无主令条件；
         * 只有主控阀 J1-2 由主令门控（无油压时其余阀得电亦不动）。v1 的
         * 主令门控属偏离原厂，v2 按文件 03 字面执行，台架对照原厂验证。 */
        uint8_t master_on = 0;
        uint8_t telesc = 0;
        if (di_ready) {
            master_on = (di_master_raw > DI_HIGH_RAW) ? 1u : 0u;
            telesc = (di_mode_raw > DI_HIGH_RAW) ? 1u : 0u;
            if (DI_MODE_INVERT) telesc = (uint8_t)(telesc ^ 1u);
        }
        uint8_t over_hoist = (g_over_hoist_main || g_over_hoist_aux) ? 1u : 0u;
        uint8_t qh_timeout = ((ms - g_qh6546_last_ms) > QH6546_TIMEOUT_MS) ? 1u : 0u;
        uint8_t host_timeout = ((ms - g_host_hb_last_ms) > HOST_HB_TIMEOUT_MS) ? 1u : 0u;
        if (host_timeout) g_com_ok = 0;   /* 8 秒无主机心跳：通信故障，只报警不锁阀 */
        uint8_t cut_dn_ex = (over_hoist || qh_timeout) ? 1u : 0u;  /* 切落幅阀+伸臂阀 */
#if OVERLOAD_CUT_ENABLE
        /* 超载切断（v1 未接：无标定压力与臂长/角度，百分比算不出；恒 0 占位） */
        uint8_t overload = 0;
        if (overload) cut_dn_ex = 1u;
#endif
        /* 0x2D8 故障位图（P1/P2 断线只在首次采样后判断，避免上电误报） */
        g_fault = 0;
        if (qh_timeout) g_fault |= FLT_QH_TIMEOUT;
        if (host_timeout) g_fault |= FLT_HOST_TIMEOUT;
        if (ms >= 100u) {
            if (g_p1_raw < P_RAW_OPEN) g_fault |= FLT_P1_OPEN;
            if (g_p2_raw < P_RAW_OPEN) g_fault |= FLT_P2_OPEN;
        }
        /* 0x1F8 IO 状态字节（upload-3 推断位图） */
        g_io_b0 = (uint8_t)(0x80u | (master_on ? 1u : 0u));
        g_io_b4 = (uint8_t)(0x04u | (master_on ? 1u : 0u) | (telesc ? (1u << 4) : 0u));
        if (di_ready) {
            all_outputs_low();  /* v2.1：先把 45 脚全置低，再按逻辑写 5 个阀 */
            pin_write(&kValveMaster, master_on);
            pin_write(&kValveLuffUp, !telesc);
            pin_write(&kValveLuffDn, !telesc && !cut_dn_ex);
            pin_write(&kValveTeleEx, telesc && !cut_dn_ex);
            pin_write(&kValveTeleRe, telesc);
        } else {
            /* DI-HUNT 模式：5 阀强制全关断 */
            all_outputs_low();  /* v2.1：其余脚也全低 */
            pin_write(&kValveMaster, 0);
            pin_write(&kValveLuffUp, 0);
            pin_write(&kValveLuffDn, 0);
            pin_write(&kValveTeleEx, 0);
            pin_write(&kValveTeleRe, 0);
        }

        /* ---- NMT 心跳与 PDO ----
         * INIT：上电发一次 [0x00] 后自动转预操作；STOP：发一次 [0x04] 后停发
         * （仍收 NMT 以恢复）；PREOP/OPER：每 1000ms 发心跳；PDO 只在操作态发。 */
        if (g_nmt_state == NMT_INIT && !g_hb_init_sent) {
            send_heartbeat(NMT_INIT);
            g_hb_init_sent = 1;
            g_nmt_state = NMT_PREOP;
        }
        if (ms - last_hb >= 1000u) {
            last_hb = ms;
            if (g_nmt_state == NMT_PREOP) send_heartbeat(NMT_PREOP);
            else if (g_nmt_state == NMT_OPER) send_heartbeat(NMT_OPER);
            else if (g_nmt_state == NMT_STOP && !g_hb_stop_sent) {
                send_heartbeat(NMT_STOP);
                g_hb_stop_sent = 1;
            }
        }
        if (g_nmt_state == NMT_OPER &&
            (ms - last_pdo) >= ((uint32_t)g_pdo_period * 10u)) {
            last_pdo = ms;
            send_pdos();
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
                   动作模式基值 0x25（bit7：1=HUNT、0=动作）；
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
