/* Demo 板级 IO 抽象层
 * 逻辑层（demo_main.c）只依赖这三个函数，不碰寄存器。
 * 等芯片型号与板内引脚对应确认后，在 io_board.c 里用真实 ADC/GPIO 实现，
 * 逻辑层一行不用改。引脚（连接器级，已按 DL-1018246 针脚表确认）：
 *   模拟量输入：J1-1（功能空脚，类型 AI/DIH）
 *   灯输出   ：J1-17（功能空脚，类型 DOH/DIH/AI/PWM，高边输出 24V）
 */
#ifndef DEMO_IO_H
#define DEMO_IO_H

/* 读台架输入电压，单位 V，范围 0~24（已按输入分压比换算回台架侧电压） */
float io_read_bench_voltage(void);

/* 灯输出：1 = 输出 24V（灯亮），0 = 关断 */
void io_write_lamp(int on);

/* 毫秒计时（主循环节拍用） */
unsigned long io_millis(void);

#endif
