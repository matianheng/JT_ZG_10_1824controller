/* 极简启动代码（C 版）：向量表 + Reset_Handler。
   只为验证「编译→bin→基址」链路；正式工程按确认后的芯片型号换 ST 官方启动文件。 */
#include <stdint.h>
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;
extern uint32_t _estack;
int main(void);
void Reset_Handler(void)
{
    uint32_t *src = &_sidata, *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;
    dst = &_sbss;
    while (dst < &_ebss) *dst++ = 0;
    main();
    for (;;) { }
}
static void Default_Handler(void) { for (;;) { } }
__attribute__((section(".isr_vector"), used))
const uint32_t g_vectors[] = {
    (uint32_t)&_estack,          /* 栈顶 */
    (uint32_t)Reset_Handler,     /* 复位 */
    (uint32_t)Default_Handler, (uint32_t)Default_Handler, (uint32_t)Default_Handler,
    (uint32_t)Default_Handler, (uint32_t)Default_Handler, 0, 0, 0, 0,
    (uint32_t)Default_Handler, (uint32_t)Default_Handler, 0,
    (uint32_t)Default_Handler, (uint32_t)Default_Handler,
};
