#include<linux/kernel.h>
#include<linux/module.h>
#include<linux/timer.h>
#include<linux/io.h>
#include<linux/jiffies.h>
MODULE_LICENSE("GPL");
#define UART_BASE_ADD (0x48022000u)
#define UART_SYSC (0x54)
#define UART_SYSC_SOFTRESET (0x00000001u << 1)
#define UART_SYSC_SOFTRESET_MASK (0x00000001u << 1)
#define UART_SYSS (0x58)
#define UART_SYSS_RESETDONE (0x00000001u)
#define UART_SYSS_RESETDONE_MASK (0x00000001u)
#define UART_DLL (0x0)
#define UART_DLL_MASK (0x000000FFu)
#define UART_RHR (0x0)
#define UART_SSR (0x44u)
#define UART_THR (0x0)
#define UART_DLH (0x4)
#define UART_DLH_MASK (0x3Fu)
#define UART_IER (0x4)
#define UART_EFR (0x8)
#define UART_FCR (0x8)
#define UART_IIR (0x8)
#define UART_LCR (0xC)
#define UART_LCR_DIV_EN (0x00000001u << 7)
#define UART_LCR_DIV_EN_MASK (0x00000001u << 7)
#define UART_LCR_CONFIG_MODE_B (0x00BF)
#define UART_LCR_BREAK_EN (0x00000001u << 6)
#define UART_LCR_BREAK_EN_MASK (0x00000001u << 6)
#define UART_LCR_PARITY_MASK (0x00000001u << 3)
#define UART_LCR_NB_STOP_MASK (0x00000001u << 2)
#define UART_LCR_CHAR_LENGTH_MASK (0x00000003u)
#define UART_MCR (0x10)
#define UART_LSR (0x14)
#define UART_TLR (0x1C)
#define UART_MDR1 (0x20)
#define UART_SCR (0x40)
#define UART_REG_CONFIG_MODE_A (0x0080)
#define UART_REG_CONFIG_MODE_B (0x00BF)
#define UART_REG_CONFIG_MODE_B_MASK()
#define UART_REG_OPERATIONAL_MODE (0x007F)
#define UART_EFR_ENHANCED_EN (0x00000010u)
#define UART_EFR_ENHANCED_EN_MASK (0x00000001u << 4)
#define UART_MCR_TCR_TLR (0x00000040u)
#define UART_MCR_TCR_TLR_MASK  (0x00000001u << 6)
#define UART_FCR_FIFO_EN (0x00000001u)
#define UART_FCR_FIFO_EN_MASK (0x00000001u)
#define UART_TRIG_LVL_GRANULARITY_1 (0x0001)
#define UART_SCR_RX_TRIG_GRANU1 (0x00000080u)
#define UART_SCR_RX_TRIG_GRANU1_MASK (0x00000001u << 7)
#define UART_TLR_RX_FIFO_TRIG_DMA (0x000000F0u)
#define UART_TLR_RX_FIFO_TRIG_DMA_SHIFT (0x00000004u)
#define UART_TLR_TX_FIFO_TRIG_DMA (0x0000000Fu)
#define UART_TLR_TX_FIFO_TRIG_DMA_SHIFT (0x00000000u)
#define UART_FCR_RX_FIFO_TRIG (0x000000C0u)
#define UART_FCR_RX_FIFO_TRIG_SHIFT (0x00000006u)
#define UART_FCR_RX_FIFO_TRIG_16CHAR (0x1u)
#define UART_FCR_RX_FIFO_TRIG_56CHAR (0x2u)
#define UART_FCR_RX_FIFO_TRIG_60CHAR (0x3u)
#define UART_FCR_RX_FIFO_TRIG_8CHAR (0x0u)
#define UART_FCR_RX_TRIG_LVL_8 (0x00000000u)
#define UART_FCR_RX_TRIG_LVL_8_MASK (0x00000003u << 6)
#define UART_FCR_RX_TRIG_LVL_16 (UART_FCR_RX_FIFO_TRIG_16CHAR << \
UART_FCR_RX_FIFO_TRIG_SHIFT)
#define UART_FCR_RX_TRIG_LVL_56 (UART_FCR_RX_FIFO_TRIG_56CHAR << \
UART_FCR_RX_FIFO_TRIG_SHIFT)
#define UART_FCR_RX_TRIG_LVL_60 (UART_FCR_RX_FIFO_TRIG_60CHAR << \
UART_FCR_RX_FIFO_TRIG_SHIFT)
#define UART_FCR_TX_TRIG_LVL_8 (0x00000000u)
#define UART_FCR_TX_TRIG_LVL_8_MASK (0x00000003u << 4)
#define UART_FCR_TX_TRIG_LVL_16 (UART_FCR_TX_FIFO_TRIG_16SPACES << \
UART_FCR_TX_FIFO_TRIG_SHIFT)
#define UART_FCR_TX_TRIG_LVL_32 (UART_FCR_TX_FIFO_TRIG_32SPACES << \
UART_FCR_TX_FIFO_TRIG_SHIFT)
#define UART_FCR_TX_TRIG_LVL_56 (UART_FCR_TX_FIFO_TRIG_56SPACES << \
UART_FCR_TX_FIFO_TRIG_SHIFT)
#define UART_FCR_TX_FIFO_TRIG (0x00000030u)
#define UART_FCR_TX_FIFO_TRIG_SHIFT (0x00000004u)
#define UART_FCR_TX_FIFO_TRIG_8SPACES (0x0u)
#define UART_FCR_TX_FIFO_TRIG_16SPACES (0x1u)
#define UART_FCR_TX_FIFO_TRIG_32SPACES (0x2u)
#define UART_FCR_TX_FIFO_TRIG_56SPACES (0x3u)
#define UART_DMA_EN_PATH_FCR (UART_SCR_DMA_MODE_CTL_FCR)
#define UART_FCR_DMA_MODE (0x00000008u)
#define UART_FCR_DMA_MODE_MASK (0x00000008u)
#define UART_FCR_DMA_MODE_SHIFT (0x00000003u)
#define UART_SCR_DMA_MODE_CTL (0x00000001u)
#define UART_SCR_DMA_MODE_CTL_MASK (0x00000001u)
#define UART_SCR_DMA_MODE_CTL_FCR (0x0u)
#define UART_SCR_DMA_MODE_2 (0x00000006u)
#define UART_SCR_DMA_MODE_2_SHIFT (0x00000001u)
#define UART_SCR_TX_TRIG_GRANU1 (0x00000040u)
#define UART_SCR_TX_TRIG_GRANU1_MASK (0x00000001u << 6)
#define UART_FCR_RX_FIFO_CLEAR_SHIFT (0x00000001u)
#define UART_FCR_TX_FIFO_CLEAR_SHIFT (0x00000002u)
#define UART_MDR1_MODE_SELECT (0x00000007u)
#define UART_MDR1_MODE_SELECT_UART13X (0x3u)
#define UART_MDR1_MODE_SELECT_UART16X (0x0u)
#define UART_MDR1_MODE_SELECT_MASK (0x00000007u)
#define UART16x_OPER_MODE (UART_MDR1_MODE_SELECT_UART16X)
#define UART13x_OPER_MODE (UART_MDR1_MODE_SELECT_UART13X)
#define UART_MDR1_MODE_SELECT_DISABLED (0x7u)
#define UART_MDR1_MODE_SELECT_DISABLED_MASK  (0x00000007u)
#define UART_IER_SLEEP_MODE_IT (0x00000010u)
#define UART_IER_THR_IT (0x00000002u)
#define UART_IIR_IT_TYPE (0x0000003Eu)
#define UART_IIR_IT_TYPE_SHIFT (0x00000001u)
#define UART_IIR_IT_TYPE_RHRINT (0x2u)
#define UART_IIR_IT_TYPE_THRINT (0x1u)
#define UART_INTID_TX_THRES_REACH (UART_IIR_IT_TYPE_THRINT << UART_IIR_IT_TYPE_SHIFT)
#define UART_INTID_RX_THRES_REACH (UART_IIR_IT_TYPE_RHRINT << UART_IIR_IT_TYPE_SHIFT)
#define UART_INT_LINE_STAT (UART_IER_LINE_STS_IT)
#define UART_INT_THR (UART_IER_THR_IT)
#define UART_INT_RHR_CTI (UART_IER_RHR_IT)
#define UART_LSR_RX_FIFO_E (0x00000001u)
#define UART_LSR_TX_FIFO_E (0x00000020u)
#define UART_LSR_TX_FIFO_E_MASK (0x00000001u << 5)
#define UART_LSR_TX_SR_E (0x00000040u)
#define UART_IER_LINE_STS_IT (0x00000004u)
#define UART_IER_RHR_IT (0x00000001u)
#define UART_IER_CTSIT_IT (0x00000001u << 7)
#define UART_IER_RTSIT_IT (0x00000001u << 6)
#
//#include "soc_AM335x.h"
#define SOC_PRCM_REGS (0x44E00000)
#define SOC_CM_PER_REGS (SOC_PRCM_REGS + 0)
#define SOC_CM_PER_REGS (SOC_PRCM_REGS + 0)
#define SOC_CM_WKUP_REGS (SOC_PRCM_REGS + 0x400)
#define SOC_UART_0_REGS (0x44E09000)
//#define SOC_UART_1_REGS (0x48022000)

//#define SOC_UART_1_REGS SOC_UART_0_REGS
#define SOC_UART_2_REGS (0x48024000)
#define SOC_UART_1_REGS SOC_UART_2_REGS
#define SOC_UART_3_REGS (0x481A6000)
#define SOC_UART_4_REGS (0x481A8000)
#define SOC_UART_5_REGS (0x481AA000)
#define SOC_CONTROL_REGS (0x44E10000)
#define SOC_CONTROL_REGS_CONF_UART1_RX (0x980u)
#define SOC_CONTROL_REGS_CONF_UART1_RX_ACTIVE (0x00000001u << 5)
#define SOC_CONTROL_REGS_CONF_UART1_RX_ACTIVE_MASK (0x00000001u << 5)
#define SOC_CONTROL_REGS_CONF_UART1_RX_PUTYPESEL (0x00000001u << 4)
#define SOC_CONTROL_REGS_CONF_UART1_RX_PUTYPESEL_MASK (0x00000001u << 4)
#define SOC_CONTROL_REGS_CONF_UART1_RX_MODE_0 (0x00000000u)
#define SOC_CONTROL_REGS_CONF_UART1_RX_MODE_MASK (0x00000007u)
#define SOC_CONTROL_REGS_CONF_UART1_TX (0x984u)
#define SOC_CONTROL_REGS_CONF_UART1_TX_RX_DISABLE (0x00000000u)
#define SOC_CONTROL_REGS_CONF_UART1_TX_RX_MASK (0x00000000u  << 5)
#define SOC_CONTROL_REGS_CONF_UART1_TX_PUTYPESEL (0x00000001u << 4)
#define SOC_CONTROL_REGS_CONF_UART1_TX_PUTYPESEL_MASK (0x00000001u << 4)
#define SOC_CONTROL_REGS_CONF_UART1_TX_MODE_0 (0x00000000u)
#define SOC_CONTROL_REGS_CONF_UART1_TX_MODE_MASK (0x00000007u)
#define SOC_AINTC_REGS (0x48200000u)
//#include "interrupt.h"
#define AINTC_HOSTINT_ROUTE_IRQ (0)
#define INTC_SIR_IRQ 0x00000040u
#define SYS_INT_UART0INT (72)
#define SYS_INT_UART1INT (73)
#define SYS_INT_UART2INT (74)
#define INTC_ILR(n) (0x100 + ((n) * 0x04))
#define INTC_ILR_PRIORITY (0x000001FCu)
#define INTC_ILR_PRIORITY_SHIFT (0x00000002u)
#define INTC_MIR_CLEAR(n) (0x88 + ((n) * 0x20))
#define INTC_MIR_CLEAR2 0x000000C8u
#define INTC_SYSCONFIG (0x10)
#define INTC_SYSCONFIG_SOFTRESET (0x00000002u)
#define INTC_SYSSTATUS (0x14)
#define INTC_SYSSTATUS_RESETDONE (0x00000001u)
#define INTC_THRESHOLD (0x68)
#define INTC_THRESHOLD_PRIORITYTHRESHOLD (0x000000FFu)
//#include "hw_types.h"
//#define HWREG(x) (*((volatile unsigned int *)(x)))
#define TRUE 1
#define FALSE 0
// from uart.c
//#include "hw_control_AM335x.h"
#define CONTROL_CONF_UART_RXD(n) (0x970 + ((n) * 0x10))
#define CONTROL_CONF_UART_TXD(n) (0x974 + ((n) * 0x10))
#define CONTROL_CONF_UART0_RXD_CONF_UART0_RXD_RXACTIVE (0x00000020u)
#define CONTROL_CONF_UART0_RXD_CONF_UART0_RXD_PUTYPESEL (0x00000010u)
#define CONTROL_CONF_UART0_TXD_CONF_UART0_TXD_PUTYPESEL (0x00000010u)
//#include "hw_cm_wkup.h"
#define CM_WKUP_CLKSTCTRL (0x0)
#define CM_WKUP_CONTROL_CLKCTRL (0x4)
#define CM_WKUP_CONTROL_CLKCTRL_MODULEMODE (0x00000003u)
#define CM_WKUP_CONTROL_CLKCTRL_MODULEMODE_ENABLE (0x2u)
#define CM_WKUP_CLKSTCTRL_CLKTRCTRL (0x00000003u)
#define CM_WKUP_CLKSTCTRL_CLKTRCTRL_SW_WKUP (0x2u)
#define CM_WKUP_CM_L3_AON_CLKSTCTRL (0x18)
#define CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKTRCTRL (0x00000003u)
#define CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKTRCTRL_SW_WKUP (0x2u)
#define CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKACTIVITY_L3_AON_GCLK (0x00000008u)
#define CM_WKUP_UART0_CLKCTRL (0xb4)
#define CM_WKUP_UART0_CLKCTRL_MODULEMODE (0x00000003u)
#define CM_WKUP_UART0_CLKCTRL_MODULEMODE_ENABLE (0x2u)
#define CM_WKUP_CONTROL_CLKCTRL_IDLEST (0x00030000u)
#define CM_WKUP_CONTROL_CLKCTRL_IDLEST_SHIFT (0x00000010u)
#define CM_WKUP_CONTROL_CLKCTRL_IDLEST_FUNC (0x0u)
#define CM_WKUP_L4WKUP_CLKCTRL_IDLEST (0x00030000u)
#define CM_WKUP_L4WKUP_CLKCTRL_IDLEST_SHIFT (0x00000010u)
#define CM_WKUP_L4WKUP_CLKCTRL_IDLEST_FUNC (0x0u)
#define CM_WKUP_L4WKUP_CLKCTRL (0xc)
#define CM_WKUP_CLKSTCTRL_CLKACTIVITY_L4_WKUP_GCLK (0x00000004u)
#define CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL_CLKACTIVITY_L4_WKUP_AON_GCLK (0x00000004u)
#define CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL (0xcc)
#define CM_WKUP_CLKSTCTRL_CLKACTIVITY_UART0_GFCLK (0x00001000u)
#define CM_WKUP_UART0_CLKCTRL_IDLEST (0x00030000u)
#define CM_WKUP_UART0_CLKCTRL_IDLEST_SHIFT (0x00000010u)
#define CM_WKUP_UART0_CLKCTRL_IDLEST_FUNC (0x0u)
//#include "hw_cm_per.h"
#define CM_PER_L4LS_CLKSTCTRL (0x00000000u)
#define CM_PER_L4LS_CLKSTCTRL_MODULEMODE_MASK (0x00000003u)
#define CM_PER_L4LS_CLKSTCTRL_MODULEMODE_ENABLE (0x00000002u)
#define CM_PER_L4LS_CLKSTCTRL_MODULEMODE_DISABLE (0x00000001u)
#define CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_UART_GFCLK_ACTIVE (0x00000001u << 10)
#define CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_UART_GFCLK_MASK (0x00000001u << 10)
#define CM_PER_UART1_CLKCTRL (0x6Cu)
#define CM_PER_UART1_CLKCTRL_MODULEMODE_ENABLE (0x00000002u)
#define CM_PER_UART1_CLKCTRL_MODULEMODE_DISABLE (0x00000000u)
#define CM_PER_UART1_CLKCTRL_MODULEMODE_MASK (0x00000003u)
#define CM_PER_UART1_CLKCTRL_IDLEST_FUNC (0x00000000u)
#define CM_PER_UART1_CLKCTRL_IDLEST_MASK (0x00000003u << 16)
#define CM_PER_UART2_CLKCTRL (0x00000007u)
#define CM_PER_L3S_CLKSTCTRL (0x4)
#define CM_PER_L3_CLKCTRL (0xe0)
#define CM_PER_L3_CLKSTCTRL (0xc)
#define CM_PER_L3_CLKCTRL_MODULEMODE (0x00000003u)
#define CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE (0x2u)
#define CM_PER_L3_INSTR_CLKCTRL (0xdc)
#define CM_PER_L3_INSTR_CLKCTRL_MODULEMODE (0x00000003u)
#define CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE (0x2u)
#define CM_PER_L3_CLKSTCTRL_CLKTRCTRL (0x00000003u)
#define CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP (0x2u)
#define CM_PER_OCPWP_L3_CLKSTCTRL (0x12c)
#define CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL (0x00000003u)
#define CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP (0x00000002u)
#define CM_PER_L3S_CLKSTCTRL_CLKTRCTRL (0x00000003u)
#define CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP (0x2u)
#define CM_PER_L3_CLKCTRL_IDLEST (0x00030000u)
#define CM_PER_L3_CLKCTRL_IDLEST_SHIFT (0x00000010u)
#define CM_PER_L3_CLKCTRL_IDLEST_FUNC (0x0u)
#define CM_PER_L3_INSTR_CLKCTRL_IDLEST (0x00030000u)
#define CM_PER_L3_INSTR_CLKCTRL_IDLEST_SHIFT (0x00000010u)
#define CM_PER_L3_INSTR_CLKCTRL_IDLEST_FUNC (0x0u)
#define CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK (0x00000010u)
#define CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L3_GCLK (0x00000010u)
#define CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK (0x00000008u)
#define REG(x)(*((volatile uint32_t *)(x)))
#define MY_BIT(x)(0x1 << x)
#define UART_MODULE_INPUT_CLK (48000000)
// from interrupt.c
#define REG_IDX_SHIFT (0x05)
#define REG_BIT_MASK (0x1F)
#define NUM_INTERRUPTS (128u)
int instance;
void UARTModuleClkCfg(unsigned int baseAddUARTx);
void UARTPinMuxing(int a);
void UARTReset(int a);
void UartFiFoConfigure(int a);
void UARTConfigProtocol(int a);
void Send(uint8_t data);
uint32_t Read_reg(uint32_t physic_add);
void Write_reg(uint32_t desired_value,uint32_t physic_add,uint32_t mask);
void Write_reg_noMask(uint32_t desired_value,uint32_t physic_add);
void Debug(uint32_t desired_value,uint32_t reg,uint32_t mask,int line,int minitime);
void Debug_noMask(uint32_t desired_value,uint32_t reg,int line,int minitime);
void call_back(struct timer_list * timer);
int Comparison(uint32_t reg1,uint32_t reg2);
void Wait(uint32_t desired_value,uint32_t reg,uint32_t mask,int line,int minitime);
void Read(uint8_t*buffer,int buff_size,uint32_t timeout);
void UARTIrqEnable(int instance);
void*__iomem tempt;
int time = 10000;
uint32_t reg_data = 0;
int flag = 0;
void call_back(struct timer_list * timer){
    pr_info("Time out\n");
    flag = 1;
}
struct timer_list my_timer = {
    .function = call_back,
    .expires = 1000,
};
int Comparison(uint32_t reg1,uint32_t reg2){
    if(reg1==reg2){
        return 0;
    }
    else
        return 1;
}
void test(void){
    asm("srs sp!, #18\n\t");
    pr_info("Test for irq handler\n");
    asm("rfe sp!");
    asm("subs pc, lr, #4");
}
void (*Test)(void) = test;
char buffer[100];
uint32_t IRQ_ISR = 0;
static int __init my_init(void){
    pr_info("Module was loaded");
    UARTModuleClkCfg(2);
    //UARTIrqEnable(instance);
    instance = 1;
    UARTPinMuxing(instance);
    UARTReset(instance);
    UartFiFoConfigure(instance);
    UARTConfigProtocol(instance);
    //UARTIrqEnable(instance);
    //IRQ_ISR = virt_to_phys((void*)Test);
    Send('o');
    return 0;
}
static void __exit my_exit(void){
    // Disable UART module
    Write_reg(UART_MDR1_MODE_SELECT_DISABLED,SOC_UART_1_REGS + UART_MDR1,UART_MDR1_MODE_SELECT_DISABLED_MASK);
    Debug(UART_MDR1_MODE_SELECT_DISABLED,SOC_UART_1_REGS + UART_MDR1,UART_MDR1_MODE_SELECT_DISABLED_MASK,__LINE__,time);
    // Set the IDLEMODE of module to Force idle
    Write_reg(0x00000000u << 3, SOC_UART_1_REGS + UART_SYSC, 0x00000003u << 3);
    Debug(0x00000000u << 3, SOC_UART_1_REGS + UART_SYSC, 0x00000003u << 3,__LINE__,time);
    // Set the module mode of CM_UART to disable
    Write_reg(0x00000000u,SOC_CM_PER_REGS + CM_PER_UART1_CLKCTRL, 0x00000003u);
    Debug(0x00000000u,SOC_CM_PER_REGS + CM_PER_UART1_CLKCTRL, 0x00000003u,__LINE__,time);
    //Debug(0x00000000u << 10,SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL, 0x00000001u << 10,__LINE__,time);
    // Gating the clock for uart module
    //Write_reg(CM_PER_UART1_CLKCTRL_MODULEMODE_DISABLE,SOC_CM_PER_REGS + CM_PER_UART1_CLKCTRL,CM_PER_UART1_CLKCTRL_MODULEMODE_MASK);
    //Debug(CM_PER_UART1_CLKCTRL_MODULEMODE_DISABLE,SOC_CM_PER_REGS + CM_PER_UART1_CLKCTRL,CM_PER_UART1_CLKCTRL_MODULEMODE_MASK,__LINE__,time);
    //Debug(0x00000003u << 16 ,SOC_CM_PER_REGS + CM_PER_UART1_CLKCTRL,0x00000003u << 16,__LINE__,time);
    pr_info("Unload module\n");
}
module_init(my_init);
module_exit(my_exit);
void UARTModuleClkCfg(unsigned int baseAddUARTx) {
    if(baseAddUARTx == 1){
    // Configure L4LS clock domain
    reg_data = Read_reg(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL);
    Write_reg(CM_PER_L4LS_CLKSTCTRL_MODULEMODE_ENABLE,SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL,CM_PER_L4LS_CLKSTCTRL_MODULEMODE_MASK);
    Debug(CM_PER_L4LS_CLKSTCTRL_MODULEMODE_ENABLE,SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL,CM_PER_L4LS_CLKSTCTRL_MODULEMODE_MASK, __LINE__, time);

    //Write_reg(CM_PER_L4LS_CLKSTCTRL_MODULEMODE_DISABLE,SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL,CM_PER_L4LS_CLKSTCTRL_MODULEMODE_MASK);
    //Debug(CM_PER_L4LS_CLKSTCTRL_MODULEMODE_DISABLE,SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL,CM_PER_L4LS_CLKSTCTRL_MODULEMODE_MASK, __LINE__, time);

    // set bit CM_PER_UART1_CLKCTRL_MODULEMODE to enable
    reg_data = Read_reg(SOC_CM_PER_REGS + CM_PER_UART1_CLKCTRL);
    Write_reg(CM_PER_UART1_CLKCTRL_MODULEMODE_ENABLE,SOC_CM_PER_REGS+CM_PER_UART1_CLKCTRL,CM_PER_UART1_CLKCTRL_MODULEMODE_MASK);
    Debug(CM_PER_UART1_CLKCTRL_MODULEMODE_ENABLE,SOC_CM_PER_REGS+CM_PER_UART1_CLKCTRL,CM_PER_UART1_CLKCTRL_MODULEMODE_MASK,__LINE__,time);

    // wait for bit M_PER_L4LS_CLKSTCTRL_CLKACTIVITY_UART_GFCLK set to 1
    Debug(CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_UART_GFCLK_ACTIVE,SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL,CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_UART_GFCLK_MASK,__LINE__,time);

    // wait for bit ID CM_PER_UART1_CLKCTRL_IDLEST set to 11
    Debug(CM_PER_UART1_CLKCTRL_IDLEST_FUNC,SOC_CM_PER_REGS+CM_PER_UART1_CLKCTRL,CM_PER_UART1_CLKCTRL_IDLEST_MASK,__LINE__,time);
    }
    else if(baseAddUARTx == 2){
    // Configure L4LS clock domain
    reg_data = Read_reg(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL);
    Write_reg(CM_PER_L4LS_CLKSTCTRL_MODULEMODE_ENABLE,SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL,CM_PER_L4LS_CLKSTCTRL_MODULEMODE_MASK);
    Debug(CM_PER_L4LS_CLKSTCTRL_MODULEMODE_ENABLE,SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL,CM_PER_L4LS_CLKSTCTRL_MODULEMODE_MASK, __LINE__, time);

    // set bit CM_PER_UART2_CLKCTRL_MODULEMODE to enablef
    reg_data = Read_reg(SOC_CM_PER_REGS + CM_PER_UART2_CLKCTRL);
    Write_reg(CM_PER_UART1_CLKCTRL_MODULEMODE_ENABLE,SOC_CM_PER_REGS+CM_PER_UART2_CLKCTRL,CM_PER_UART1_CLKCTRL_MODULEMODE_MASK);
    Debug(CM_PER_UART1_CLKCTRL_MODULEMODE_ENABLE,SOC_CM_PER_REGS+CM_PER_UART2_CLKCTRL,CM_PER_UART1_CLKCTRL_MODULEMODE_MASK,__LINE__,time);

    // wait for bit M_PER_L4LS_CLKSTCTRL_CLKACTIVITY_UART_GFCLK set to 1
    Debug(CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_UART_GFCLK_ACTIVE,SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL,CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_UART_GFCLK_MASK,__LINE__,time);

    // wait for bit ID CM_PER_UART1_CLKCTRL_IDLEST set to 11
    Debug(CM_PER_UART1_CLKCTRL_IDLEST_FUNC,SOC_CM_PER_REGS+CM_PER_UART2_CLKCTRL,CM_PER_UART1_CLKCTRL_IDLEST_MASK,__LINE__,time);
    }
}

void UARTPinMuxing(int a){
if(a==1){
    // config rxd pin of uart1, active pin and choose pull up mode.
    Write_reg(SOC_CONTROL_REGS_CONF_UART1_RX_ACTIVE|SOC_CONTROL_REGS_CONF_UART1_RX_PUTYPESEL|SOC_CONTROL_REGS_CONF_UART1_RX_MODE_0,SOC_CONTROL_REGS + SOC_CONTROL_REGS_CONF_UART1_RX,
    SOC_CONTROL_REGS_CONF_UART1_RX_ACTIVE_MASK|SOC_CONTROL_REGS_CONF_UART1_RX_PUTYPESEL_MASK|SOC_CONTROL_REGS_CONF_UART1_RX_MODE_MASK);
    Debug(SOC_CONTROL_REGS_CONF_UART1_RX_ACTIVE|SOC_CONTROL_REGS_CONF_UART1_RX_PUTYPESEL|SOC_CONTROL_REGS_CONF_UART1_RX_MODE_0,
    SOC_CONTROL_REGS + SOC_CONTROL_REGS_CONF_UART1_RX,SOC_CONTROL_REGS_CONF_UART1_RX_ACTIVE_MASK|SOC_CONTROL_REGS_CONF_UART1_RX_PUTYPESEL_MASK|SOC_CONTROL_REGS_CONF_UART1_RX_MODE_MASK,__LINE__,time);
    // config txd pin of uart1, choose pull up mode.
    Write_reg(SOC_CONTROL_REGS_CONF_UART1_TX_RX_DISABLE|SOC_CONTROL_REGS_CONF_UART1_TX_PUTYPESEL|SOC_CONTROL_REGS_CONF_UART1_TX_MODE_0,SOC_CONTROL_REGS + SOC_CONTROL_REGS_CONF_UART1_TX,
    SOC_CONTROL_REGS_CONF_UART1_TX_PUTYPESEL_MASK|SOC_CONTROL_REGS_CONF_UART1_TX_MODE_MASK|SOC_CONTROL_REGS_CONF_UART1_TX_RX_MASK);
    Debug(SOC_CONTROL_REGS_CONF_UART1_TX_RX_DISABLE|SOC_CONTROL_REGS_CONF_UART1_TX_PUTYPESEL|SOC_CONTROL_REGS_CONF_UART1_TX_MODE_0,SOC_CONTROL_REGS + SOC_CONTROL_REGS_CONF_UART1_TX,
    SOC_CONTROL_REGS_CONF_UART1_TX_PUTYPESEL_MASK|SOC_CONTROL_REGS_CONF_UART1_TX_MODE_MASK|SOC_CONTROL_REGS_CONF_UART1_TX_RX_MASK,__LINE__,time);
    }
else if(a==2){

    }
}
void UARTReset(int a){
    Write_reg(UART_SYSC_SOFTRESET,SOC_UART_1_REGS + UART_SYSC,UART_SYSC_SOFTRESET_MASK);
    //Debug(UART_SYSC_SOFTRESET,SOC_UART_1_REGS + UART_SYSC,UART_SYSC_SOFTRESET_MASK,__LINE__,time);
    Debug(UART_SYSS_RESETDONE,SOC_UART_1_REGS + UART_SYSS,UART_SYSS_RESETDONE_MASK,__LINE__,time);
}
void UartFiFoConfigure(int a){
    // Following program model in reference manual
    uint32_t tempt_uart_lcr = Read_reg(SOC_UART_1_REGS + UART_LCR);
    Write_reg_noMask(UART_REG_CONFIG_MODE_B,SOC_UART_1_REGS + UART_LCR);
    Debug_noMask(UART_REG_CONFIG_MODE_B,SOC_UART_1_REGS + UART_LCR,__LINE__,time);

    // save the current efr register and set 1 to efr enhanced bit
    uint32_t tempt_uart_efr = Read_reg(SOC_UART_1_REGS + UART_EFR);
    Write_reg(UART_EFR_ENHANCED_EN,SOC_UART_1_REGS + UART_EFR,UART_EFR_ENHANCED_EN_MASK);
    Debug(UART_EFR_ENHANCED_EN,SOC_UART_1_REGS + UART_EFR,UART_EFR_ENHANCED_EN_MASK,__LINE__,time);

    //switch to mode A
    Write_reg_noMask(UART_REG_CONFIG_MODE_A,SOC_UART_1_REGS + UART_LCR);
    Debug_noMask(UART_REG_CONFIG_MODE_A,SOC_UART_1_REGS + UART_LCR,__LINE__,time);

    // Enable access to TCR and TLR register by enable sub mode on bit tcr_tlr of mcr register
    uint32_t tempt_uart_mcr = Read_reg(SOC_UART_1_REGS + UART_MCR);
    Write_reg(UART_MCR_TCR_TLR,SOC_UART_1_REGS + UART_MCR,UART_MCR_TCR_TLR_MASK);
    Debug(UART_MCR_TCR_TLR,SOC_UART_1_REGS + UART_MCR,UART_MCR_TCR_TLR_MASK,__LINE__,time);

    uint32_t tempt_uart_scr = Read_reg(SOC_UART_1_REGS + UART_SCR);
    pr_info("Value of SCR register: %x\n",tempt_uart_scr);
    uint32_t tempt_uart_tlr = Read_reg(SOC_UART_1_REGS + UART_TLR);
    pr_info("Value of TLR register: %x\n",tempt_uart_tlr);

    pr_info("Check value of LCR register before config trigger level: %x\n",Read_reg(SOC_UART_1_REGS + UART_LCR));
    //set RX_FIFO_TRGGER_LEVEL and TX_FIFO_TRIGGER_LEVEL to 1.
    // Write 01 to RX_FIFO_TRIG
    Write_reg(0x00000003u << 6,SOC_UART_1_REGS + UART_FCR, 0x00000001u << 6 | 0x00000001u << 7);
    // Write 01 to TX_FIFO_TRIG
    Write_reg(0x00000003u << 4,SOC_UART_1_REGS + UART_FCR, 0x00000001u << 4 | 0x00000001u << 5);
    // No DMA
    Write_reg(0x00000000u << 3,SOC_UART_1_REGS + UART_FCR,UART_FCR_DMA_MODE_MASK);
    // Write 1 to enable FIFO function
    Write_reg(UART_FCR_FIFO_EN,SOC_UART_1_REGS + UART_FCR,UART_FCR_FIFO_EN_MASK);
    // Clear the TX FIFO
    Write_reg(0x00000001u << 1,SOC_UART_1_REGS + UART_FCR, 0x00000001u << 1);
    // Clear the RX FIFO
    Write_reg(0x00000001u << 2,SOC_UART_1_REGS + UART_FCR, 0x00000001u << 2);
    //Switch register to configuration mode B
    Write_reg_noMask(UART_REG_CONFIG_MODE_B,SOC_UART_1_REGS + UART_LCR);

    //Set trigger level for dma and interrupt,in this setup we use the configurationo for FCR register
    Write_reg(0x00000000u,SOC_UART_1_REGS + UART_TLR,0x000000FFu);
    Debug(0x00000000u,SOC_UART_1_REGS + UART_TLR,0x000000FFu,__LINE__,time);

    // Choose DMA mode from scr registe: No DMA
    Write_reg(0x00000001u, SOC_UART_1_REGS + UART_SCR,0x00000001u);
    Debug(0x00000001u, SOC_UART_1_REGS + UART_SCR,0x00000001u,__LINE__,time);
    Write_reg(0x00000000u << 1,SOC_UART_1_REGS + UART_SCR,0x00000003u << 1);
    Debug(0x00000000u << 1,SOC_UART_1_REGS + UART_SCR,0x00000003u << 1,__LINE__,time);
    // Choose generate THR interrupt when TX FIFO and TX shift register are empty
    Write_reg(0x00000000u << 3,SOC_UART_1_REGS + UART_SCR,0x00000001u << 3);
    Debug(0x00000000u << 3,SOC_UART_1_REGS + UART_SCR,0x00000001u << 3,__LINE__,time);
    // Enable the granularity of 1 for trigger RX and TX level
    Write_reg(0x00000001u << 7 | 0x00000001u << 6,SOC_UART_1_REGS + UART_SCR,0x00000001u << 7 | 0x00000001u << 6);
    Debug(0x00000001u << 7 | 0x00000001u << 6,SOC_UART_1_REGS + UART_SCR,0x00000001u << 7 | 0x00000001u << 6,__LINE__,time);

    //Switch to mode A to configure MCR register
    Write_reg_noMask(UART_REG_CONFIG_MODE_A,SOC_UART_1_REGS + UART_LCR);
    Debug_noMask(UART_REG_CONFIG_MODE_A,SOC_UART_1_REGS + UART_LCR,__LINE__,time);

    //Restore TCR_TLR bit of MCR register
    Write_reg(tempt_uart_mcr,SOC_UART_1_REGS + UART_MCR,UART_MCR_TCR_TLR_MASK);
    Debug(tempt_uart_mcr,SOC_UART_1_REGS + UART_MCR,UART_MCR_TCR_TLR_MASK,__LINE__,time);

    //Restore ehanced bit in efr register.
    Write_reg(tempt_uart_efr,SOC_UART_1_REGS + UART_EFR,0x00000001u << 4);
    Debug(tempt_uart_efr,SOC_UART_1_REGS + UART_EFR,0x00000001u << 4,__LINE__,time);


    //Restore the value of LCR register
    Write_reg_noMask(tempt_uart_lcr,SOC_UART_1_REGS + UART_LCR);
    Debug_noMask(tempt_uart_lcr,SOC_UART_1_REGS + UART_LCR,__LINE__,time);
}
void UARTConfigProtocol(int a){
    //  Disable UART to access UART_DLL and UART_DLH register
    Write_reg(UART_MDR1_MODE_SELECT_DISABLED,SOC_UART_1_REGS + UART_MDR1,UART_MDR1_MODE_SELECT_DISABLED_MASK);
    Debug(UART_MDR1_MODE_SELECT_DISABLED,SOC_UART_1_REGS + UART_MDR1,UART_MDR1_MODE_SELECT_DISABLED_MASK,__LINE__,time);

    // Switch to B mode to access EFR register
    Write_reg_noMask(UART_REG_CONFIG_MODE_B,SOC_UART_1_REGS + UART_LCR);
    Debug_noMask(UART_REG_CONFIG_MODE_B,SOC_UART_1_REGS + UART_LCR,__LINE__,time);

    // set EHANCED bit EFR to 1 to accesss IER[7:4] register
    uint32_t tempt_uart_efr = Read_reg(SOC_UART_1_REGS + UART_EFR);
    Write_reg(UART_EFR_ENHANCED_EN,SOC_UART_1_REGS + UART_EFR,UART_EFR_ENHANCED_EN_MASK);
    Debug(UART_EFR_ENHANCED_EN,SOC_UART_1_REGS + UART_EFR,UART_EFR_ENHANCED_EN_MASK,__LINE__,time);

    // Switch to operational mode to access ier register
    Write_reg_noMask(0x00000000u,SOC_UART_1_REGS + UART_LCR);
    Debug_noMask(0x00000000u,SOC_UART_1_REGS + UART_LCR,__LINE__,time);

    // Clear IER register
    Write_reg_noMask(0x00000000u,SOC_UART_1_REGS + UART_IER);
    Debug_noMask(0x00000000u,SOC_UART_1_REGS + UART_IER,__LINE__,time);

    //Switch to B mode to access DLL and DLH register
    Write_reg_noMask(UART_REG_CONFIG_MODE_B, SOC_UART_1_REGS + UART_LCR);
    Debug_noMask(UART_REG_CONFIG_MODE_B, SOC_UART_1_REGS + UART_LCR,__LINE__,time);

    //set the baudrate through the DLL and DLH register:
    Write_reg(0x38u,SOC_UART_1_REGS + UART_DLL,UART_DLL_MASK);
    Debug(0x38u,SOC_UART_1_REGS + UART_DLL,UART_DLL_MASK,__LINE__,time);
    Write_reg(0x01u,SOC_UART_1_REGS + UART_DLH,UART_DLH_MASK);
    Debug(0x01u,SOC_UART_1_REGS + UART_DLH,UART_DLH_MASK,__LINE__,time);

    //Switch to operational mode to access IER register
    Write_reg_noMask(0x00000000u,SOC_UART_1_REGS + UART_LCR);
    Debug_noMask(0x00000000u,SOC_UART_1_REGS + UART_LCR,__LINE__,time);

    // load new interupt configuration, CHANGE IT IF YOU WANT TO USE INTERUPT
    Write_reg((0x00000001u)|(0x00000001u << 1)|(0x00000001u << 2)|(0x00000001u << 3),SOC_UART_1_REGS + UART_IER,0x000000FFu);
    Debug((0x00000001u)|(0x00000001u << 1)|(0x00000001u << 2)|(0x00000001u << 3),SOC_UART_1_REGS + UART_IER,0x000000FFu,__LINE__,time);
    // Disable all interrupt of UART1
    /* Write_reg(0x00000000u,SOC_UART_1_REGS + UART_IER,0x000000FFu);
    Debug(0x00000000u,SOC_UART_1_REGS + UART_IER,0x000000FFu,__LINE__,time); */
    /* Write_reg_noMask(0x00000000u,SOC_UART_1_REGS + UART_IER);
    Debug_noMask(0x00000000u,SOC_UART_1_REGS + UART_IER,__LINE__,time); */

    // Switch to B mode, to access the EFR register
    Write_reg_noMask(UART_REG_CONFIG_MODE_B,SOC_UART_1_REGS + UART_LCR);
    Debug_noMask(UART_REG_CONFIG_MODE_B,SOC_UART_1_REGS + UART_LCR,__LINE__,time);

    // Restore the value of efr register
    Write_reg(tempt_uart_efr,SOC_UART_1_REGS + UART_EFR,UART_EFR_ENHANCED_EN_MASK);
    Debug(tempt_uart_efr,SOC_UART_1_REGS + UART_EFR,UART_EFR_ENHANCED_EN_MASK,__LINE__,time);

    // Load the new protocol formatting (parity,stop bit,character length)
    Write_reg(0x00000000u <<7|0x00000000u << 6,SOC_UART_1_REGS + UART_LCR,0x00000003u << 6);
    Debug(0x00000000u <<7|0x00000000u << 6,SOC_UART_1_REGS + UART_LCR,0x00000003u << 6,__LINE__,time);
    // No parity, stop bit = 1, char_lenth = 8
    Write_reg(0x03|0x00000000u << 2|0x00000000u << 3,SOC_UART_1_REGS + UART_LCR,0x00000003u|0x00000001u << 2|0x00000001u << 3);
    Debug(0x03|0x00000000u << 2|0x00000000u << 3,SOC_UART_1_REGS + UART_LCR,0x00000003u|0x00000001u << 2|0x00000001u << 3,__LINE__,time);
    // Choose the 16x mode on MDR1 register
    Write_reg(UART_MDR1_MODE_SELECT_UART16X,SOC_UART_1_REGS + UART_MDR1, UART_MDR1_MODE_SELECT_MASK);
    Debug(UART_MDR1_MODE_SELECT_UART16X,SOC_UART_1_REGS + UART_MDR1, UART_MDR1_MODE_SELECT_MASK,__LINE__,time);
}
void Wait(uint32_t desired_value,uint32_t reg,uint32_t mask,int line,int minitime){
    uint32_t tempt =1;
    unsigned long sectime = msecs_to_jiffies(minitime)+jiffies; // get the root
    while((tempt!=0)&&time_before(jiffies,sectime)){
        tempt = (uint32_t)(Read_reg(reg)&mask) - (uint32_t)(desired_value&mask);
    }
        pr_info("Value of LSR regsiter after waiting: %x\n",Read_reg(reg));

}
int check = 0;
void Read(uint8_t*buffer,int buff_size,uint32_t timeout){
    unsigned long time_mark = msecs_to_jiffies(timeout) + jiffies;
    int size = 0;
    uint32_t tempt = 0;
    while(size < buff_size && time_before(jiffies,time_mark)){
         /* if(Read_reg(SOC_UART_1_REGS + UART_LSR)&0x00000001u){
            buffer[size] = Read_reg(SOC_UART_1_REGS + UART_RHR) & (0x000000FFu);
            pr_info("Value of INTC_SIR_IRQ: %x\n",Read_reg(SOC_AINTC_REGS + INTC_SIR_IRQ));
            pr_info("Value of IIR register: %x\n",Read_reg(SOC_UART_1_REGS + UART_IIR));
            size++;
            time_mark = msecs_to_jiffies(timeout) + jiffies;
        }  */
         tempt = Read_reg(SOC_UART_1_REGS + UART_IIR);
        if((tempt & (0x01u << 2)) && !check){
            pr_info("Got it\n");
            pr_info("Value of UART_IIR: %x\n",Read_reg(SOC_UART_1_REGS + UART_IIR));
            pr_info("Value of INTC_SIR_IRQ: %x\n",Read_reg(SOC_AINTC_REGS + INTC_SIR_IRQ));
            check = 1;
        }
    }
}
void Send(uint8_t data){
    //Debug(UART_LSR_TX_FIFO_E,SOC_UART_1_REGS + UART_LSR,UART_LSR_TX_FIFO_E_MASK,__LINE__,time);
    Wait(UART_LSR_TX_FIFO_E,SOC_UART_1_REGS + UART_LSR,UART_LSR_TX_FIFO_E_MASK,__LINE__,time);
    Write_reg(data,SOC_UART_1_REGS + UART_THR,0x000000FFu);
    pr_info("Value IIR register : %x\n",Read_reg(SOC_UART_1_REGS + UART_IIR));
    //Debug(0x00u,SOC_UART_1_REGS + UART_LSR,0x00000001u << 6,__LINE__,time)f
}
uint32_t Read_reg(uint32_t physic_add){
    tempt = ioremap(physic_add,1000);
    return readl_relaxed(tempt);
}
void Write_reg(uint32_t desired_value,uint32_t physic_add,uint32_t mask){
    tempt = ioremap(physic_add,1000);
    uint32_t tempt3 = readl_relaxed(tempt); // value of register
    uint32_t tempt1 = (~mask)&tempt3;
    uint32_t tempt2 = mask&desired_value;
    tempt3 = tempt1|tempt2;
    writel_relaxed(tempt3,tempt);
}
void Write_reg_noMask(uint32_t desired_value,uint32_t physic_add){
    tempt = ioremap(physic_add,1000);
    writel_relaxed(desired_value,tempt);
}
void Debug(uint32_t desired_value,uint32_t reg,uint32_t mask,int line,int minitime){
    uint32_t tempt =1;
    unsigned long sectime = msecs_to_jiffies(minitime)+jiffies; // get the root
    while((tempt!=0)&&time_before(jiffies,sectime)){
        tempt = (uint32_t)(Read_reg(reg)&mask) - (uint32_t)(desired_value&mask);
    }
    if(tempt != 0)
        pr_info("Can not get desired value at %d, value of register: %x\n",line,Read_reg(reg));
    else
        pr_info("Completely config at line: %d, value: %x\n",line,Read_reg(reg));

}
void Debug_noMask(uint32_t desired_value,uint32_t reg,int line,int minitime){
    uint32_t tempt =1;
    unsigned long sectime = msecs_to_jiffies(minitime)+jiffies; // get the root
    while((tempt!=0)&&time_before(jiffies,sectime)){
        tempt = (uint32_t)(Read_reg(reg) - (uint32_t)(desired_value));
    }
    if(tempt != 0)
        pr_info("Can not get desired value at %d, value of register: %x\n",line,Read_reg(reg));
    else
        pr_info("Completely config at line: %d, value: %x\n",line,Read_reg(reg));
}
void UARTIrqEnable(int instance){
    // Enable IRQ interrupt
    asm("    mrs     r0, CPSR\n\t"
        "    bic     r0, r0, #0x80\n\t"
        "    msr     CPSR_c, r0");
    // Reset INTC module
    /* Write_reg(0x00000001u << 1, SOC_AINTC_REGS + INTC_SYSCONFIG, 0x00000001u << 1);
    // Debug(0x00000000u,SOC_AINTC_REGS + INTC_SYSSTATUS,0x00000001u,__LINE__,time); */

    // Disable threshold of interrupt
    Write_reg(INTC_THRESHOLD_PRIORITYTHRESHOLD,SOC_AINTC_REGS + INTC_THRESHOLD,0x000000FFu);
    Debug(INTC_THRESHOLD_PRIORITYTHRESHOLD,SOC_AINTC_REGS + INTC_THRESHOLD,0x000000FFu,__LINE__,time);

    // Set the priority for UART1
    Write_reg((0<<INTC_ILR_PRIORITY_SHIFT)| 0x00000000u,SOC_AINTC_REGS + INTC_ILR(SYS_INT_UART1INT),0x1FFu);
    Debug((0<<INTC_ILR_PRIORITY_SHIFT)| 0x00000000u,SOC_AINTC_REGS + INTC_ILR(SYS_INT_UART1INT),0x1FFu,__LINE__,time);

    // Clear mask bit of UART2
    Write_reg(0x01u << 10,SOC_AINTC_REGS + 0xc8u,0x01u << 10);
    asm(" dsb");
    Debug(0x00000000u << 10, SOC_AINTC_REGS + 0xc4u,0x00000001u << 10, __LINE__, time);
}