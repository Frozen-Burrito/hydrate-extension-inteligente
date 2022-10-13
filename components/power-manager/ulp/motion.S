/*
 * Como los archivos de ensamblador para el ULP pasan por el pre-procesador
 * de C antes de ser compilados, es posible usar directivas "include" y macros
 * en este archivo. 
*/
#include <soc/rtc_cntl_reg.h>
#include <soc/soc_ulp.h>

    /* Definir variables en la seccion .bss (zero-init data) */ 
    .bss

    /* El codigo debe estar en la seccion ".text" */
    .text
    .global entry
entry:

    /* TODO: programa de ULP para percibir movimiento */
    .global exit
exit:
    halt

    .global wake_up
wake_up:
    /* Revisar si el sistema puede ser despertado de deep sleep. */
    READ_RTC_FIELD(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP)

    and r0, r0, 1
    jump exit, eq

    /* Despertar el SoC, terminar el programa. */
    wake
    WRITE_RTC_FIELD(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN, 0)
    halt
