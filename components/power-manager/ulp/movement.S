/*
 * Como los archivos de ensamblador para el ULP pasan por el pre-procesador
 * de C antes de ser compilados, es posible usar directivas "include" y macros
 * en este archivo. 
*/
#include <soc/rtc_cntl_reg.h>
#include <soc/soc_ulp.h>

wake_up:
    /* Revisar si el sistema puede ser despertado de deep sleep. */
    wake
    halt
