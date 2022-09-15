
/*
 * Auto generated Run-Time-Environment Configuration File
 *      *** Do not modify ! ***
 *
 * Project: 'Blinky' 
 * Target:  'S32K148 Flash' 
 */

#ifndef RTE_COMPONENTS_H
#define RTE_COMPONENTS_H


/*
 * Define the Device Header File: 
 */
#define CMSIS_device_header "device_registers.h"

/*  ARM::CMSIS:RTOS2:Keil RTX5:Source:5.5.1 */
//#define RTE_CMSIS_RTOS2                 /* CMSIS-RTOS2 */
        //#define RTE_CMSIS_RTOS2_RTX5            /* CMSIS-RTOS2 Keil RTX5 */
        //#define RTE_CMSIS_RTOS2_RTX5_SOURCE     /* CMSIS-RTOS2 Keil RTX5 Source */
/*  Keil.ARM Compiler::Compiler:Event Recorder:DAP:1.4.0 */
/*#define RTE_Compiler_EventRecorder*/
/*#define RTE_Compiler_EventRecorder_DAP*/


#include "S32K148.h"
#include "core_cm4.h" 
#include "cmsis_armclang.h"

#define RTE_Network_RTOS

#define RTE_Network_IPv4
#define RTE_Network_Interface_Ethernet

#define LWIP_NOASSERT    

#define EXCLUSIVE_ACCESS 1

#endif /* RTE_COMPONENTS_H */
