/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : lpit1.c
**     Project     : S32K144_FlexCAN_RXFIFO_SDK_RTM300_AVM
**     Processor   : S32K144_48
**     Component   : lpit
**     Version     : Component SDK_S32K1xx_15, Driver 01.00, CPU db: 3.00.000
**     Repository  : SDK_S32K1xx_15
**     Compiler    : GNU C Compiler
**     Date/Time   : 2025-01-14, 15:53, # CodeGen: 97
**
**     Copyright 1997 - 2015 Freescale Semiconductor, Inc. 
**     Copyright 2016-2017 NXP 
**     All Rights Reserved.
**     
**     THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
**     IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
**     OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
**     IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
**     INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
**     SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
**     HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
**     STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
**     IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
**     THE POSSIBILITY OF SUCH DAMAGE.
** ###################################################################*/
/*!
** @file lpit1.c
** @version 01.00
*/         
/*!
**  @addtogroup lpit1_module lpit1 module documentation
**  @{
*/         

/* MODULE lpit1.
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External variable could be made static.
 * The external variables will be used in other source files in application code.
 */

#include "lpit1.h"

/*! Global configuration of lpit1 */
const lpit_user_config_t lpit1_InitConfig =
{
    .enableRunInDebug = false,         /*!< true: LPIT run in debug mode; false: LPIT stop in debug mode */
    .enableRunInDoze = false           /*!< true: LPIT run in doze mode; false: LPIT stop in doze mode */
};

/*! User channel configuration 0 */
lpit_user_channel_config_t lpit1_ChnConfig0 =
{
    .timerMode = LPIT_PERIODIC_COUNTER,
    .periodUnits = LPIT_PERIOD_UNITS_MICROSECONDS,
    .period = 1000U,
    .triggerSource = LPIT_TRIGGER_SOURCE_EXTERNAL,
    .triggerSelect = 0U,
    .enableReloadOnTrigger = false,
    .enableStopOnInterrupt = false,
    .enableStartOnTrigger = false,
    .chainChannel = false,
    .isInterruptEnabled = true
};
/* END lpit1. */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.1 [05.21]
**     for the Freescale S32K series of microcontrollers.
**
** ###################################################################
*/

