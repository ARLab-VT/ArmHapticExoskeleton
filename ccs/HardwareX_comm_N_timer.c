//! \file	HardwareX_comm_N_timer.c
//! \brief	Encapsulate HAL settings of sci communication(Sci, B)
//! 		for haptic_joint.c and anti_cogging.c

// **************************************************************************
// the includes

#include "sw/solutions/instaspin_motion/src/HardwareX_comm_N_timer.h"

// **************************************************************************
// the globals

COMM_N_TIMER_Obj COMM_N_TIMER;

// **************************************************************************
// the functions

COMM_N_TIMER_Handle COMM_N_TIMER_init(void *pMemory,const size_t numBytes)
{
    COMM_N_TIMER_Handle commNtimerHandle;

  if(numBytes < sizeof(COMM_N_TIMER_Obj))
    return((COMM_N_TIMER_Handle)NULL);

  // assign the handle
  commNtimerHandle = (COMM_N_TIMER_Handle)pMemory;

  return(commNtimerHandle);
} // end of COMM_N_TIMER_init() function


void COMM_N_TIMER_setup(COMM_N_TIMER_Handle commNtimerHandle)
{
    COMM_N_TIMER_Obj *commNtimer = (COMM_N_TIMER_Obj *) commNtimerHandle;

    // initialize the sci B channel
    commNtimer->sciBHandle = SCI_init((void *)SCIB_BASE_ADDR, sizeof(SCI_Obj));

    //setup serial communication
    SCI_reset(commNtimer->sciBHandle);
    SCI_disableParity(commNtimer->sciBHandle);
    SCI_setNumStopBits(commNtimer->sciBHandle,SCI_NumStopBits_One);
    SCI_setCharLength(commNtimer->sciBHandle,SCI_CharLength_8_Bits);
    SCI_disableLoopBack(commNtimer->sciBHandle);

    //registry: SCICTL1
    SCI_enableTx(commNtimer->sciBHandle);
    SCI_enableRx(commNtimer->sciBHandle);
    SCI_disableSleep(commNtimer->sciBHandle);
    SCI_disableTxWake(commNtimer->sciBHandle);

    //registry: SCICTL2
    SCI_disableRxInt(commNtimer->sciBHandle);

    // set baud rate to 57600, refer to spruh18g, Table 13-11
    // under 60 MMz -> 194 is equivalent to SCI_BaudRate_9_6_kBaud.
    // under 90 MHz -> 194 is 57600
    SCI_setBaudRate(commNtimer->sciBHandle,SCI_BaudRate_9_6_kBaud);

    SCI_disableRxErrorInt(commNtimer->sciBHandle);
    SCI_setTxDelay(commNtimer->sciBHandle, 0);
    SCI_setPriority(commNtimer->sciBHandle,SCI_Priority_FreeRun);
    SCI_enable(commNtimer->sciBHandle);

    // enabling the peripheral clocks (followings are written by default in hal.c):
    // CLK_enableScibClock(obj->clkHandle);
    // GPIO_setMode(obj->gpioHandle,GPIO_Number_15,GPIO_15_Mode_SCIRXDB);
    // GPIO_setMode(obj->gpioHandle,GPIO_Number_58,GPIO_58_Mode_SCITXDB);

    return;
} // end of COMM_N_TIMER_setup() function

void COMM_N_TIMER_enableSciInts(COMM_N_TIMER_Handle commNtimerHandle, HAL_Handle handle)
{
    COMM_N_TIMER_Obj *commNtimer = (COMM_N_TIMER_Obj *) commNtimerHandle;
    HAL_Obj *obj = (HAL_Obj *)handle;

    // enable the PIE interrupts associated with the SCI interrupts
    // enable SCIB RX interrupt in PIE
    PIE_enableInt(obj->pieHandle,PIE_GroupNumber_9,PIE_InterruptSource_SCIBRX);

    // enable SCI RX interrupts
    // enable SCIB RX interrupt
    SCI_enableRxInt(commNtimer->sciBHandle);

    // enable the cpu interrupt for SCI interrupts
    CPU_enableInt(obj->cpuHandle,CPU_IntNumber_9);
} // end of COMM_N_TIMER_enableSciInts() function


void COMM_N_TIMER_PIE_enableTimer0Int(PIE_Handle pieHandle)
{
    PIE_Obj *pie = (PIE_Obj *)pieHandle;

    // set the value
    pie->PIEIER_PIEIFR[0].IER |= PIE_IERx_INTx7_BITS;

    return;
} // end of COMM_N_TIMER_PIE_enableTimer0Int() function

// end of file
