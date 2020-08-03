#ifndef _HARDWAREX_COMM_N_TIMER_H_
#define _HARDWAREX_COMM_N_TIMER_H_



//! \file   HardwareX_comm_N_timer.h
//! \brief	Encapsulate HAL settings for
//! 		haptic_joint.c and anti_cogging.c



// **************************************************************************
// the includes
#include "sw/modules/types/src/types.h"
#include "sw/drivers/sci/src/32b/f28x/f2806x/sci.h"
#include "sw/drivers/gpio/src/32b/f28x/f2806x/gpio.h"
#include "sw/modules/math/src/32b/math.h"
#include "sw/modules/hal/boards/boostxldrv8305_revA/f28x/f2806x/src/hal_obj.h"



//!
//! \defgroup HARDWAREX_COMM_N_TIMER_H

//!
//! \ingroup HARDWAREX_COMM_N_TIMER_H
//@{

#ifdef __cplusplus
extern "C" {
#endif

// **************************************************************************
// the defines

// **************************************************************************

//! \brief Defines the commNtimer object
//!
typedef struct _COMM_N_TIMER_Obj_
{
    SCI_Handle sciBHandle;      //!< the SCI handle
} COMM_N_TIMER_Obj;

//! \brief Defines the commNtimer handle
typedef struct _COMM_N_TIMER_Obj_ *COMM_N_TIMER_Handle;


//! \breif externally defines the commNtimer object
extern COMM_N_TIMER_Obj COMM_N_TIMER;


// **************************************************************************
// the function prototypes

//! \brief Initializes the COMM_N_TIMER object
//! \param[in] pMemory      Memory pointer to object
//! \param[in] numBytes     Object size
//! \return                 Object handle
COMM_N_TIMER_Handle COMM_N_TIMER_init(void *pMemory,const size_t numBytes);


//! \brief Instantiate COMM_N_TIMER object parameters
//! \param[in] commNtimerHandle         Handle to COMM_N_TIMER object
void COMM_N_TIMER_setup(COMM_N_TIMER_Handle commNtimerHandle);


//! \brief Enables the SCI interrupts
//! \param[in] commNtimerHandle         Handle to COMM_N_TIMER object, the hardware abstraction (HAL) handle
extern void COMM_N_TIMER_enableSciInts(COMM_N_TIMER_Handle commNtimerHandle, HAL_Handle handle);

//! \brief Enables the Timer 0 interrupt
//! \param[in] pieHandle                The peripheral interrupt expansion (PIE) handle
extern void COMM_N_TIMER_PIE_enableTimer0Int(PIE_Handle pieHandle);

//! \brief get QEP index count number
//! \param[in] handle                   the hardware abstraction (HAL) handle
static inline uint32_t COMM_N_TIMER_getQepIndxCounts(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;
    QEP_Obj *qep = (QEP_Obj *)obj->qepHandle[0];

    return qep->QPOSILAT;
}

static inline void COMM_N_TIMER_setQepPosnCounts(HAL_Handle handle, uint32_t value)
{
    HAL_Obj *obj = (HAL_Obj *)handle;
    QEP_Obj *qep = (QEP_Obj *)obj->qepHandle[0];

    qep->QPOSCNT = value;
}

#ifdef __cplusplus
}
#endif // extern "C"

//@}  // ingroup

#endif // end of _HARDWAREX_COMM_N_TIMER_H_ definition
