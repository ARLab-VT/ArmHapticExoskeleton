/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, LineStream Technologies Incorporated
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
* *  Neither the names of Texas Instruments Incorporated, LineStream
 *    Technologies Incorporated, nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//! \file   solutions/instaspin_motion/src/proj_lab12b.c
//! \brief  SpinTAC Velocity Controller using a quadrature encoder for feedback
//!
//! (C) Copyright 2012, LineStream Technologies, Inc.
//! (C) Copyright 2011, Texas Instruments, Inc.

//! \defgroup PROJ_LAB12b PROJ_LAB12b
//@{

//! \defgroup PROJ_LAB12b_OVERVIEW Project Overview
//!
//! SpinTAC Velocity Controller using a quadrature encoder for feedback
//!

// **************************************************************************
// the includes

// system includes
#include <math.h>
#include "main.h"
#include "sw/solutions/instaspin_motion/src/HardwareX_comm_N_timer.h" // modified by Hubert Kim
#include "sw/solutions/instaspin_motion/src/anticogging_lookupTable.h" // modified by Hubert Kim


#ifdef FLASH
#pragma CODE_SECTION(mainISR,"ramfuncs");
#endif

// Include header files used in the main function


// **************************************************************************
// the defines

#define LED_BLINK_FREQ_Hz   5

// custom defines
#define dLogSize 10                   // number of byte to sent back to user
#define SCALE_FACTOR 1000             // scaling up for the resolution in the communication.
#define EXTRA_SCALE_FACTOR 10000

// toggles
//#define AngleResetViaIndxPin_Dbug   // debugging process for the encoder reset. Once calibrated turn off
#define AngleResetViaIndxPin        // encoder reading reset with the arm swing.(active)
//#define posRev_1_currA_0
#define sensor_1stOrderFiltered       // Data Logging purpose: 1st order filter on the sensor reading to make them stable
//#define Anticogged
// **************************************************************************
// the globals

uint_least16_t gCounter_updateGlobals = 0;

bool Flag_Latch_softwareUpdate = true;

CTRL_Handle ctrlHandle;

HAL_Handle halHandle;

USER_Params gUserParams;

HAL_PwmData_t gPwmData = {_IQ(0.0), _IQ(0.0), _IQ(0.0)};

HAL_AdcData_t gAdcData;

_iq gMaxCurrentSlope = _IQ(0.0);

#ifdef FAST_ROM_V1p6
CTRL_Obj *controller_obj;
#else
CTRL_Obj ctrl;				//v1p7 format
#endif

ENC_Handle encHandle;
ENC_Obj enc;

SLIP_Handle slipHandle;
SLIP_Obj slip;

ST_Obj st_obj;
ST_Handle stHandle;

COMM_N_TIMER_Handle commNtimerHandle; // added by Hubert Kim

#ifdef sensor_1stOrderFiltered
FILTER_FO_Handle filterHandle[2]; //!< the handle for the current and torque reading
FILTER_FO_Obj    filter[2];
#endif

uint16_t gLEDcnt = 0;

volatile MOTOR_Vars_t gMotorVars = MOTOR_Vars_INIT;

#ifdef FLASH
// Used for running BackGround in flash, and ISR in RAM
extern uint16_t *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;
#endif


#ifdef DRV8301_SPI
// Watch window interface to the 8301 SPI
DRV_SPI_8301_Vars_t gDrvSpi8301Vars;
#endif
#ifdef DRV8305_SPI
// Watch window interface to the 8305 SPI
DRV_SPI_8305_Vars_t gDrvSpi8305Vars;
#endif

_iq gFlux_pu_to_Wb_sf;

_iq gFlux_pu_to_VpHz_sf;

_iq gTorque_Ls_Id_Iq_pu_to_Nm_sf;

_iq gTorque_Flux_Iq_pu_to_Nm_sf;


// custom variables, added by Hubert Kim
_iq speedFeedback, iqReference = 0;

volatile uint16_t *ArrayPtrs4DLog[dLogSize];
static volatile uint_least8_t cntr = 0; // array index counter for sending the parsal serial
const uint16_t datID_1stPckt = 'f';     //
const uint16_t datID_2ndPckt = 's';     // 0x73
const uint16_t datID_3rdPckt = 't';     // 0x74
const uint16_t datID_4thPckt = 'F';
const uint16_t datID_timerCount = 'T';  // 0x54
static volatile uint16_t timer0cnt;   // timer0 time stamp
volatile uint16_t *ptr_ArrayPtrs4DLog;// point for the array of pointer
static volatile uint16_t dataGet = 0;
static volatile bool assy16BitDat = 0;
static volatile int16_t assembledCMD = 0;
static volatile bool HL_t = 0;
static volatile uint16_t LSB = 0;
static volatile int16_t MSB = 0;
uint16_t success;
static volatile bool HL = 0;        // boolean toggle for little endianness
static volatile int16_t PKG_ONE = 0; // variables store the converted readings before sending
static volatile int16_t PKG_TWO = 0;
static volatile int16_t PKG_THREE = 0;
static volatile int16_t PKG_FOUR = 0;
static volatile int_least8_t data = 0;
static volatile bool flag_updatePkg = 0;
static volatile int16_t debugVariable = 0;

#ifdef AngleResetViaIndxPin
#ifdef AngleResetViaIndxPin_Dbug // variables for the debugging purpose
uint32_t debug_IndxCnt_1 = 0;
uint32_t debug_QepPosCnts = 0;
uint32_t debug_QepPosMax = 0;
uint32_t debug_zeroOffset = 0;
#endif
bool mtrPosInitiated = 0;
static volatile _iq mtrPosMrev_iq = _IQ(0.0);
static volatile _iq reCnstrctd_pos_erev = _IQ(0);
static volatile _iq temporary_pos_conv = _IQ(0);
#endif

#ifdef sensor_1stOrderFiltered
volatile _iq gSensorFilterPole_Hz = _IQ(5);// 5 Hz is the highest cutoff frequency for current sampling freq. (100Hz) with 20dB reduction at Nyquist freq.
volatile int16_t samplingFrq = 100; // the same as the timer0ISR() speed
#endif

bool initExperiment = 0;
_iq tuning_KP = _IQ(2);
_iq tuning_KD = _IQ(-0.15);
_iq TargetPos_iq = 0;
_iq PosError_iq = 0;
//_iq Imax = _IQ(0.6); // 0.6 corresponds to 14.4 A, very close to 15 A (rated maximum current)
//_iq Imin = _IQ(-0.6);
_iq Imax = _IQ(0.4); // 0.4 corresponds to 9.6 A
_iq Imin = _IQ(-0.4);

static volatile uint32_t mtrPos = 0;
uint32_t maxPos = 0;
static volatile float I_compensation = 0;
// **************************************************************************
// the function prototypes

void updateDLogPkg();

#ifdef AngleResetViaIndxPin
void posOffsetConv(HAL_Handle halHandle);
#ifdef AngleResetViaIndxPin_Dbug
#else
void runRsOnLine(CTRL_Handle);
#endif
#endif

// **************************************************************************
// the functions

void main(void)
{
  uint_least8_t estNumber = 0;

#ifdef FAST_ROM_V1p6
  uint_least8_t ctrlNumber = 0;
#endif

  // Only used if running from FLASH
  // Note that the variable FLASH is defined by the project
  #ifdef FLASH
  // Copy time critical code and Flash setup code to RAM
  // The RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
  // symbols are created by the linker. Refer to the linker files.
  memCopy((uint16_t *)&RamfuncsLoadStart,(uint16_t *)&RamfuncsLoadEnd,(uint16_t *)&RamfuncsRunStart);
  #endif

  // initialize the hardware abstraction layer
  halHandle = HAL_init(&hal,sizeof(hal));
  commNtimerHandle = COMM_N_TIMER_init(&COMM_N_TIMER, sizeof(COMM_N_TIMER)); // added by Hubert Kim

  // check for errors in user parameters
  USER_checkForErrors(&gUserParams);


  // store user parameter error in global variable
  gMotorVars.UserErrorCode = USER_getErrorCode(&gUserParams);


  // do not allow code execution if there is a user parameter error
  if(gMotorVars.UserErrorCode != USER_ErrorCode_NoError)
    {
      for(;;)
        {
          gMotorVars.Flag_enableSys = false;
        }
    }


  // initialize the user parameters
  USER_setParams(&gUserParams);


  // set the hardware abstraction layer parameters
  HAL_setParams(halHandle,&gUserParams);
  COMM_N_TIMER_setup(commNtimerHandle); //added by Hubert Kim


  // initialize the controller
#ifdef FAST_ROM_V1p6
  ctrlHandle = CTRL_initCtrl(ctrlNumber, estNumber);  		//v1p6 format (06xF and 06xM devices)
  controller_obj = (CTRL_Obj *)ctrlHandle;
#else
  ctrlHandle = CTRL_initCtrl(estNumber,&ctrl,sizeof(ctrl));	//v1p7 format default
#endif


  {
    CTRL_Version version;

    // get the version number
    CTRL_getVersion(ctrlHandle,&version);

    gMotorVars.CtrlVersion = version;
  }


  // set the default controller parameters
  CTRL_setParams(ctrlHandle,&gUserParams);


  // setup faults
  HAL_setupFaults(halHandle);


  // initialize the interrupt vector table
  HAL_initIntVectorTable(halHandle);


  // enable the ADC interrupts
  HAL_enableAdcInts(halHandle);

  // enable sci interrupts
  COMM_N_TIMER_enableSciInts(commNtimerHandle, halHandle); // added by Hubert Kim

  // enable global interrupts
  HAL_enableGlobalInts(halHandle);


  // enable debug interrupts
  HAL_enableDebugInt(halHandle);


  // disable the PWM
  HAL_disablePwm(halHandle);


  // initialize the ENC module
  encHandle = ENC_init(&enc, sizeof(enc));


  // setup the ENC module
  ENC_setup(encHandle, 1, USER_MOTOR_NUM_POLE_PAIRS, USER_MOTOR_ENCODER_LINES, 0, USER_IQ_FULL_SCALE_FREQ_Hz, USER_ISR_FREQ_Hz, 8000.0);


  // initialize the SLIP module
  slipHandle = SLIP_init(&slip, sizeof(slip));


  // setup the SLIP module
  SLIP_setup(slipHandle, _IQ(gUserParams.ctrlPeriod_sec));


  // initialize the SpinTAC Components
  stHandle = ST_init(&st_obj, sizeof(st_obj));
  
  
  // setup the SpinTAC Components
  ST_setupVelCtl(stHandle);
  ST_setupPosConv(stHandle);


#ifdef DRV8301_SPI
  // turn on the DRV8301 if present
  HAL_enableDrv(halHandle);
  // initialize the DRV8301 interface
  HAL_setupDrvSpi(halHandle,&gDrvSpi8301Vars);
#endif

#ifdef DRV8305_SPI
  // turn on the DRV8305 if present
  HAL_enableDrv(halHandle);
  // initialize the DRV8305 interface
  HAL_setupDrvSpi(halHandle,&gDrvSpi8305Vars);
#endif

  // enable DC bus compensation
  CTRL_setFlag_enableDcBusComp(ctrlHandle, true);


  // initialize the filter
#ifdef  sensor_1stOrderFiltered
  {
      uint16_t cnt = 0;

      // 5 Hz cutoff frequency, Fs is the timer0ISR.
      _iq b0 = _IQmpy(gSensorFilterPole_Hz, _IQ(1.0/samplingFrq));
      _iq b1 = _IQ(0.0);
      _iq a1 = b0 - _IQ(1.0);

      for(cnt = 0;cnt<2;cnt++)
      {
          filterHandle[cnt] = FILTER_FO_init(&filter[cnt],sizeof(filter[0]));
          FILTER_FO_setDenCoeffs(filterHandle[cnt],a1);
          FILTER_FO_setNumCoeffs(filterHandle[cnt],b0,b1);
          FILTER_FO_setInitialConditions(filterHandle[cnt],_IQ(0.0),_IQ(0.0));
      }
  }
#endif

  // compute scaling factors for flux and torque calculations
  gFlux_pu_to_Wb_sf = USER_computeFlux_pu_to_Wb_sf();
  gFlux_pu_to_VpHz_sf = USER_computeFlux_pu_to_VpHz_sf();
  gTorque_Ls_Id_Iq_pu_to_Nm_sf = USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf();
  gTorque_Flux_Iq_pu_to_Nm_sf = USER_computeTorque_Flux_Iq_pu_to_Nm_sf();


  // assign data to read into the array of pointers
  ArrayPtrs4DLog[0] = (uint16_t *)&datID_1stPckt;
  ArrayPtrs4DLog[1] = (uint16_t *)&PKG_ONE;
  ArrayPtrs4DLog[2] = (uint16_t *)&datID_2ndPckt;
  ArrayPtrs4DLog[3] = (uint16_t *)&PKG_TWO;
  ArrayPtrs4DLog[4] = (uint16_t *)&datID_3rdPckt;
  ArrayPtrs4DLog[5] = (uint16_t *)&PKG_THREE;
  ArrayPtrs4DLog[6] = (uint16_t *)&datID_4thPckt;
  ArrayPtrs4DLog[7] = (uint16_t *)&PKG_FOUR;
  ArrayPtrs4DLog[8] = (uint16_t *)&datID_timerCount;
  ArrayPtrs4DLog[9] = (uint16_t *)&timer0cnt; // for the anti-cogging test, we log the position command instead


  for(;;)
  {
    // Waiting for enable system flag to be set
    while(!(gMotorVars.Flag_enableSys));

    // Dis-able the Library internal PI.  Iq has no reference now
    CTRL_setFlag_enableSpeedCtrl(ctrlHandle, false);

    // loop while the enable system flag is true
    while(gMotorVars.Flag_enableSys)
      {
        CTRL_Obj *obj = (CTRL_Obj *)ctrlHandle;
        ST_Obj *stObj = (ST_Obj *)stHandle;

        // increment counters
        gCounter_updateGlobals++;

        // enable/disable the use of motor parameters being loaded from user.h
        CTRL_setFlag_enableUserMotorParams(ctrlHandle,gMotorVars.Flag_enableUserParams);

        // enable/disable Rs recalibration during motor startup
        EST_setFlag_enableRsRecalc(obj->estHandle,gMotorVars.Flag_enableRsRecalc);

        // enable/disable automatic calculation of bias values
        CTRL_setFlag_enableOffset(ctrlHandle,gMotorVars.Flag_enableOffsetcalc);


        if(CTRL_isError(ctrlHandle))
          {
            // set the enable controller flag to false
            CTRL_setFlag_enableCtrl(ctrlHandle,false);

            // set the enable system flag to false
            gMotorVars.Flag_enableSys = false;

            // disable the PWM
            HAL_disablePwm(halHandle);
          }
        else
          {
            // update the controller state
            bool flag_ctrlStateChanged = CTRL_updateState(ctrlHandle);

            // enable or disable the control
            CTRL_setFlag_enableCtrl(ctrlHandle, gMotorVars.Flag_Run_Identify);

            if(flag_ctrlStateChanged)
              {
                CTRL_State_e ctrlState = CTRL_getState(ctrlHandle);

                if(ctrlState == CTRL_State_OffLine)
                  {
                    // enable the PWM
                    HAL_enablePwm(halHandle);
                  }
                else if(ctrlState == CTRL_State_OnLine)
                  {
                    if(gMotorVars.Flag_enableOffsetcalc == true)
                    {
                      // update the ADC bias values
                      HAL_updateAdcBias(halHandle);
                    }
                    else
                    {
                      // set the current bias
                      HAL_setBias(halHandle,HAL_SensorType_Current,0,_IQ(I_A_offset));
                      HAL_setBias(halHandle,HAL_SensorType_Current,1,_IQ(I_B_offset));
                      HAL_setBias(halHandle,HAL_SensorType_Current,2,_IQ(I_C_offset));

                      // set the voltage bias
                      HAL_setBias(halHandle,HAL_SensorType_Voltage,0,_IQ(V_A_offset));
                      HAL_setBias(halHandle,HAL_SensorType_Voltage,1,_IQ(V_B_offset));
                      HAL_setBias(halHandle,HAL_SensorType_Voltage,2,_IQ(V_C_offset));
                    }

                    // Return the bias value for currents
                    gMotorVars.I_bias.value[0] = HAL_getBias(halHandle,HAL_SensorType_Current,0);
                    gMotorVars.I_bias.value[1] = HAL_getBias(halHandle,HAL_SensorType_Current,1);
                    gMotorVars.I_bias.value[2] = HAL_getBias(halHandle,HAL_SensorType_Current,2);

                    // Return the bias value for voltages
                    gMotorVars.V_bias.value[0] = HAL_getBias(halHandle,HAL_SensorType_Voltage,0);
                    gMotorVars.V_bias.value[1] = HAL_getBias(halHandle,HAL_SensorType_Voltage,1);
                    gMotorVars.V_bias.value[2] = HAL_getBias(halHandle,HAL_SensorType_Voltage,2);

                    // enable the PWM
                    HAL_enablePwm(halHandle);
                  }
                else if(ctrlState == CTRL_State_Idle)
                  {
                    // disable the PWM
                    HAL_disablePwm(halHandle);
                    gMotorVars.Flag_Run_Identify = false;
                  }

                if((CTRL_getFlag_enableUserMotorParams(ctrlHandle) == true) &&
                  (ctrlState > CTRL_State_Idle) &&
                  (gMotorVars.CtrlVersion.minor == 6))
                  {
                    // call this function to fix 1p6
                    USER_softwareUpdate1p6(ctrlHandle);
                  }

              }
          }


        if(EST_isMotorIdentified(obj->estHandle))
          {
            // set the current ramp
            EST_setMaxCurrentSlope_pu(obj->estHandle,gMaxCurrentSlope);
            gMotorVars.Flag_MotorIdentified = true;

            // set the speed reference
            CTRL_setSpd_ref_krpm(ctrlHandle,gMotorVars.SpeedRef_krpm);

            // set the speed acceleration
            CTRL_setMaxAccel_pu(ctrlHandle,_IQmpy(MAX_ACCEL_KRPMPS_SF,gMotorVars.MaxAccel_krpmps));

            // enable the SpinTAC Speed Controller
            STVELCTL_setEnable(stObj->velCtlHandle, true);

            if(EST_getState(obj->estHandle) != EST_State_OnLine)
            {
            	// if the estimator is not running, place SpinTAC into reset
            	STVELCTL_setEnable(stObj->velCtlHandle, false);
            }

            if(Flag_Latch_softwareUpdate)
            {
              Flag_Latch_softwareUpdate = false;

              USER_calcPIgains(ctrlHandle);
			  
              // initialize the watch window kp and ki current values with pre-calculated values
              gMotorVars.Kp_Idq = CTRL_getKp(ctrlHandle,CTRL_Type_PID_Id);
              gMotorVars.Ki_Idq = CTRL_getKi(ctrlHandle,CTRL_Type_PID_Id);

			  // initialize the watch window Bw value with the default value
              gMotorVars.SpinTAC.VelCtlBw_radps = STVELCTL_getBandwidth_radps(stObj->velCtlHandle);

              // initialize the watch window with maximum and minimum Iq reference
              gMotorVars.SpinTAC.VelCtlOutputMax_A = _IQmpy(STVELCTL_getOutputMaximum(stObj->velCtlHandle), _IQ(USER_IQ_FULL_SCALE_CURRENT_A));
              gMotorVars.SpinTAC.VelCtlOutputMin_A = _IQmpy(STVELCTL_getOutputMinimum(stObj->velCtlHandle), _IQ(USER_IQ_FULL_SCALE_CURRENT_A));
            }

          }
        else
          {
            Flag_Latch_softwareUpdate = true;

            // the estimator sets the maximum current slope during identification
            gMaxCurrentSlope = EST_getMaxCurrentSlope_pu(obj->estHandle);
          }


        // when appropriate, update the global variables
        if(gCounter_updateGlobals >= NUM_MAIN_TICKS_FOR_GLOBAL_VARIABLE_UPDATE)
          {
            // reset the counter
            gCounter_updateGlobals = 0;

            updateGlobalVariables_motor(ctrlHandle, stHandle);
          }


        // update Kp and Ki gains
        updateKpKiGains(ctrlHandle);

        // set the SpinTAC (ST) bandwidth scale
        STVELCTL_setBandwidth_radps(stObj->velCtlHandle, gMotorVars.SpinTAC.VelCtlBw_radps);

        // set the maximum and minimum values for Iq reference
        STVELCTL_setOutputMaximums(stObj->velCtlHandle, _IQmpy(gMotorVars.SpinTAC.VelCtlOutputMax_A, _IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A)), _IQmpy(gMotorVars.SpinTAC.VelCtlOutputMin_A, _IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A)));

        // enable/disable the forced angle
        EST_setFlag_enableForceAngle(obj->estHandle,gMotorVars.Flag_enableForceAngle);

        // enable or disable power warp
        CTRL_setFlag_enablePowerWarp(ctrlHandle,gMotorVars.Flag_enablePowerWarp);

#ifdef DRV8301_SPI
        HAL_writeDrvData(halHandle,&gDrvSpi8301Vars);

        HAL_readDrvData(halHandle,&gDrvSpi8301Vars);
#endif
#ifdef DRV8305_SPI
        HAL_writeDrvData(halHandle,&gDrvSpi8305Vars);

        HAL_readDrvData(halHandle,&gDrvSpi8305Vars);
#endif
      } // end of while(gFlag_enableSys) loop


    // disable the PWM
    HAL_disablePwm(halHandle);

    // set the default controller parameters (Reset the control to re-identify the motor)
    CTRL_setParams(ctrlHandle,&gUserParams);
    gMotorVars.Flag_Run_Identify = false;
	
    // setup the SpinTAC Components
    ST_setupVelCtl(stHandle);
    ST_setupPosConv(stHandle);

  } // end of for(;;) loop

} // end of main() function


interrupt void mainISR(void)
{

  static uint16_t stCnt = 0;
  CTRL_Obj *obj = (CTRL_Obj *)ctrlHandle;

  // toggle status LED
  if(++gLEDcnt >= (uint_least32_t)(USER_ISR_FREQ_Hz / LED_BLINK_FREQ_Hz))
  {
    HAL_toggleLed(halHandle,(GPIO_Number_e)HAL_Gpio_LED2);
    gLEDcnt = 0;
  }


  // compute the electrical angle
  ENC_calcElecAngle(encHandle, HAL_getQepPosnCounts(halHandle));


  // acknowledge the ADC interrupt
  HAL_acqAdcInt(halHandle,ADC_IntNumber_1);


  // convert the ADC data
  HAL_readAdcData(halHandle,&gAdcData);


  // Run the SpinTAC Components
  if(stCnt++ >= ISR_TICKS_PER_SPINTAC_TICK) {
	  ST_runPosConv(stHandle, encHandle, ctrlHandle);
#ifdef AngleResetViaIndxPin
      posOffsetConv(halHandle);        // update the re-calibrated encoder reading every cycle
#endif
      ST_runVelCtl(stHandle, ctrlHandle); // not running the motor while calibrating the position
	  stCnt = 1;
  }


//  if(USER_MOTOR_TYPE == MOTOR_Type_Induction) {
//    // update the electrical angle for the SLIP module
//    SLIP_setElectricalAngle(slipHandle, ENC_getElecAngle(encHandle));
//    // compute the amount of slip
//    SLIP_run(slipHandle);
//
//
//    // run the controller
//    CTRL_run(ctrlHandle,halHandle,&gAdcData,&gPwmData,SLIP_getMagneticAngle(slipHandle));
//  }
//  else {
//     run the controller

#ifdef AngleResetViaIndxPin
    #ifdef AngleResetViaIndxPin_Dbug
    CTRL_run(ctrlHandle,halHandle,&gAdcData,&gPwmData,ENC_getElecAngle(encHandle));
    #else
        if(mtrPosInitiated) // after index pin re-initialized.
            CTRL_run(ctrlHandle,halHandle,&gAdcData,&gPwmData,reCnstrctd_pos_erev);// after index pin reset
    #endif
#else
        CTRL_run(ctrlHandle,halHandle,&gAdcData,&gPwmData,ENC_getElecAngle(encHandle));
#endif
//  }

  // write the PWM compare values
  HAL_writePwmData(halHandle,&gPwmData);


  // setup the controller
  CTRL_setup(ctrlHandle);

  // if we are forcing alignment, using the Rs Recalculation, align the eQEP angle with the rotor angle
  if((EST_getState(obj->estHandle) == EST_State_Rs) && (USER_MOTOR_TYPE == MOTOR_Type_Pm))
  {
	  ENC_setZeroOffset(encHandle, (uint32_t)(HAL_getQepPosnMaximum(halHandle) - HAL_getQepPosnCounts(halHandle)));
  }

  return;
} // end of mainISR() function


void updateGlobalVariables_motor(CTRL_Handle handle, ST_Handle sthandle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  ST_Obj *stObj = (ST_Obj *)sthandle;


  // get the speed estimate
  gMotorVars.Speed_krpm = EST_getSpeed_krpm(obj->estHandle);

  // get the speed from eQEP
  gMotorVars.SpeedQEP_krpm = _IQmpy(STPOSCONV_getVelocityFiltered(stObj->posConvHandle), _IQ(ST_SPEED_KRPM_PER_PU));

  // get the real time speed reference coming out of the speed trajectory generator
  gMotorVars.SpeedTraj_krpm = _IQmpy(CTRL_getSpd_int_ref_pu(handle),EST_get_pu_to_krpm_sf(obj->estHandle));

  // get the torque estimate
  gMotorVars.Torque_Nm = USER_computeTorque_Nm(handle, gTorque_Flux_Iq_pu_to_Nm_sf, gTorque_Ls_Id_Iq_pu_to_Nm_sf);

  // get the magnetizing current
  gMotorVars.MagnCurr_A = EST_getIdRated(obj->estHandle);

  // get the rotor resistance
  gMotorVars.Rr_Ohm = EST_getRr_Ohm(obj->estHandle);

  // get the stator resistance
  gMotorVars.Rs_Ohm = EST_getRs_Ohm(obj->estHandle);

  // get the stator inductance in the direct coordinate direction
  gMotorVars.Lsd_H = EST_getLs_d_H(obj->estHandle);

  // get the stator inductance in the quadrature coordinate direction
  gMotorVars.Lsq_H = EST_getLs_q_H(obj->estHandle);

  // get the flux in V/Hz in floating point
  gMotorVars.Flux_VpHz = EST_getFlux_VpHz(obj->estHandle);

  // get the flux in Wb in fixed point
  gMotorVars.Flux_Wb = USER_computeFlux(handle, gFlux_pu_to_Wb_sf);

  // get the controller state
  gMotorVars.CtrlState = CTRL_getState(handle);

  // get the estimator state
  gMotorVars.EstState = EST_getState(obj->estHandle);

  // Get the DC buss voltage
  gMotorVars.VdcBus_kV = _IQmpy(gAdcData.dcBus,_IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/1000.0));

  // get the Iq reference from the speed controller
  gMotorVars.IqRef_A = _IQmpy(STVELCTL_getTorqueReference(stObj->velCtlHandle), _IQ(USER_IQ_FULL_SCALE_CURRENT_A));

  // gets the Velocity Controller status
  gMotorVars.SpinTAC.VelCtlStatus = STVELCTL_getStatus(stObj->velCtlHandle);

  // get the inertia setting
  gMotorVars.SpinTAC.InertiaEstimate_Aperkrpm = _IQmpy(STVELCTL_getInertia(stObj->velCtlHandle), _IQ(ST_SPEED_PU_PER_KRPM * USER_IQ_FULL_SCALE_CURRENT_A));

  // get the friction setting
  gMotorVars.SpinTAC.FrictionEstimate_Aperkrpm = _IQmpy(STVELCTL_getFriction(stObj->velCtlHandle), _IQ(ST_SPEED_PU_PER_KRPM * USER_IQ_FULL_SCALE_CURRENT_A));

  // get the Velocity Controller error
  gMotorVars.SpinTAC.VelCtlErrorID = STVELCTL_getErrorID(stObj->velCtlHandle);

  // get the Position Converter error
  gMotorVars.SpinTAC.PosConvErrorID = STPOSCONV_getErrorID(stObj->posConvHandle);

  return;
} // end of updateGlobalVariables_motor() function


void updateKpKiGains(CTRL_Handle handle)
{
  if((gMotorVars.CtrlState == CTRL_State_OnLine) && (gMotorVars.Flag_MotorIdentified == true) && (Flag_Latch_softwareUpdate == false))
    {
      // set the kp and ki speed values from the watch window
      CTRL_setKp(handle,CTRL_Type_PID_spd,gMotorVars.Kp_spd);
      CTRL_setKi(handle,CTRL_Type_PID_spd,gMotorVars.Ki_spd);

      // set the kp and ki current values for Id and Iq from the watch window
      CTRL_setKp(handle,CTRL_Type_PID_Id,gMotorVars.Kp_Idq);
      CTRL_setKi(handle,CTRL_Type_PID_Id,gMotorVars.Ki_Idq);
      CTRL_setKp(handle,CTRL_Type_PID_Iq,gMotorVars.Kp_Idq);
      CTRL_setKi(handle,CTRL_Type_PID_Iq,gMotorVars.Ki_Idq);
	}

  return;
} // end of updateKpKiGains() function


void ST_runPosConv(ST_Handle handle, ENC_Handle encHandle, CTRL_Handle ctrlHandle)
{
	ST_Obj *stObj = (ST_Obj *)handle;

	// get the electrical angle from the ENC module
#ifdef AngleResetViaIndxPin
    // get the electrical angle from the ENC module
    #ifdef AngleResetViaIndxPin_Dbug
        STPOSCONV_setElecAngle_erev(stObj->posConvHandle, ENC_getElecAngle(encHandle));
    #else
        STPOSCONV_setElecAngle_erev(stObj->posConvHandle, reCnstrctd_pos_erev);
    #endif
#else
    STPOSCONV_setElecAngle_erev(stObj->posConvHandle, ENC_getElecAngle(encHandle));
#endif
    if(USER_MOTOR_TYPE ==  MOTOR_Type_Induction) {
      // The CurrentVector feedback is only needed for ACIM
      // get the vector of the direct/quadrature current input vector values from CTRL
      STPOSCONV_setCurrentVector(stObj->posConvHandle, CTRL_getIdq_in_addr(ctrlHandle));
    }

	// run the SpinTAC Position Converter
	STPOSCONV_run(stObj->posConvHandle);

	if(USER_MOTOR_TYPE ==  MOTOR_Type_Induction) {
	  // The Slip Velocity is only needed for ACIM
	  // update the slip velocity in electrical angle per second, Q24
	  SLIP_setSlipVelocity(slipHandle, STPOSCONV_getSlipVelocity(stObj->posConvHandle));
	}
}


void ST_runVelCtl(ST_Handle handle, CTRL_Handle ctrlHandle)
{

    ST_Obj *stObj = (ST_Obj *)handle;
//    CTRL_Obj *ctrlObj = (CTRL_Obj *)ctrlHandle;

    // Get the mechanical speed in pu
    speedFeedback = STPOSCONV_getVelocityFiltered(stObj->posConvHandle);

	// Run the SpinTAC Controller
	// Note that the library internal ramp generator is used to set the speed reference
    // STVELCTL_setVelocityReference(stObj->velCtlHandle, TRAJ_getIntValue(ctrlObj->trajHandle_spd));
    // STVELCTL_setAccelerationReference(stObj->velCtlHandle, _IQ(0.0));	// Internal ramp generator does not provide Acceleration Reference
    // STVELCTL_setVelocityFeedback(stObj->velCtlHandle, speedFeedback);
    // STVELCTL_run(stObj->velCtlHandle);

	// select SpinTAC Velocity Controller
    // iqReference = STVELCTL_getTorqueReference(stObj->velCtlHandle);



#ifdef AngleResetViaIndxPin_Dbug
    CTRL_setIq_ref_pu(ctrlHandle, 0);
#else

    #ifdef posRev_1_currA_0
    // position error-based PD controller
    TargetPos_iq = _IQ((float)assembledCMD/SCALE_FACTOR);
        #ifdef AngleResetViaIndxPin
    PosError_iq =  TargetPos_iq - mtrPosMrev_iq;
        #else
    PosError_iq =  TargetPos_iq - st_obj.vel.conv.Pos_mrev;
        #endif
    _iq KpTerm = _IQmpy(PosError_iq,tuning_KP);
    _iq KdTerm = _IQmpy(speedFeedback,tuning_KD);
    if (_IQabs(KpTerm) > _IQabs(KdTerm)){                   // Kp should be bigger than Kd (to mitigate undesired directional run)
        if (tuning_KD < 0)
            {iqReference = KpTerm + KdTerm;}
        else {iqReference = 0;}
    }
    #else
    // Start receiving external position command upon checking
    if(initExperiment==1){
        iqReference = _IQ((float)assembledCMD/SCALE_FACTOR);

        #ifdef Anticogged
        debugVariable = (int16_t) ((double)mtrPos/maxPos*USER_MOTOR_ENCODER_LINES); // <!> % only takes two input parameters as integers
        I_compensation = (float)mapCurrent_A(debugVariable,assembledCMD)/EXTRA_SCALE_FACTOR;
        if((mtrPosInitiated)&&(initExperiment == 1)) {iqReference = iqReference + _IQmpy(_IQ(I_compensation),_IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A));}
        #endif
    }
    #endif

    //safety checking
    if(iqReference > Imax) {iqReference = Imax;}
    else if(iqReference < Imin) {iqReference = Imin;}
    //directional
    if(iqReference < _IQ(0.0)) { CTRL_setSpd_ref_krpm(ctrlHandle,_IQ(-0.01));}
    else if((iqReference > _IQ(0.0)) || (iqReference == _IQ(0.0))) { CTRL_setSpd_ref_krpm(ctrlHandle,_IQ(0.01));}

    CTRL_setIq_ref_pu(ctrlHandle, iqReference);

#endif
}

// added by Hubert Kim
interrupt void sciBRxISR(void)
{
    HAL_Obj *obj = (HAL_Obj *)halHandle;
    COMM_N_TIMER_Obj *commNtimer = (COMM_N_TIMER_Obj *) commNtimerHandle;

    dataGet = SCI_getDataNonBlocking(commNtimer->sciBHandle, &success);

    if (assy16BitDat)
    {
        if(HL_t)
        {
            MSB = dataGet;
            assembledCMD = (MSB <<8) | LSB; assy16BitDat = 0;
#ifdef Acceleration_based_antiCoggingTest
            test_shiftFactor = _IQ((float)assembledCMD/SCALE_FACTOR);
#endif
        }
        else {LSB = dataGet; HL_t = 1;}
    }
    else
    {
        switch (dataGet){
        case /*current input*/ 'c':
            assy16BitDat = 1;
            HL_t = 0;
            break;
        case /*initialize motor & timer*/ 125:
            // acknowledge & enable the Timer 0 interrupts
            HAL_enableTimer0Int(halHandle);
            timer0cnt = 0;
            gMotorVars.Flag_enableSys = 1;
#ifdef AngleResetViaIndxPin
    #ifdef AngleResetViaIndxPin_Dbug
    #else
                gMotorVars.Flag_enableRsRecalc = 0;
    #endif
#else
            gMotorVars.Flag_Run_Identify = 1;
#endif
            break;
        case /*pause & wait for re-initialize*/ 124:
            gMotorVars.Flag_enableSys = 0;
            gMotorVars.Flag_Run_Identify = 0;
            TIMER_disableInt(obj->timerHandle[0]); // = stop data logging
            break;
        case /* switch to position-input (PART 2)*/ 123:
            initExperiment = 1;
            break;
        }
    }
    PIE_clearInt(obj->pieHandle,PIE_GroupNumber_9);
}


// added by Hubert Kim
interrupt void timer0ISR(void)
{

    //acknowledge the Timer 0 interrupt
    HAL_acqTimer0Int(halHandle);
    COMM_N_TIMER_Obj *commNtimer = (COMM_N_TIMER_Obj *) commNtimerHandle;
//    ST_Obj *stObj = (ST_Obj *)stHandle;

    updateDLogPkg();

    ptr_ArrayPtrs4DLog = (uint16_t *)ArrayPtrs4DLog[cntr];

    if (HL == 1){ cntr++; data = ((*ptr_ArrayPtrs4DLog)>>8) & 0xFF; }
    else /*MSB first*/ { data = (*ptr_ArrayPtrs4DLog) & 0xFF; }
    while (!SCI_putDataNonBlocking(commNtimer->sciBHandle,data));

    HL ^= 1;
    cntr %= dLogSize;

    if(cntr == 0) flag_updatePkg = 1;

#ifdef AngleResetViaIndxPin
#ifdef AngleResetViaIndxPin_Dbug
    gMotorVars.Flag_Run_Identify = 1;
#else
    // check index-reset
    if(mtrPosInitiated) { gMotorVars.Flag_Run_Identify = 1; }
#endif
#endif

#ifdef AngleResetViaIndxPin
    if(!mtrPosInitiated)    // first time calibration done for getting the offset value
    {
        uint32_t IndxCntr = COMM_N_TIMER_getQepIndxCounts(halHandle);// actuator encoder counter
        uint32_t IndxOffset = 0;
        uint32_t absPos = 0;
        if(IndxCntr > 0 && !mtrPosInitiated)
        {
            /*adjusted debug variable: how much travel to get the index pin*/ IndxOffset = 15175;   // debug_IndxCnt_1
            absPos = HAL_getQepPosnCounts(halHandle) + (IndxOffset - IndxCntr);
            COMM_N_TIMER_setQepPosnCounts(halHandle,absPos);                                        // reset the qep readings
            /*adjusted debug variable: offset as booting up the encoder*/ ENC_setZeroOffset(encHandle, 391); // debug_zeroOffset, same as default RsRecalc
            mtrPosInitiated = 1;
        }
    }
#ifdef AngleResetViaIndxPin_Dbug
    CTRL_Obj *obj = (CTRL_Obj *)ctrlHandle;

    debug_IndxCnt_1 = COMM_N_TIMER_getQepIndxCounts(halHandle);
    if ((EST_getState(obj->estHandle) == EST_State_Rs) && (USER_MOTOR_TYPE == MOTOR_Type_Pm)) // below is the same as the default Rs start-up codes
    {
        debug_QepPosMax = HAL_getQepPosnMaximum(halHandle);
        debug_QepPosCnts = HAL_getQepPosnCounts(halHandle);
        debug_zeroOffset = debug_QepPosMax - debug_QepPosCnts;
//        ENC_setZeroOffset(encHandle, (uint32_t)(HAL_getQepPosnMaximum(halHandle) - HAL_getQepPosnCounts(halHandle)));
        ENC_setZeroOffset(encHandle, debug_zeroOffset);
    }
#endif
#endif

    return;
} // end of timer0ISR() function


#ifdef AngleResetViaIndxPin
void posOffsetConv(HAL_Handle halHandle)
{
  mtrPos = HAL_getQepPosnCounts(halHandle);
  maxPos = HAL_getQepPosnMaximum(halHandle);
  if (mtrPos > maxPos/2)  { mtrPosMrev_iq = _IQ(0-(double)(maxPos - mtrPos)/maxPos); }
  else {mtrPosMrev_iq = _IQ((double)mtrPos/maxPos); }

  // test the position reading conversion (mechanically reconstructed -> electrically reconstructed)
  temporary_pos_conv= _IQmpy(mtrPosMrev_iq,_IQ(USER_MOTOR_NUM_POLE_PAIRS)) % _IQ(1);
  if (temporary_pos_conv >= _IQ(0)) { reCnstrctd_pos_erev = temporary_pos_conv; }
  else {reCnstrctd_pos_erev = _IQ(1)+temporary_pos_conv; }

  return;
}
#endif

void updateDLogPkg()
{

    CTRL_Obj *obj = (CTRL_Obj *)ctrlHandle;
    ST_Obj *stObj = (ST_Obj *)stHandle;

    if ((cntr == 0)&&(flag_updatePkg == 1)) /* update the whole packet */ {
        timer0cnt++; // time stamp update @ 10ms, when timer0ISR @ 0.5ms
        PKG_ONE = (int16_t) ((float)mtrPos/maxPos*EXTRA_SCALE_FACTOR);
#ifdef AngleResetViaIndxPin
//        PKG_TWO = (int16_t)(_IQtoF(mtrPosMrev_iq)*EXTRA_SCALE_FACTOR);
        PKG_TWO = (int16_t) FILTER_FO_run(filterHandle[1],(_IQtoF(USER_computeTorque_Nm(obj, gTorque_Flux_Iq_pu_to_Nm_sf, gTorque_Ls_Id_Iq_pu_to_Nm_sf))*EXTRA_SCALE_FACTOR));
#else
//          PKG_TWO = ((int16_t) (_IQtoF(st_obj.pos.conv.Pos_mrev)*USER_MOTOR_ENCODER_LINES) % USER_MOTOR_ENCODER_LINES ); //saturated variable
#ifdef antiCoggingMapping
#else
        // converting the (0-10) to (0-1)
        debugVariable = ( (int32_t) (_IQtoF(st_obj.vel.conv.Pos_mrev)*USER_MOTOR_ENCODER_LINES) ) % (int32_t)USER_MOTOR_ENCODER_LINES ; // <!> % only takes two input parameters as integers
        if (debugVariable < 0) { PKG_TWO = (int16_t) ((USER_MOTOR_ENCODER_LINES + debugVariable)/USER_MOTOR_ENCODER_LINES*SCALE_FACTOR); } // see if this is different than above.
        else { PKG_TWO = (int16_t) (debugVariable/(float)USER_MOTOR_ENCODER_LINES*(float)SCALE_FACTOR); }
//      debugVariable = (int32_t) (_IQtoF(st_obj.pos.conv.Pos_mrev));
//      PKG_TWO = (int16_t) (_IQtoF(st_obj.pos.conv.Pos_mrev)*(float)SCALE_FACTOR);
#endif
#endif
#ifdef sensor_1stOrderFiltered
      PKG_THREE = (int16_t) FILTER_FO_run(filterHandle[1],(_IQtoF(_IQmpy(PID_getFbackValue(obj->pidHandle_Iq),_IQ(USER_IQ_FULL_SCALE_CURRENT_A)) )*EXTRA_SCALE_FACTOR));
#else
//      PKG_THREE = (int16_t)(_IQtoF(_IQmpy(iqReference,_IQ(USER_IQ_FULL_SCALE_CURRENT_A)))*(float)SCALE_FACTOR);
      PKG_THREE = (int16_t) FILTER_FO_run(filterHandle[1],(_IQtoF(_IQmpy(PID_getFbackValue(obj->pidHandle_Iq),_IQ(USER_IQ_FULL_SCALE_CURRENT_A)) )*EXTRA_SCALE_FACTOR));
#endif
//      PKG_FOUR = (int16_t)(_IQtoF(st_obj.vel.conv.Pos_erev)*(float)SCALE_FACTOR);
      PKG_FOUR = (int16_t) (_IQtoF(FILTER_FO_run(filterHandle[0],STPOSCONV_getVelocity(stObj->posConvHandle)))*EXTRA_SCALE_FACTOR);
      flag_updatePkg = 0;
    }
} // end of updateDLogPkg() function

#ifdef AngleResetViaIndxPin
#ifdef AngleResetViaIndxPin_Dbug
#else
void runRsOnLine(CTRL_Handle handle)
{
    CTRL_Obj *obj = (CTRL_Obj *)handle;

    if (gMotorVars.Flag_Run_Identify == true )
    {
        if(EST_getState(obj->estHandle) == EST_State_OnLine)
        {
            float_t RsError_Ohm = gMotorVars.RsOnLine_Ohm - gMotorVars.Rs_Ohm;

            EST_setFlag_enableRsOnLine(obj->estHandle, true);
            EST_setRsOnLineId_mag_pu(obj->estHandle, _IQmpy(_IQdiv64(gMotorVars.RsOnLineCurrent_A),_IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A))); // rs online current magnitude

            if(abs(RsError_Ohm) < (gMotorVars.Rs_Ohm * 0.05))
            {
                EST_setFlag_updateRs(obj->estHandle, true);
            }
        }
        else
        {
            EST_setRsOnLineId_mag_pu(obj->estHandle, _IQ(0.0));
            EST_setRsOnLineId_pu(obj->estHandle, _IQ(0.0));
            EST_setRsOnLine_pu(obj->estHandle, EST_getRs_pu(obj->estHandle));
            EST_setFlag_enableRsOnLine(obj->estHandle, false);
            EST_setFlag_updateRs(obj->estHandle, false);
            EST_setRsOnLine_qFmt(obj->estHandle, EST_getRs_qFmt(obj->estHandle));
        }
    }

    return;
} // end of runRsOnLine() function
#endif
#endif


//@} //defgroup
// end of file
