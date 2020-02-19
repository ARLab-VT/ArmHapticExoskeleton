/*
  communication state machine + Debugging the torque ripple
  PART I - session 1: neutral condition, session 2: pre-load, session 3: constant speed
  PART II - Device/Human Chirp test
  PART III - Trajectory tracking comparison between visual vs. JTF
  
  Pair with: proj_lab12b.c
  
  Update Log: 
  3/11/2019 ISM for ATT State Machine ver.1
  3/18/2019 ISM for ATT State Machine ver.2 (Iqref >0 -> extension)
  3/21/2019 ISM for ATT State Machine ver.3 (separate amptlitude with the direction)
  3/22/2019 Iq_ref direction problem persists: initially before the experiment, everytime motor runs, check the direction first.
  4/07/2019 Adding back the method 3, constant speed perturbation from "MovingJND_Animation.pde"
  4/15/2019 GUI for the indicating method 3 is changed to "velocity bar"(Keep the strip: not the position error we condition. The speed error)
  4/23/2019 Look-up table provided for the step difference: initial phase, the difference is big -> decreases as it goes toward the end of the experiment.
  5/15/2019 iq_ref direction issue -> this should be finalized now: (+) is flexion force. STVELCTL_setEnable(stObj->velCtlHandle, false); in the code
  6/19/2019 Probability of "Random direction selection" adjusted according to the remaining transition points.  
  6/24/2019 Adding features for PART 2: reading text trajectory via threading.
  6/26/2019 PART I becomes a single package.
  6/27/2019 PART II as a single package.[
  7/10/2019 Modification on the data log array for calibrating the CCS's torque feedback.
  8/23/2019 Add a beep sound to cue the event "change of direction".
  8/24/2019 Modofied to ease the task difficulty of method 3.
  8/25/2019 Add a demonstration/practice session with a skip button.
  8/29/2019 Added monitoring FSR. Replace the last packet from timerCount to four FSR indicators, 0000 ~ 1111
            The Constant Speed ATT state machine is modified to update its reference speed by itself.
  8/30/2019 Add two practice session before each of the experiment. Let it transition to the actual experiment when the requirements are met. Switch from 3-stage to 2-stage levels. 
            Reflected Knowledge of Result(KR) and Reward and punishment via scoring system.
  9/01/2019 Change the mehtod 3 response type from the absolute direction to the relative direction. f- 'flexion' to 'assistive', r- 'extension' to 'resistive'
  9/04/2019 Reverse the order for method 1~2: minigame -> weight 2 -> weight 1 -> neutral. (no coding change)
            Totally rennovated method 3:
            1) One direction two converging variables. 
            2) When speed changes, do the minigame for adjusting the time. (learn rhythm. So two minigames for method 3)
            3) Restoration motion followed by the test movement should allow user response.
  
  Error Log:
  3/22/2019 iq_ref direction alternating problem (with no additional change)-> withe the correction, (+) should be flexion
  
  -> Further details on the recent update(4/11/2019):  
   code for ATT integrated with the visual guidance.
   Torque is applied under the two conditions: 
   1) the visual condition provides the timing of when is the constant speed guiding region [Continuous time condition]
   2) the error for the position threshold indicates that the user is following well and thus it is ready to apply the torque [ Discrete time condition]
  
   states:
  
   CALCULATE - 
     conditions: beginning of the section 1~2(accelerating) or beginning of the 3~4(decelerating). 
    understand the user responses: if the prior accl/decel section raised the flag and no user response -> cannot detected
                     if the prior accl/decel section raised the flag and user responded -> detected
                     if the prior accl/decel section didn't raise the flag -> shouldn't be any response as well
                      => four differnt cmobinations for flx ext & aid hurt
                     => so take two points: see if the previous step actually applied the torque & if so, analyze the responses.
    monitor the position error during the sequence: if even once the arm is off the threshold, outOfThreshold_flag up
  
   
   APPLYandLISTEN - 
     conditions: section 2~3(constant speed region) 
    if the error is within the threshold during the section 1~2 or 3~4, (determined by the outOfThreshold_flag) apply randomly decided torque
    internally, remembers whether the flag was on or not. 
    applies the calculated torque only if accl/decel section raised the flag. Otherwise, simply pass it
   
   toggle switch: We want to switch the state machine only when there exists a transition in the visual sequence, not during the same sequence.
   ==> so, toggle changes once in the sequence transition, and during that change, the state machine is called. State machine is called only once 
   as soon as the visual sequcne changes 
  
   [ to be fixed after the pilot testing]
   Because this state machine is not running under the random timer, the torque input will last, if enable condition met, until the end of the sequence.
   And it is always not preferred to last the input that long. torque input should only last within the constant speed region.
   ==> For the 'applying torque condition', run a separate timer. Therefore, even if the condition is met, it only last the short duration of time. 1 sec for now 
   ==> It also needs a toggle such that only it is on whenever necessary
    
   -> Further details on the update(4/23/2019)
   [logTable] To adjust the running time for the experiment and the transition points;
   1) We fix the transition point for each test to: 5.
   2) Since we have small number to achieve during the each sequence of the test, we apply the initial value big, exponentially decreased as it goes.
   Therefore: the look-up table (e^(-0.5*x)) size is 1 by 5 and made of: 0.607 0.3679 0.2231 0.1353 0.0821. 
   The Step Difference updates every transition point = seriesNum
   
   Protocol:
   1. Debugging or data logging for the loadcell tuning:
   Execute CCS -> Execute Projcessing.js -> Move human interface to hit the index pin -> Wait till all the state machine in CCS updates to Online status -> hit enter in Processing & start recording / apply torque / hit spacebar to quit and save
   2. Experiment
   Execute CCS -> Execute Projcessing.js -> Move human interface to hit the index pin -> Wait till all the state machine in CCS updates to Online status -> follow the gui: choose the PART and etc.
   
   Using debugging mode:
   Run -> Press 1 -> Press Enter
   
*/

import processing.serial.*;
import ddf.minim.*;

Serial myPort;  
Minim playSound;
AudioSample up;
AudioSample down;
        
Table logTable = new Table();                                                       // table required for logging the data

// --------------------------------------------------------------------------------------------------------------------------
// the defines
boolean phLEAD = false;                                                             // turn on/off phase-lead model for running the trajectory output (only relevant to the position control)
boolean Acceleration_based_antiCoggingTest = false;
boolean logLoadCell = true;                                                        // for calibrating the CCS's torque feedback, or additional calibrations
static final int backgroundFrameRate = 100;                                         // Hz

// the constants
static final int I_MAX_mA = 14000;                                                  // 14 A, maximum current goes to the motor
static final int I_MIN_mA = -14000;
static final float KRPM_2_DEGPERSEC = 6000.0;                                       // 1 rpm = 6 deg/s
static final float KRPM_2_RADPERSEC = 104.72;                                       // 1 krpm = 1000 * 0.10472
static final float REVTODEG = 360.0;                                                // 1 rev = 360 deg
static final float RADTODEG = 57.2958;                                              // 1 rad = 57.2958 degree
static final float inputResolution = 100;
static final int SCALE_FACTOR = 1000;                                               // For the communication resolution
static final int EXTRA_SCALE_FACTOR = 1000000;                                      // for really tiny number reciept
static final int[] randomCounterNumbers = {200, 250, 225};                          // depends on the framerate, this varies as well
//static final float deviationTorqThreshold_mNm = 40;                               // [terminate condition #1] I want the converged value to be within this range
static final int TRANSITION_POINTS_LIMITS = 6;//6;                                  // [terminate condition #2] have the transition points more than 4 to finish the sequence.
static final char[] randomDirection = {'f','e'};
static final float torqCnst = 0.083053;                                             // [Nm/A] real-time torque constant, updates every loop (reciprocol is used to convert to current)
static final float torqCnst_range = 0.01;
static final float lowerStripe = 10;                                                // min RoM for Vconst[deg]
static final float upperStripe = 85-10;                                             // max RoM for Vconst
static final float upperLimit = 85;                                                 // max RoM
static final float lowerLimit = 10;                                                 // min RoM
static final float[] absolute_acceleration_deg_per_frameRateSquare = {0.008, 0.012,0.015};           // [deg/framerate^2] (3 different stage of ref. speed: 0.004; 0.008; 0.012)
static final float positionThreshold = 15; //8                                          // [deg], incomingPosition is raw data of pu from CCS
static final int intervalCounts_constant_speed = 30;                                // time duration applying torque. 30 * 10ms/frameRate = 0.3 sec (recall the main loop 100 Hz)
static final int offsetPosition = 5;                                                // to adust
static final int miniGame_minScore = 0;
static final int miniGame_maxScore = 100;
static final int miniGame1_reward = 10;
static final int miniGame2_reward = 5;
static final int miniGame_punishment = -5;

volatile float targetIref = 0;
float rndPsInpt = 0;
float rps = 0;                                                                      // button input from GUI, random pulse input
float cnstSPD = 0;
//**{Phase-lead model coeffs & variables}**
float cPAD_k = 0.2763;
float cPAD_k_1 = 0.01787;
float cPAD_k_2 = -0.1104;
float cPFD_k_1 = 1.137;
float cPFD_k_2 = -0.3211;
float PAD_k = 0;
float PAD_k_1 = 0;
float PAD_k_2 = 0;
float PFD_k = 0;
float PFD_k_1 = 0;
float PFD_k_2 = 0;
float targetPos4GUI_deg_modeled = 0;
//**{IIR LPF for JND}**
//float cST_OUT_K_1 =0.91572;// coefficients set for tau = 0.11365
//float cST_IN_K = 0.042141;
//float cST_IN_K_1 = 0.042141;
float cST_OUT_K_1 =0.95695;// coefficients set for tau = 0.2273 (0.5s of rise time)
float cST_IN_K = 0.021524;
float cST_IN_K_1 = 0.021524;
float ST_OUT_K =0; // variables
float ST_OUT_K_1 =0;
float ST_IN_K = 0;
float ST_IN_K_1 = 0;
//**{Serial Communication}**
boolean assy16Bit = false;
boolean HL = false;
receive_Stat incomingDAT_status;
volatile short LSB = 0;
volatile short MSB = 0;
volatile float incomingCurrent = 0.0;
volatile float incomingPosition = 0.0;
volatile float incomingPosition_deg = 0.0;
volatile float incomingTorque = 0.0;
volatile float incomingSpeed = 0.0;
//volatile float incomingSpeed_degS = 0.0;
volatile int incomingTimeStamp = 0;                               // was 'long' type before
volatile float incomingLogData = 0.0;
volatile short LSB_t = 0;
volatile short MSB_t = 0; 
volatile float[] storeThisRow = new float[6];                     // temporary array that stores all the recieved data

//**{Acceleration_based_antiCoggingTest}**
//static final float V_ref = 22;                                    //[V]
//static final float V_full = 44; 
//static final float V_resolution = 0.001;                          //[V]
//static final float V_huge_jump = 1;
//float V_target = 22;                                              // by default 22 V


//**{Stair Case Method}**
boolean run_ATT = false;
volatile float currentStimuli_mNm = 0;                             // current stimuli, torque
volatile float currentStimuli_mA = 0;                              // current stimuli, current
volatile int mainCounter = 0;
volatile byte response_ext = 0;                                    // initial 2, depends on the user's response 0 or 1
volatile byte response_flx = 0;                                    // initial 2, depends on the user's response 0 or 1 
volatile byte response = 0;                                        // initial 2, depends on the user's response 0 or 1
volatile byte responseExt_reCnstrctd_prev = 0;                                   // initial 2, depends on the user's response 0 or 1
volatile byte responseFlx_reCnstrctd_prev = 0;
volatile byte responseExt_reCnstrctd = 0;
volatile byte responseFlx_reCnstrctd = 0;
volatile int seriesNum_flx = 1;
volatile int seriesNum_ext = 1;
volatile int sequence_flx = 1; 
volatile int sequence_ext = 1;
volatile SEQUENCE_INDICATOR sequenceIndicator;

/*Init Val*/volatile float absTorqExt_mNm = 700;
/*Init Val*/volatile float absTorqFlx_mNm = 700;
/*Init Val*/volatile float absStepSizeExt_mNm = 100;
/*Init Val*/volatile float absStepSizeFlx_mNm = 100;
volatile char inputDirection = ' ';                                      // state variable, actual values assigned inside the 'randomDirection' char array above
volatile char inputDirection_prev = ' ';
volatile char inputDirection_extMotion = ' ';
volatile char inputDirection_extMotion_prev = ' ';
volatile char inputDirection_flxMotion = ' ';
volatile char inputDirection_flxMotion_prev = ' ';
volatile float absTorqExt_mNm_prev = 0;                          
volatile float absTorqFlx_mNm_prev = 0;
boolean isFlexionDone = false;
boolean isExtensionDone = false;
volatile int randomCounterNumber = 0;                              // counter storing the randomly picked delaying time
volatile byte userCue_toggle = 0;
volatile boolean userCue_transition = false;
volatile float tempTorCnst = 0;

//**{Constant speed perturbation }**
/*Init Val*/volatile float absTorqFlx_aid_mNm  = 700;
/*Init Val*/volatile float absTorqFlx_hurt_mNm = 700;
/*Init Val*/volatile float absTorqExt_aid_mNm  = 700;
/*Init Val*/volatile float absTorqExt_hurt_mNm = 700;
/*Init Val*/volatile float absStepSizeExt_AID_mNm = 100;
/*Init Val*/volatile float absStepSizeFlx_AID_mNm = 100;
/*Init Val*/volatile float absStepSizeExt_HURT_mNm = 100;
/*Init Val*/volatile float absStepSizeFlx_HURT_mNm = 100;

float targetPos4GUI_deg = offsetPosition;                                    // Target position input: 0 ~ 85 degree + offsetPosition
boolean run_cnstSPDTorq = false;
volatile float aVelocity_deg_per_frameRate = 0.0;
volatile float aAcceleration_deg_per_frameRateSquare = 0.0;
volatile float farmAngle = 0.0;                   // converted encoder reading, in [rad]
                                                    // default, false => section 2~3, activate the torque input; when the error is big, becomes true and bypass the torque input
volatile byte outOfThreshold_flag = 0;       // flag that allows to apply the torque: flag on during the section 1~2 or 2~3, and reset on APPLYandLISTEN 
volatile byte stateMachineToggle = 0;        // toggle to make sure to hit the state machine only once during the sequence
volatile byte applyTorq_toggle = 0;          // toggle on by APPLYandLISTEN state -> off by the end of intervalCounts_constant_speed
volatile int method_three_Counter = 0;              // counter for running the interval counter for the constant speed -> only starts when the conditons are met: 
                                                    // 1) constant speed region, 2) flag on during the section 1~2 or 2~3 tells ok
volatile byte responseFlx_reCnstrctd_aid = 0;
volatile byte responseFlx_reCnstrctd_hurt = 0;
volatile byte responseExt_reCnstrctd_aid = 0;
volatile byte responseExt_reCnstrctd_hurt = 0;
volatile byte responseFlx_reCnstrctd_aid_prev = 0;
volatile byte responseFlx_reCnstrctd_hurt_prev = 0;
volatile byte responseExt_reCnstrctd_aid_prev = 0;
volatile byte responseExt_reCnstrctd_hurt_prev = 0;
boolean isFlexionDone_aid = false;
boolean isFlexionDone_hurt = false;
boolean isExtensionDone_aid = false;
boolean isExtensionDone_hurt = false;
volatile float absTorqFlx_aid_mNm_prev = 0;
volatile float absTorqFlx_hurt_mNm_prev = 0;
volatile float absTorqExt_aid_mNm_prev = 0;
volatile float absTorqExt_hurt_mNm_prev = 0;
volatile int sequence_flx_aid = 1;
volatile int sequence_flx_hurt = 1;
volatile int sequence_ext_aid = 1;
volatile int sequence_ext_hurt = 1;
volatile int seriesNum_flx_aid = 1;
volatile int seriesNum_flx_hurt = 1;
volatile int seriesNum_ext_aid = 1;
volatile int seriesNum_ext_hurt = 1;
volatile float transitionTorqFlx_mNm_prev = 0;
volatile float transitionTorqFlx_mNm_prevPrev = 0;
volatile float transitionTorqExt_mNm_prev = 0;
volatile float transitionTorqExt_mNm_prevPrev = 0;
volatile float transitionTorqFlx_aid_mNm_prev = 0;
volatile float transitionTorqFlx_aid_mNm_prevPrev = 0;
volatile float transitionTorqFlx_hurt_mNm_prev = 0;
volatile float transitionTorqFlx_hurt_mNm_prevPrev = 0;
volatile float transitionTorqExt_aid_mNm_prev = 0;
volatile float transitionTorqExt_aid_mNm_prevPrev = 0;
volatile float transitionTorqExt_hurt_mNm_prev = 0;
volatile float transitionTorqExt_hurt_mNm_prevPrev = 0;
volatile float avgdTransitionTorqFlx_aid_mNm = 0;
volatile float avgdTransitionTorqFlx_hurt_mNm = 0;
volatile float avgdTransitionTorqExt_aid_mNm = 0;
volatile float avgdTransitionTorqExt_hurt_mNm = 0;
volatile float avgdTransitionTorqExt_mNm = 0;
volatile float avgdTransitionTorqFlx_mNm = 0;
volatile byte  motionATTSequenceIndicator = 0;      // 1~4; 1-flx aid, 2-flx hurt, 3-ext aid, 4-ext hurt. 

volatile int miniGame_currentScore = 0;

// lookuptable array
volatile float[] lookUpArray_stepDiff = new float[5];                     // temporary array that stores all the recieved data
volatile float stepSizeRatio = 0;                      // how fast step difference changes at the transition point: empirically chosen with 'num_trnstnPts'

// read path file threading 
boolean dataReadFinished = false;                 // toggles whether the desired trajectory is ready or not.
byte pathType = 0;
boolean readingTrajectoryData = false;
boolean enable_Recording = false;
float[] positionCMDLoaded;
boolean beginTest = false;
/* NEVER RESET */ byte testProcedure = 1;
/* NEVER RESET */ byte ATTcondition = 0;
/* NEVER RESET */ byte ATTMcondition = 0;
/* NEVER RESET */ boolean enablePART_I = false;
/* NEVER RESET */ boolean enablePART_II = false;

volatile int motionATTSequenceIndicator_prev = 0;
volatile boolean miniGameToggle = false;
volatile int miniGameCounter_ArmUpForceUp = 0;
volatile int miniGameCounter_ArmDownForceUp = 0;
volatile int miniGameCounter_ArmDownForceDown = 0;
volatile int miniGameCounter_ArmUpForceDown = 0;
// --------------------------------------------------------------------------------------------------------------------------

void setup()
  {    
    size(250, 250, P2D);
  
    // **{Serial comm set}**       
    printArray(Serial.list()); 
    String portName = Serial.list()[0];                             // can be checked by previous line of code
    myPort = new Serial(this, portName, 57600);                     // DO NOT USE PARITY. IT BREAKS the frame. 
    //myPort = new Serial(this, portName, 115200,'O',8,1.0);        // Odd parity, 8 bit data, stop bit one
    //myPort = new Serial(this, portName, 1125000,'O',8,1.0);       // Odd parity, 8 bit data, stop bit one
    myPort.clear(); 
    
    // **{Sound Object}**
    playSound = new Minim(this);
    down = playSound.loadSample("down_final.mp3", // filename
                                512);           // buffer size
    up = playSound.loadSample("up_final.mp3", // filename
                                512);
    if (( down == null )||( up == null )) println("Didn't find a sound source!"); // if a file doesn't exist, loadSample will return null
  
    
    // **{Lookup table set}**        
    lookUpArray_stepDiff[0] = 0.904837; //lookUpArray_stepDiff[0] = 0.6065;
    lookUpArray_stepDiff[1] = 0.740818; //lookUpArray_stepDiff[1] = 0.3678;
    lookUpArray_stepDiff[2] = 0.606531; //lookUpArray_stepDiff[2] = 0.2231;
    lookUpArray_stepDiff[3] = 0.496585; //lookUpArray_stepDiff[3] = 0.1353;
    lookUpArray_stepDiff[4] = 0.40657; //lookUpArray_stepDiff[4] = 0.0821;
            
    // **{Data logging set}**        
    logTable.addColumn("data 1");            
    logTable.addColumn("data 2");            
    logTable.addColumn("data 3");            
    logTable.addColumn("data 4");            
    logTable.addColumn("data 5");            
    logTable.addColumn("data 6");            
    logTable.addColumn("data 7");            
    logTable.addColumn("data 8");            
    logTable.addColumn("data 9");            
    logTable.addColumn("data 10");           
    logTable.addColumn("data 11");           
    logTable.addColumn("data 12");            
    logTable.addColumn("data 13");           
    logTable.addColumn("data 14");             
    logTable.addColumn("data 15");           
    logTable.addColumn("data 16");           
    logTable.addColumn("data 17");           
      
    frameRate(backgroundFrameRate);           // global fps:    

    sequenceIndicator = SEQUENCE_INDICATOR.APPLYandLISTEN;
    myPort.write(125); // tell CCS that the Processing ready so initialize the system.
    //updateVars();
    
} // end of setup()



void draw()
{
  
  // [Sound cue] motionATTSequenceIndicator: flx aid(1); flx hurt(2); ext aid(3); ext hurt(4);
  if ((motionATTSequenceIndicator_prev == 1) || (motionATTSequenceIndicator_prev == 4)) // flx -> ext:  (1 or 4) -> (2 or 3) 
  { if ((motionATTSequenceIndicator == 2) || (motionATTSequenceIndicator == 3)) {down.trigger();} }
  else if ((motionATTSequenceIndicator_prev == 2) || (motionATTSequenceIndicator_prev == 3))
  { if ((motionATTSequenceIndicator == 1) || (motionATTSequenceIndicator == 4)) {up.trigger();} }
  else if ((motionATTSequenceIndicator != 0)&&(motionATTSequenceIndicator_prev == 0)) {up.trigger();} // initially 
  
  if (frameCount % 10 == 0) {//! Only animation part [every 10 Hz]
    //println(frameRate);
     
    //---------------------------------------------------------------------<< GUI >>
    if (run_cnstSPDTorq){// turn on GUI indicator for method 3: 
      background(169,221,199);
      smooth();     
      fill(237,41,57);
      stroke(153); 
      
      //- indicates the range of motion + flx/ext sequence = provides when to initiate/terminate the motion to enhance the Knowledge of Result
      if (aVelocity_deg_per_frameRate>0) fill(255,205,42);
      else fill(23,219,36);
      arc(width/3,height*5/6,200,200,PI+HALF_PI+5/RADTODEG-offsetPosition/RADTODEG,TWO_PI-offsetPosition/RADTODEG);
      //- forearm, desired position, filled with varying color w.r.t. the tracking situation
      pushMatrix();
      //translate(122+targetPos4GUI_deg/360, 200+targetPos4GUI_deg/360); // coordinate + angle in deg
      translate(width/3,height*5/6);
      rotate(-radians(targetPos4GUI_deg));                     // takes the angle in (deg to rad)
      if(outOfThreshold_flag==1) {fill(255,42,44);}          // meaning, arm is tracking well (blue) 
      else { fill(38,59,232);}
      rect(0,0,100,20);                                        // we substitute the forward arm image with the rectangular figure
      popMatrix();
      //-forearm, current position, encoder, frame
      farmAngle = incomingPosition*2*PI;                // convert the recieved encoder reading [rev] to [rad]
      pushMatrix();
      translate(width/3,height*5/6);
      rotate(-farmAngle);
      noFill(); stroke(204, 102, 0);
      rect(0,0,100,20);  
      popMatrix();
      
      if (testProcedure == 4){                          // test procedure: KR + minigame
        textSize(15); fill(255, 255, 255);
        // **[Knowledge of Results]**
        // motionATTSequenceIndicator: flx aid(1); flx hurt(2); ext aid(3); ext hurt(4);         //<>//
        switch(motionATTSequenceIndicator) {
          case 1:
            text("ARM UP: FORCE UP",20,100);
            if (response  == 1) {
              if (miniGameToggle == true){ 
                  miniGame_currentScore = miniGame_currentScore + miniGame2_reward;
                  miniGameToggle = false;
                  miniGameCounter_ArmUpForceUp++;
                }
            } 
            break;
          case 2:
            text("ARM DOWN: FORCE UP",20,100);
            if (response  == 1) {
              if (miniGameToggle == true)
              {
                miniGame_currentScore = miniGame_currentScore + miniGame2_reward;
                miniGameToggle = false;
                miniGameCounter_ArmDownForceUp ++;
              }
            }   
            else   
            break;
          case 3:
            text("ARM DOWN: FORCE DOWN",20,100);
            if (response  == 1) {
              if (miniGameToggle == true)
              {
                miniGame_currentScore = miniGame_currentScore + miniGame2_reward;
                miniGameToggle = false;
                miniGameCounter_ArmDownForceDown ++;
              }
            } 
            break;
          case 4:
            text("ARM UP: FORCE DOWN",20,100);
            if (response  == 1) {
              if (miniGameToggle == true)
              {
                miniGame_currentScore = miniGame_currentScore + miniGame2_reward;
                miniGameToggle = false;
                miniGameCounter_ArmUpForceDown ++;
              }
            }       
            break;
        }// end of Knowledge of Result
        
        // **[mini game- reward and punishment]**
        if(miniGame_currentScore <0) {miniGame_currentScore = miniGame_minScore;}
        else if (miniGame_currentScore >= 100) {testProcedure = 5; resetParameters(); targetIref = 0; println("testProcedure 4-> 5"); println("========================");}
        textSize(25); fill(255, 255, 255); text(miniGame_currentScore+"/"+miniGame_maxScore,20,25);
        textSize(10);
        text("A-U F-U",30,60); text(miniGameCounter_ArmUpForceUp,30,70);  
        text("A-D F-U",75,60); text(miniGameCounter_ArmDownForceUp,75,70);  
        text("A-D F-D",120,60); text(miniGameCounter_ArmDownForceDown,120,70);  
        text("A-D F-D",165,60); text(miniGameCounter_ArmUpForceDown,165,70);
        
      } // end of practice text illustration
    } // end of GUI for method 3

    else if (run_ATT){ // for method 1 & 2
      // provides visual cue on changing background color only.
      if (sequenceIndicator == SEQUENCE_INDICATOR.APPLYandLISTEN) background(141, 236, 120);
      else background(255,42,44);
      smooth();
      
      if (testProcedure == 1){                          // test result should only be shown on the practice session
        textSize(15); fill(255, 255, 255); 
        // **[Knowledge of Results]**
        /*random time-delayed loop*/ if ((randomCounterNumber - mainCounter) < 10 ){ 
          if (currentStimuli_mNm > 0){ 
            if( response_flx == 1){  miniGame_currentScore = miniGame_currentScore + miniGame1_reward;}
            else                  {  miniGame_currentScore = miniGame_currentScore + miniGame_punishment;}} 
          else if (currentStimuli_mNm < 0){
            if( response_ext == 1){  miniGame_currentScore = miniGame_currentScore + miniGame1_reward;}
            else                  {  miniGame_currentScore = miniGame_currentScore + miniGame_punishment;}} 
        }
        
        if (currentStimuli_mNm > 0){ 
            if( response_flx == 1){text("Correct: flexion torque",20,100);}
            else                  {text("Wrong: flexion torque",20,100);  }
          } 
          else if (currentStimuli_mNm < 0){
            if( response_ext == 1){text("Correct: extension torque",20,100);}
            else                  {text("Wrong: extension torque",20,100);  }
          } 
          else /* initial run */ {text("No torque",20,100);}
        
        // **[mini game- reward and punishment]**
        if(miniGame_currentScore <0) {miniGame_currentScore = miniGame_minScore;}
        else if (miniGame_currentScore >= 100) {testProcedure = 2; resetParameters(); println("testProcedure 1-> 2");println("========================");}
        textSize(25); fill(255, 255, 255); text(miniGame_currentScore+"/"+miniGame_maxScore,50,125);

      } // end of practice text illustration
    } // end of GUI for method 1 &2
    else { // for IDLE screen
      background(169,221,199);
      smooth();
      textSize(10); fill(255, 255, 255); 

    // below is debugging purpose
    fill(23,219,36);
    text(incomingTorque,20,120);
    text("Torq[mNm]",20,100);

  } // end of IDLE screen
    
    //FSR indicators
    fill(0,0,0);
    rect(155,5,80,20); 
    /*FSR 1*/if (incomingTimeStamp >= 1000){fill(30, 132, 73);ellipse(165,15,15,15);incomingTimeStamp = incomingTimeStamp - 1000; }
    else  {fill(169,50,38);ellipse(165,15,15,15); }
    /*FSR 2*/if (incomingTimeStamp >= 100) {fill(30, 132, 73);ellipse(185,15,15,15);incomingTimeStamp = incomingTimeStamp - 100; }
    else  {fill(169,50,38);ellipse(185,15,15,15); }  
    /*FSR 3*/if (incomingTimeStamp >= 10)  {fill(30, 132, 73);ellipse(205,15,15,15);incomingTimeStamp = incomingTimeStamp - 10; }
    else  {fill(169,50,38);ellipse(205,15,15,15); } 
    /*FSR 4*/if (incomingTimeStamp >= 1)   {fill(30, 132, 73);ellipse(225,15,15,15);}
    else  {fill(169,50,38);ellipse(225,15,15,15); }
    
    {//---------------------------------------------------------------------<< text >>
      textSize(12); fill(75, 196, 213);
      // =PART I
      if (enablePART_I && !enablePART_II){
        text("PART I: ",60,40);
        if (beginTest) { // enable threading only if the enter being hit
          // GUI indicator
          text(" Running ",100,40);
          if (run_ATT) {text(" ATT",160,40); text(ATTcondition + 1, 190,40);} // because counting from 0
          else if (run_cnstSPDTorq) {text(" ATTM",160,40); text(ATTMcondition + 1, 190,40);}
          
          if ((testProcedure == 1)||(testProcedure == 2)||(testProcedure == 3)) {  run_ATT = true;  run_cnstSPDTorq = false;  }
          else if ((testProcedure == 4)||(testProcedure == 5)) {  run_ATT = false;   run_cnstSPDTorq = true; }
        } else {text(" IDLE",100,40);}
      }// end of PART I
      
      // =PART II
      else if (!enablePART_I && enablePART_II) {
        text("PART II: ",60,40);
        // send ASCII code to CCS for switching to position-error driven control
        myPort.write(123);  
        // depends on the trjectory chosen, call the trajectory array stored and load it.
        // (stay the loaded data until hitting enter)
        //  monitor the experimental state & run until subject hitting the space bar
      }// end of PART II
      
      // =IDLE or (DEBUGGING)
      else { 
        
        if (beginTest) { // enable threading only if the enter being hit
          // GUI indicator
          text("DEBUGGING",160,40);
          enable_Recording = true; // debugging purpose
          // check the testProcedure and assign the corresponding state tasks:
        }
        
        else {
          text("Choose Part of the Experiment",60,40);
          run_ATT = false; run_cnstSPDTorq = false; // IDLE state, turn off the PART I. 
        }
        
      }// end of IDLE
      
    }// == end of main state machine    
  }// end of GUI
  if (motionATTSequenceIndicator_prev != motionATTSequenceIndicator) {miniGameToggle = true;}
  motionATTSequenceIndicator_prev = motionATTSequenceIndicator;
  
  
// Since running the state machines in multi-threading, termination conditions should be outside of the threads
  if(run_cnstSPDTorq) /*program termination condition*/ if((isExtensionDone_aid && isFlexionDone_hurt) && (isExtensionDone_hurt && isFlexionDone_aid)) {stopRecordingAndSave();}

//--- thread lists --- [run on 100 Hz]
  thread("ATT"); 
  thread("cnstSPDTorq");
  
  mainCounter = mainCounter + 1;
} // end of draw()

// ----------------------------------------------<< Threads >>------------------------------------------------

//! def\     Static test (method #1) for the Absolute Torque Threshold (ATT) test
//! detail\  - determines the random interval period in between ipnut
//!          - torque input yielded from the user's response
//!          - toqrue constant updated in real-time based on the feedback readings
//!          - current input converted from the torque input via the updated torque constant
void ATT() { 
    if (run_ATT){

      // real-time running
      if(sequenceIndicator.getValue() == 1){ 
        //-- [User response's directional validity test]
        if (currentStimuli_mNm > 0)/* flexion directional input*/{response = response_flx;response_ext = 0;} 
        else if (currentStimuli_mNm < 0)/*extension directional input*/{response = response_ext;response_flx = 0;} 
        else /* initial run */ {response = 0;}
      }else {response_ext = 0;response_flx = 0;}
      
      /*random time-delayed loop*/ if ((randomCounterNumber - mainCounter) < 0 ) 
      {
        // reset timer counter
        mainCounter = 0; // (For the counter, avoid uisng frameCount: it calls the setup() at 0 and other pre-functions again.)
        
        {//-- [Selection for the random interval between the input]   
          int indxCounter = (int) random(randomCounterNumbers.length);
          randomCounterNumber = randomCounterNumbers[indxCounter];
        }    
    
        {//-- [State machine]: determines the direction of the input randomly and apply current output with whatever stored in the transition variables
         // output: currentStimuli_mNm
           switch (sequenceIndicator)
           {
             case CALCULATE: // zeroing state

               //- Take the 'filtered' response, based on the previous sequence, update/calculate the corresponding amplitude -----------------------------
               if( inputDirection_prev == 'f') // previous sequence was flexion, update the flexion amplitude with the filtered response =====================
               {
                 responseFlx_reCnstrctd = response;
                 
                 if(!isFlexionDone){
                   if(responseFlx_reCnstrctd == 1){        // case 'yes' 
                     if (responseFlx_reCnstrctd_prev == 0){// prev. cas 'no' (transition)
                       stepSizeRatio = lookUpArray_stepDiff[seriesNum_flx - 1];
                       absStepSizeFlx_mNm = absStepSizeFlx_mNm*stepSizeRatio;
                       sequence_flx = 1;
                       seriesNum_flx = seriesNum_flx + 1;
                       
                       // previous value(not current one we're applying now) caused transition so is reflected on the updated averaging 
                       //avgdTransitionTorqFlx_mNm = avgdTransitionTorqFlx_mNm + (absTorqFlx_mNm_prev - avgdTransitionTorqFlx_mNm)/seriesNum_flx;
                       //deviationTorqFlx_mNm = abs(avgdTransitionTorqFlx_mNm - absTorqFlx_mNm_prev);                 
                       // * save the torq magnitude right before this current transition & update the prevPrev
                       transitionTorqFlx_mNm_prevPrev = transitionTorqFlx_mNm_prev;
                       transitionTorqFlx_mNm_prev = absTorqFlx_mNm_prev;
                       
                       // determine if we collected enough data 
                       ///*termination: converging condition*/if ((deviationTorqFlx_mNm < deviationTorqThreshold_mNm)&&( absStepSizeFlx_mNm < absStepSizeThreshold_mNm)) {isFlexionDone = true;}                 
                       /*termination: transition pts #s*/if (seriesNum_flx >= TRANSITION_POINTS_LIMITS) {isFlexionDone = true;}                 
                     }// end of transition point
                     
                     sequence_flx = sequence_flx + 1;
                     if ( absTorqFlx_mNm > absStepSizeFlx_mNm) { absTorqFlx_mNm = absTorqFlx_mNm - absStepSizeFlx_mNm;}
                     else {absTorqFlx_mNm = 0;}
                   } // end of case 'yes'
                   else /*responseFlx_reCnstrctd == 0*/{    // case 'no'
                     if (responseFlx_reCnstrctd_prev == 1){ // prev. case 'yes' (transition)
                       stepSizeRatio = lookUpArray_stepDiff[seriesNum_flx - 1];
                       absStepSizeFlx_mNm = absStepSizeFlx_mNm*stepSizeRatio;
                       sequence_flx = 1;
                       seriesNum_flx = seriesNum_flx + 1;
                       
                       // previous value(not current one we're applying now) caused transition so is reflected on the updated averaging 
                       //avgdTransitionTorqFlx_mNm = avgdTransitionTorqFlx_mNm + (absTorqFlx_mNm_prev - avgdTransitionTorqFlx_mNm)/seriesNum_flx;
                       //deviationTorqFlx_mNm = abs(avgdTransitionTorqFlx_mNm - absTorqFlx_mNm_prev);                 
                       transitionTorqFlx_mNm_prevPrev = transitionTorqFlx_mNm_prev;                      
                       transitionTorqFlx_mNm_prev = absTorqFlx_mNm_prev;
                       
                       // determine if we collected enough data 
                       ///*termination: converging condition*/if ((deviationTorqFlx_mNm < deviationTorqThreshold_mNm)&&( absStepSizeFlx_mNm < absStepSizeThreshold_mNm)) {isFlexionDone = true;}                 
                       /*termination: transition pts #s*/if (seriesNum_flx >= TRANSITION_POINTS_LIMITS) {isFlexionDone = true;}                 
                     }// end of transition point 
                     
                     sequence_flx = sequence_flx + 1;
                     absTorqFlx_mNm = absTorqFlx_mNm + absStepSizeFlx_mNm;
                   } // end of response is no
                   
                   absTorqFlx_mNm_prev = absTorqFlx_mNm;
                 } else /* when collected enough data */ {}
                 
                 responseFlx_reCnstrctd_prev = responseFlx_reCnstrctd;
                 response_flx = 0;        
               }// end of flexion amplitude calculation 
               
               
               else if (inputDirection_prev == 'e') // previous sequence was extension, update the extension amplitude ==========================================
               {
                 responseExt_reCnstrctd = response;

                 if(!isExtensionDone){
                   if( responseExt_reCnstrctd == 1){      // YES
                     if(responseExt_reCnstrctd_prev == 0){ // transition case: NO -> YES
                       stepSizeRatio = lookUpArray_stepDiff[seriesNum_ext - 1];
                       absStepSizeExt_mNm = absStepSizeExt_mNm*stepSizeRatio;
                       sequence_ext = 1;
                       seriesNum_ext = seriesNum_ext + 1;
                       
                       // previous value(not current one we're applying now) caused transition so is reflected on the updated averaging 
                       //avgdTransitionTorqExt_mNm = avgdTransitionTorqExt_mNm + (absTorqExt_mNm_prev - avgdTransitionTorqExt_mNm)/seriesNum_ext;
                       //deviationTorqExt_mNm = abs(avgdTransitionTorqExt_mNm - absTorqExt_mNm_prev);
                       transitionTorqExt_mNm_prevPrev = transitionTorqExt_mNm_prev;                      
                       transitionTorqExt_mNm_prev = absTorqExt_mNm_prev;
      
                       // determine if we collected enough data 
                       ///*termination: converging condition*/if ((deviationTorqExt_mNm < deviationTorqThreshold_mNm)&&( absStepSizeExt_mNm < absStepSizeThreshold_mNm)) {isExtensionDone = true;}                 
                       /*termination: transition pts #s*/if (seriesNum_ext >= TRANSITION_POINTS_LIMITS) {isExtensionDone = true;}                 
                     } // end of transition case

                     if ( absTorqExt_mNm > absStepSizeExt_mNm) {absTorqExt_mNm = absTorqExt_mNm - absStepSizeExt_mNm;}
                     else {absTorqExt_mNm = 0;}       
                     sequence_ext = sequence_ext + 1;
                   } // end of case yes
                   else /*responseExt_reCnstrctd == 0*/{   // NO
                     if (responseExt_reCnstrctd_prev == 1) {// transition case: YES -> NO
                       stepSizeRatio = lookUpArray_stepDiff[seriesNum_ext - 1];
                       absStepSizeExt_mNm = absStepSizeExt_mNm*stepSizeRatio;
                       sequence_ext = 1;
                       seriesNum_ext = seriesNum_ext + 1;
                       
                       // previous value(not current one we're applying now) caused transition so is reflected on the updated averaging 
                       //avgdTransitionTorqExt_mNm = avgdTransitionTorqExt_mNm + (absTorqExt_mNm_prev - avgdTransitionTorqExt_mNm)/seriesNum_ext;
                       //deviationTorqExt_mNm = abs(avgdTransitionTorqExt_mNm - absTorqExt_mNm_prev);                 
                       transitionTorqExt_mNm_prevPrev = transitionTorqExt_mNm_prev;                      
                       transitionTorqExt_mNm_prev = absTorqExt_mNm_prev;
                       
                       // determine if we collected enough data 
                       ///*termination: converging condition*/if ((deviationTorqExt_mNm < deviationTorqThreshold_mNm)&&( absStepSizeExt_mNm < absStepSizeThreshold_mNm)) {isExtensionDone = true;}                 
                       /*termination: transition pts #s*/if (seriesNum_ext >= TRANSITION_POINTS_LIMITS) {isExtensionDone = true;}                 
                     }// end of transition point 
                     
                     sequence_ext = sequence_ext + 1; 
                     absTorqExt_mNm = absTorqExt_mNm + absStepSizeExt_mNm;                     
                   } // end of case no
                   
                   absTorqExt_mNm_prev = absTorqExt_mNm;
                 } else /* when collected enough data */ {}
                 
                 responseExt_reCnstrctd_prev = responseExt_reCnstrctd;
                 response_ext = 0;
               }// end of extension amplitude calculation 
               
               //- randomly decide which state to go next & apply the torque ----------------------------------------------------------------------------
               
               if (!isExtensionDone && !isFlexionDone){ int randomIndex = (int)round(random(0,100)/100);  inputDirection = randomDirection[randomIndex];}
               else
                  if(isExtensionDone)                 { int randomIndex = (int)round(random(0,70)/100);   inputDirection = randomDirection[randomIndex];}
                  else /*isFlexionDone*/              { int randomIndex = (int)round(random(30,100)/100); inputDirection = randomDirection[randomIndex];}

               inputDirection_prev = inputDirection;
               
               if (inputDirection == 'f'){ /* direction == flexion */ currentStimuli_mNm = absTorqFlx_mNm;}
               else { /* direction == extension */ currentStimuli_mNm = -absTorqExt_mNm;}

               sequenceIndicator = SEQUENCE_INDICATOR.APPLYandLISTEN;
             break;

             case APPLYandLISTEN: // filter the user response from(response_flx and response_ext) to 'response' (this is the only variable passed into the CALCULATE state             
               sequenceIndicator = SEQUENCE_INDICATOR.CALCULATE;
               currentStimuli_mNm = 0;
               response_flx = 0; response_ext = 0;
             break;
                          
           } // end of switch
        } // end of [state machine]
        
        /*termination condition*/ if (isExtensionDone && isFlexionDone) {stopRecordingAndSave();}          
                
        //Update the Torque Constant [mNm/mA]   
        tempTorCnst = torqCnst;
        //// -see if the data is nan
        //if ((incomingTorque != Float.NaN) && (incomingCurrent != Float.NaN)) {tempTorCnst = incomingTorque/(incomingCurrent);}
        //// -check the constant's boundary condition
        //if ((tempTorCnst > torqCnst + torqCnst_range) || (tempTorCnst < torqCnst - torqCnst_range)) {tempTorCnst = torqCnst; } // out of boundary, go back to the initial value (=average value)   
        
        //Convert the torque input to the current input                  
        if(tempTorCnst != Float.NaN) {currentStimuli_mA = currentStimuli_mNm/tempTorCnst;}
        else {currentStimuli_mA = currentStimuli_mNm/torqCnst;}
        
        // sanity check: current should never exceed +/- 11A (= 11000 mA)
        if (currentStimuli_mA > I_MAX_mA) {currentStimuli_mA = I_MAX_mA;}
        else if (currentStimuli_mA < I_MIN_mA) {currentStimuli_mA = I_MIN_mA;}    
        
      } // end of random delay counter condition
            
      // send the command to the motor
      if(!Acceleration_based_antiCoggingTest) {targetIref =   JND_LPF((int)currentStimuli_mA);}   // send the current in [mA] without SCALE_FACTOR
      //if(randomCounterNumber != 0) {targetIref = (currentStimuli_mA* ((float) mainCounter/randomCounterNumber));}
      else {targetIref = 0;}
      
    } // end of if(run_ATT)
} // end of ATT() function
// -----------------------------------------------------------------------------------------------------------------------------------------------------<<

void cnstSPDTorq(){
  if (run_cnstSPDTorq){
    
    // Update sequeniceIndicator 
    if(aVelocity_deg_per_frameRate >0) {
      if(inputDirection_flxMotion == 'f') {motionATTSequenceIndicator = 1;}
      else if(inputDirection_flxMotion == 'e') {motionATTSequenceIndicator = 4;}
    }
    else if (aVelocity_deg_per_frameRate <0){
      if(inputDirection_extMotion == 'f') {motionATTSequenceIndicator = 2;}
      else if(inputDirection_extMotion == 'e') {motionATTSequenceIndicator = 3;}
    }
    
    
    // disconnect the sequence and statemachine
    sequenceIndicator = SEQUENCE_INDICATOR.IDLE;
    
    // Take the user responses. Recall: flx aid(1), flx hurt(2), ext aid(3), ext hurt(4)  => flxion torque as aid / flexion torque as hurt / extension torque as aid / extension torque as hurt
    // {resistive or assistive reponse set}
    //if (response_flx == 1) { if ((motionATTSequenceIndicator == 1) || (motionATTSequenceIndicator == 3)) {response = 1;}  }
    //else if (response_ext == 1) { if ((motionATTSequenceIndicator == 2) || (motionATTSequenceIndicator == 4)) {response = 1;}  }
    //else {response = 0; response_flx= 0; response_ext=0;}
    // {away or toward response set}
    if (response_flx == 1) { if ((motionATTSequenceIndicator == 1) || (motionATTSequenceIndicator == 2)) {response = 1;}  }
    else if (response_ext == 1) { if ((motionATTSequenceIndicator == 3) || (motionATTSequenceIndicator == 4)) {response = 1;}  }
    else {response = 0; response_flx= 0; response_ext=0;}
    // {on and off} response set
    //if ((response_flx == 1)||(response_ext == 1)) {response = 1;response_flx=0;response_ext=0;}
    
    {//-- dynamics for visual guidance ---------------------------------------------
      targetPos4GUI_deg += aVelocity_deg_per_frameRate;                         // in [degree/framerate]
      aVelocity_deg_per_frameRate += aAcceleration_deg_per_frameRateSquare;     // in [degree/framerate^2]
      targetPos4GUI_deg = constrain(targetPos4GUI_deg,offsetPosition,upperLimit + offsetPosition);                   // might not necessary

      // sequence transitions
      if(targetPos4GUI_deg < lowerStripe + offsetPosition){                                                           // [section 1~2], accelerating region
        aAcceleration_deg_per_frameRateSquare = absolute_acceleration_deg_per_frameRateSquare[ATTMcondition];
        if (stateMachineToggle == 0) {sequenceIndicator = SEQUENCE_INDICATOR.CALCULATE;} // call the state once, update the amplitude from the responses
      } // end of [section 1~2]

      else if((targetPos4GUI_deg >= lowerStripe + offsetPosition) && (targetPos4GUI_deg < upperStripe + offsetPosition)){               // [section 2~3], constant speed region
        aAcceleration_deg_per_frameRateSquare = 0;
        if (stateMachineToggle == 1) {sequenceIndicator = SEQUENCE_INDICATOR.APPLYandLISTEN;} // call the state once, this allows torque if applicable       

        // constantly monitoring if the subject is following the visual cue during this whole sequence
        //if the position error is beyond the threshold even once. by default flag is off.
        if(abs(targetPos4GUI_deg/REVTODEG - incomingPosition) > positionThreshold/REVTODEG ){outOfThreshold_flag = 1;} //everything in rev.(PU)
      } // end of [section 2~3]

      else if((targetPos4GUI_deg >= upperStripe + offsetPosition) && (targetPos4GUI_deg < upperLimit + offsetPosition)){               // [section 3~4], deceleration region
        aAcceleration_deg_per_frameRateSquare = -absolute_acceleration_deg_per_frameRateSquare[ATTMcondition];
        if (stateMachineToggle == 0) {sequenceIndicator = SEQUENCE_INDICATOR.CALCULATE;}
      } // end of [section 3~4]
    }// end of dynamics for visual guidance 


    {//-- state machine ---------------------------------------------------------------
      switch(sequenceIndicator)
      {
        case CALCULATE:
          //- make sure to toggle up so this doesn't get calculated more than once in the same visual sequence.
          stateMachineToggle = 1;

          //- understands the user responses on the proper conditions and update accordingly.
          //if (outOfThreshold_flag == 0) {// updates only if the previously torque was applied. without error hits the boundary, default flag is false 

            // first let's see if this is the flexion or extension (mechanical speed > 0 is flexion, also guiding cue's speed >0 is flexion)
            if (aVelocity_deg_per_frameRate > 0) {     // [flexion motion]

              if (inputDirection_flxMotion_prev == 'f') {     // previous input was flexion torque
                responseFlx_reCnstrctd_aid = response;  // reconstruct the response
                if(!isFlexionDone_aid){// termination condtion for the flexion threshold, aid 
/* case 'yes' */  if(responseFlx_reCnstrctd_aid == 1) {        
                    if(responseFlx_reCnstrctd_aid_prev == 0){    // prev. case 'no' (transition)
                      stepSizeRatio = lookUpArray_stepDiff[seriesNum_flx_aid - 1];
                      absStepSizeFlx_AID_mNm = absStepSizeFlx_AID_mNm*stepSizeRatio;
                      sequence_flx_aid = 1;
                      seriesNum_flx_aid = seriesNum_flx_aid + 1;

                      // update: converging average
                      //avgdTransitionTorqFlx_aid_mNm = avgdTransitionTorqFlx_aid_mNm + (absTorqFlx_aid_mNm_prev - avgdTransitionTorqFlx_aid_mNm)/seriesNum_flx_aid;
                      // update: deviation
                      //deviationTorqFlx_aid_mNm = abs(avgdTransitionTorqFlx_aid_mNm - absTorqFlx_aid_mNm_prev);
                      transitionTorqFlx_aid_mNm_prevPrev = transitionTorqFlx_aid_mNm_prev;
                      transitionTorqFlx_aid_mNm_prev = absTorqFlx_aid_mNm_prev;
                       
                       
                      /*termination condition:*/ if(seriesNum_flx_aid >= TRANSITION_POINTS_LIMITS) {isFlexionDone_aid = true;} 
                    }// end of transition point
                    sequence_flx_aid = sequence_flx_aid + 1;
                    if (absTorqFlx_aid_mNm > absStepSizeFlx_AID_mNm) {absTorqFlx_aid_mNm = absTorqFlx_aid_mNm - absStepSizeFlx_AID_mNm;}
                    else {absTorqFlx_aid_mNm = absStepSizeFlx_AID_mNm;}

                  }// end of case 'yes'
/* case 'no' */   else /*(responseFlx_reCnstrctd_aid == 0)*/{      
                    if((responseFlx_reCnstrctd_aid_prev == 1)){    // prev. case 'yes' (transition)
                      stepSizeRatio = lookUpArray_stepDiff[seriesNum_flx_aid - 1];
                      absStepSizeFlx_AID_mNm = absStepSizeFlx_AID_mNm*stepSizeRatio;
                      sequence_flx_aid = 1;
                      seriesNum_flx_aid = seriesNum_flx_aid + 1;

                      // update: converging average
                      //avgdTransitionTorqFlx_aid_mNm = avgdTransitionTorqFlx_aid_mNm + (absTorqFlx_aid_mNm_prev - avgdTransitionTorqFlx_aid_mNm)/seriesNum_flx_aid;
                      // update: deviation
                      //deviationTorqFlx_aid_mNm = abs(avgdTransitionTorqFlx_aid_mNm - absTorqFlx_aid_mNm_prev);
                       transitionTorqFlx_aid_mNm_prevPrev = transitionTorqFlx_aid_mNm_prev;
                       transitionTorqFlx_aid_mNm_prev = absTorqFlx_aid_mNm_prev;
                       
                      /*termination condition:*/ if(seriesNum_flx_aid >= TRANSITION_POINTS_LIMITS) {isFlexionDone_aid = true;}                       
                    }// end of transition point
                    sequence_flx_aid = sequence_flx_aid + 1;
                    absTorqFlx_aid_mNm = absTorqFlx_aid_mNm + absStepSizeFlx_AID_mNm;

                  }// end of case 'no'
                  

                  absTorqFlx_aid_mNm_prev = absTorqFlx_aid_mNm;
                } // else, collected enough data  

                responseFlx_reCnstrctd_aid_prev = responseFlx_reCnstrctd_aid;
                response_flx = 0;
              } // end of flexion torque amplitude calculation

              else if (inputDirection_flxMotion_prev == 'e') {  // previous input was extension torque
                responseExt_reCnstrctd_hurt = response;  // reconstruct the response
                if(!isExtensionDone_hurt){// termination condtion for the extension threshold, hurt 
/* case 'yes' */  if (responseExt_reCnstrctd_hurt == 1){        
                    if (responseExt_reCnstrctd_hurt_prev == 0){    // prev. case 'no' (transition)
                      stepSizeRatio = lookUpArray_stepDiff[seriesNum_ext_hurt - 1];
                      absStepSizeExt_HURT_mNm = absStepSizeExt_HURT_mNm*stepSizeRatio;
                      sequence_ext_hurt = 1;
                      seriesNum_ext_hurt = seriesNum_ext_hurt + 1;

                      // update: converging average
                      //avgdTransitionTorqExt_hurt_mNm = avgdTransitionTorqExt_hurt_mNm + (absTorqExt_hurt_mNm_prev - avgdTransitionTorqExt_hurt_mNm)/seriesNum_ext_hurt;
                      // update: deviation
                      //deviationTorqExt_hurt_mNm = abs(avgdTransitionTorqExt_hurt_mNm - absTorqExt_hurt_mNm_prev);
                       transitionTorqExt_hurt_mNm_prevPrev = transitionTorqExt_hurt_mNm_prev;
                       transitionTorqExt_hurt_mNm_prev = absTorqExt_hurt_mNm_prev;

                      /*termination condition:*/ if(seriesNum_ext_hurt >= TRANSITION_POINTS_LIMITS) {isExtensionDone_hurt = true;}                       
                    }// end of transition point
                    sequence_ext_hurt = sequence_ext_hurt + 1;
                    if (absTorqExt_hurt_mNm > absStepSizeExt_HURT_mNm) {absTorqExt_hurt_mNm = absTorqExt_hurt_mNm - absStepSizeExt_HURT_mNm;}
                    else {absTorqExt_hurt_mNm = absStepSizeExt_HURT_mNm;}

                  }// end of case 'yes'
/* case 'no' */   else /*(responseExt_reCnstrctd_hurt == 0)*/{    // case 'no'
                    if (responseExt_reCnstrctd_hurt_prev == 1){    // prev. case 'yes' (transition)
                      stepSizeRatio = lookUpArray_stepDiff[seriesNum_ext_hurt - 1];
                      absStepSizeExt_HURT_mNm = absStepSizeExt_HURT_mNm*stepSizeRatio;
                      sequence_ext_hurt = 1;
                      seriesNum_ext_hurt = seriesNum_ext_hurt + 1;
                       
                      // update: converging average
                      //avgdTransitionTorqExt_hurt_mNm = avgdTransitionTorqExt_hurt_mNm + (absTorqExt_hurt_mNm_prev - avgdTransitionTorqExt_hurt_mNm)/seriesNum_ext_hurt;
                      // update: deviation
                      //deviationTorqExt_hurt_mNm = abs(avgdTransitionTorqExt_hurt_mNm - absTorqExt_hurt_mNm_prev);
                      transitionTorqExt_hurt_mNm_prevPrev = transitionTorqExt_hurt_mNm_prev;
                      transitionTorqExt_hurt_mNm_prev = absTorqExt_hurt_mNm_prev;
                       
                       
                      /*termination condition:*/ if(seriesNum_ext_hurt >= TRANSITION_POINTS_LIMITS) {isExtensionDone_hurt = true;}                       
                    }// end of transition point
                    sequence_ext_hurt = sequence_ext_hurt + 1;
                    absTorqExt_hurt_mNm = absTorqExt_hurt_mNm + absStepSizeExt_HURT_mNm;

                  }// end of case 'no'
                  absTorqExt_hurt_mNm_prev = absTorqExt_hurt_mNm;
                } // else, collected enough data

                responseExt_reCnstrctd_hurt_prev = responseExt_reCnstrctd_hurt;
                response_ext = 0;
              } // end of extension torque amplitude calculation

            } // end of flexion motion


            else if (aVelocity_deg_per_frameRate < 0) {  // [extension motion]
              if (inputDirection_extMotion_prev == 'f') {     // previous input was flexion torque
                responseFlx_reCnstrctd_hurt = response;  // reconstruct the response
                if(!isFlexionDone_hurt){// termination condtion for the flexion threshold, hurt 
/* case 'yes' */  if (responseFlx_reCnstrctd_hurt == 1){        // case 'yes'
                    if (responseFlx_reCnstrctd_hurt_prev == 0){    // prev. case 'no' (transition)
                      stepSizeRatio = lookUpArray_stepDiff[seriesNum_flx_hurt - 1];
                      absStepSizeFlx_HURT_mNm = absStepSizeFlx_HURT_mNm*stepSizeRatio;
                      sequence_flx_hurt = 1;
                      seriesNum_flx_hurt = seriesNum_flx_hurt + 1;

                      // update: converging average
                      //avgdTransitionTorqFlx_hurt_mNm = avgdTransitionTorqFlx_hurt_mNm + (absTorqFlx_hurt_mNm_prev - avgdTransitionTorqFlx_hurt_mNm)/seriesNum_flx_hurt;
                      // update: deviation
                      //deviationTorqFlx_hurt_mNm = abs(avgdTransitionTorqFlx_hurt_mNm - absTorqFlx_hurt_mNm_prev);
                      transitionTorqFlx_hurt_mNm_prevPrev = transitionTorqFlx_hurt_mNm_prev;
                      transitionTorqFlx_hurt_mNm_prev = absTorqFlx_hurt_mNm_prev;

                      /*termination condition:*/ if(seriesNum_flx_hurt >= TRANSITION_POINTS_LIMITS) {isFlexionDone_hurt = true;}                       
                    }// end of transition point
                    sequence_flx_hurt = sequence_flx_hurt + 1;
                    if (absTorqFlx_hurt_mNm > absStepSizeFlx_HURT_mNm) {absTorqFlx_hurt_mNm = absTorqFlx_hurt_mNm - absStepSizeFlx_HURT_mNm;}
                    else {absTorqFlx_hurt_mNm = absStepSizeFlx_HURT_mNm;}                    

                  }// end of case 'yes'
/* case 'no' */   else /*(responseFlx_reCnstrctd_hurt == 0)*/{    // case 'no'
                    if (responseFlx_reCnstrctd_hurt_prev == 1){    // prev. case 'yes' (transition)
                      stepSizeRatio = lookUpArray_stepDiff[seriesNum_flx_hurt - 1];
                      absStepSizeFlx_HURT_mNm = absStepSizeFlx_HURT_mNm*stepSizeRatio;
                      sequence_flx_hurt = 1;
                      seriesNum_flx_hurt = seriesNum_flx_hurt + 1;
                       
                      // update: converging average
                      //avgdTransitionTorqFlx_hurt_mNm = avgdTransitionTorqFlx_hurt_mNm + (absTorqFlx_hurt_mNm_prev - avgdTransitionTorqFlx_hurt_mNm)/seriesNum_flx_hurt;
                      // update: deviation
                      //deviationTorqFlx_hurt_mNm = abs(avgdTransitionTorqFlx_hurt_mNm - absTorqFlx_hurt_mNm_prev);
                      transitionTorqFlx_hurt_mNm_prevPrev = transitionTorqFlx_hurt_mNm_prev;
                      transitionTorqFlx_hurt_mNm_prev = absTorqFlx_hurt_mNm_prev;
                      
                      /*termination condition:*/ if(seriesNum_flx_hurt >= TRANSITION_POINTS_LIMITS) {isFlexionDone_hurt = true;}                         
                    }// end of transition point
                    sequence_flx_hurt = sequence_flx_hurt + 1;
                    absTorqFlx_hurt_mNm = absTorqFlx_hurt_mNm + absStepSizeFlx_HURT_mNm;

                  }// end of case 'no'
                  absTorqFlx_hurt_mNm_prev = absTorqFlx_hurt_mNm;
                } // else, collected enough data

                responseFlx_reCnstrctd_hurt_prev = responseFlx_reCnstrctd_hurt;
                response_flx = 0;
              } // end of flexion torque amplitude calculation

              else if (inputDirection_extMotion_prev == 'e') {  // previous input was extension torque
                responseExt_reCnstrctd_aid = response;  // reconstruct the response
                if(!isExtensionDone_aid){// termination condtion for the extension threshold, aid 
/* case 'yes' */  if (responseExt_reCnstrctd_aid == 1){        // case 'yes'
                    if (responseExt_reCnstrctd_aid_prev == 0){    // prev. case 'no' (transition)
                      stepSizeRatio = lookUpArray_stepDiff[seriesNum_ext_aid - 1];
                      absStepSizeExt_AID_mNm = absStepSizeExt_AID_mNm*stepSizeRatio;
                      sequence_ext_aid = 1;
                      seriesNum_ext_aid = seriesNum_ext_aid + 1;

                      // update: converging average
                      //avgdTransitionTorqExt_aid_mNm = avgdTransitionTorqExt_aid_mNm + (absTorqExt_aid_mNm_prev - avgdTransitionTorqExt_aid_mNm)/seriesNum_ext_aid;
                      // update: deviation
                      //deviationTorqExt_aid_mNm = abs(avgdTransitionTorqExt_aid_mNm - absTorqExt_aid_mNm_prev);
                      transitionTorqExt_aid_mNm_prevPrev = transitionTorqExt_aid_mNm_prev;
                      transitionTorqExt_aid_mNm_prev = absTorqExt_aid_mNm_prev;
                       
                      /*termination condition:*/ if(seriesNum_ext_aid >= TRANSITION_POINTS_LIMITS) {isExtensionDone_aid = true;}                             
                    }// end of transition point
                    sequence_ext_aid = sequence_ext_aid + 1;
                    if (absTorqExt_aid_mNm > absStepSizeExt_AID_mNm) {absTorqExt_aid_mNm = absTorqExt_aid_mNm - absStepSizeExt_AID_mNm;}
                    else {absTorqExt_aid_mNm = absStepSizeExt_AID_mNm;}  

                  }// end of case 'yes'
/* case 'no' */   else /*(responseExt_reCnstrctd_aid == 0)*/{      // case 'no'
                    if ((responseExt_reCnstrctd_aid_prev == 1)){  // prev. case 'yes' (transition)
                      stepSizeRatio = lookUpArray_stepDiff[seriesNum_ext_aid - 1];
                      absStepSizeExt_AID_mNm = absStepSizeExt_AID_mNm*stepSizeRatio;
                      sequence_ext_aid = 1;
                      seriesNum_ext_aid = seriesNum_ext_aid + 1;


                      // update: converging average
                      //avgdTransitionTorqExt_aid_mNm = avgdTransitionTorqExt_aid_mNm + (absTorqExt_aid_mNm_prev - avgdTransitionTorqExt_aid_mNm)/seriesNum_ext_aid;
                      // update: deviation
                      //deviationTorqExt_aid_mNm = abs(avgdTransitionTorqExt_aid_mNm - absTorqExt_aid_mNm_prev);
                      transitionTorqExt_aid_mNm_prevPrev = transitionTorqExt_aid_mNm_prev;
                      transitionTorqExt_aid_mNm_prev = absTorqExt_aid_mNm_prev;

                      /*termination condition:*/ if(seriesNum_ext_aid >= TRANSITION_POINTS_LIMITS) {isExtensionDone_aid = true;}                       
                    }// end of transition point
                    sequence_ext_aid = sequence_ext_aid + 1;
                    absTorqExt_aid_mNm = absTorqExt_aid_mNm + absStepSizeExt_AID_mNm;

                  }// end of case 'no'
                  absTorqExt_aid_mNm_prev = absTorqExt_aid_mNm;
                } // else, collected enough data

                responseExt_reCnstrctd_aid_prev = responseExt_reCnstrctd_aid;
                response_ext = 0;
              } // end of extension torque amplitude calculation
            } // end of extension motion
            
          currentStimuli_mNm = 0;
          // calculate the average for: the Motion ATT:
          avgdTransitionTorqExt_aid_mNm  = 0.5 * (transitionTorqExt_aid_mNm_prev + transitionTorqExt_aid_mNm_prevPrev);
          avgdTransitionTorqFlx_aid_mNm  = 0.5 * (transitionTorqFlx_aid_mNm_prev + transitionTorqFlx_aid_mNm_prevPrev);
          avgdTransitionTorqExt_hurt_mNm = 0.5 * (transitionTorqExt_hurt_mNm_prev + transitionTorqExt_hurt_mNm_prevPrev);
          avgdTransitionTorqFlx_hurt_mNm = 0.5 * (transitionTorqFlx_hurt_mNm_prev + transitionTorqFlx_hurt_mNm_prevPrev); 
        //} // end of (if the tracking performance is good) -out of threshold requirements


        // as we reconstructed the response, reset
        response = 0;
        // reset the flag to the default setting.
        outOfThreshold_flag = 0;
        break;
// -------------------------------------------------------------------------------

        case APPLYandLISTEN:
          
          {
            // Randomly decide direction and apply torque, regardless of the tracking performance 
              if (aVelocity_deg_per_frameRate > 0){     // [flexion motion]
                if (!isFlexionDone_hurt && !isFlexionDone_aid) { int randomIndex = (int)round(random(0,100)/100);  inputDirection_flxMotion = randomDirection[randomIndex];}
                else 
                  if (isFlexionDone_hurt && !isFlexionDone_aid)          // remaining is flexion, aid (flexion torque)
                    {int randomIndex = (int)round(random(0,70)/100);   inputDirection_flxMotion = randomDirection[randomIndex];}
                  else if(isFlexionDone_aid && !isFlexionDone_hurt)      // remaining is flexion, hurt (extension torque)
                    {int randomIndex = (int)round(random(30,100)/100); inputDirection_flxMotion = randomDirection[randomIndex];}
                  else                                                   // nothing remains in this direction of motion
                    {int randomIndex = (int)round(random(0,100)/100);  inputDirection_flxMotion = randomDirection[randomIndex];}

                  inputDirection_flxMotion_prev = inputDirection_flxMotion;
                  
                  // motionATTSequenceIndicator: flx aid(1); flx hurt(2); ext aid(3); ext hurt(4);
                  if (inputDirection_flxMotion == 'f')  { currentStimuli_mNm = absTorqFlx_aid_mNm;   } // (1)flexion torque functions as aid for flexion motion    (
                  else/*inputDirection_flxMotion == 'e'*/ { currentStimuli_mNm = -absTorqExt_hurt_mNm; } // (4)extension torque functions as hurt for flexion motion
              }
              else if (aVelocity_deg_per_frameRate < 0){// [extension motion]
                if (!isExtensionDone_aid && !isExtensionDone_hurt) { int randomIndex = (int)round(random(0,100)/100);inputDirection_extMotion = randomDirection[randomIndex];}
                else
                  if (isExtensionDone_aid && !isExtensionDone_hurt)       // remaining is extension, hurt (flexion torque)
                    {int randomIndex = (int)round(random(0,70)/100);    inputDirection_extMotion = randomDirection[randomIndex];}
                  else if (isExtensionDone_hurt && !isExtensionDone_aid)  // remaining is extension, aid (extension torque)
                    {int randomIndex = (int)round(random(30,100)/100);  inputDirection_extMotion = randomDirection[randomIndex];}
                  else                                                    // nothing remains in this direction of motion
                    {int randomIndex = (int)round(random(0,100)/100);   inputDirection_extMotion = randomDirection[randomIndex];}

                  inputDirection_extMotion_prev = inputDirection_extMotion;

                  // motionATTSequenceIndicator: flx aid(1); flx hurt(2); ext aid(3); ext hurt(4);
                  if (inputDirection_extMotion == 'f')  { currentStimuli_mNm = absTorqFlx_hurt_mNm; } // (2)flexion torque functions as hurt for extension motion   
                  else/*inputDirection_extMotion == 'e'*/ { currentStimuli_mNm = -absTorqExt_aid_mNm;  } // (3)extension torque functions as aid for extension motion
              }
        }/*end of random choice for the torque direction*/

        // **update and reset variables**
        // make sure to toggle up so this doesn't get calculated more than once in the same visual sequence.
        stateMachineToggle = 0;
          
        break;
        
        case IDLE: 
          // Default state, do nothing. - only called at the very initial stage
        break;
      }// end of switch
    }// end of [state machine]                          

    {// deliever the torque input
      //update the torque constant [mNm/mA]
      tempTorCnst = torqCnst;
      // convert the torque input to the current input
      if(tempTorCnst != Float.NaN) {currentStimuli_mA = currentStimuli_mNm / tempTorCnst;}
      else {currentStimuli_mA = currentStimuli_mNm / torqCnst;}  
      // sanity check: current should never exceed +/- 11 A (= 11000 mA)
      if (currentStimuli_mA > I_MAX_mA){currentStimuli_mA = I_MAX_mA;}
      else if (currentStimuli_mA < I_MIN_mA){currentStimuli_mA = I_MIN_mA;}
      // send the command to the motor in the integer form
      targetIref = JND_LPF((int)currentStimuli_mA);
    }
        
  }// end of run_cnstSPDTorq
}// end of cnstSPDTorq() function
// -----------------------------------------------------------------------------------------------------------------------------------------------------<<

void storeData() {
  TableRow newRow = logTable.addRow(); // 'c' is the very fist byte coming in. 
  
  if(run_cnstSPDTorq){ // motion ATT yields 4 converging thresholds. It requires different variabels to log
    newRow.setFloat("data 1",  currentStimuli_mA);  // calculated current
    newRow.setFloat("data 2",  currentStimuli_mNm); // converted torque
    newRow.setFloat("data 3",  absTorqFlx_aid_mNm);    // for each side, absolute calculated torque 
    newRow.setFloat("data 4",  absTorqExt_hurt_mNm);
    newRow.setFloat("data 5",  absTorqFlx_hurt_mNm);    // for each side, absolute calculated torque 
    newRow.setFloat("data 6",  absTorqExt_aid_mNm);
    //newRow.setInt  ("data 8",stateMachineToggle);
    newRow.setFloat("data 7",  aVelocity_deg_per_frameRate);
    newRow.setInt  ("data 8",  response); 
    newRow.setFloat("data 9",  avgdTransitionTorqFlx_aid_mNm); // converging value
    newRow.setFloat("data 10", avgdTransitionTorqExt_hurt_mNm); // converging value 
    newRow.setFloat("data 11", avgdTransitionTorqFlx_hurt_mNm); // converging value
    newRow.setFloat("data 12", avgdTransitionTorqExt_aid_mNm); // converging value 
    //newRow.setInt  ("data 10", applyTorq_toggle);
    //newRow.setInt  ("data 11", outOfThreshold_flag); 
    newRow.setFloat("data 13", storeThisRow[receive_Stat.TORQUE.getValue()]);         
    newRow.setFloat("data 14", storeThisRow[receive_Stat.CURRENT.getValue()]);  // current feedback
    newRow.setFloat("data 15", storeThisRow[receive_Stat.POSITION.getValue()]); // position feedback [rev]
    newRow.setFloat("data 16", storeThisRow[receive_Stat.SPEED.getValue()]);    // speed feedback [krpm]
    //newRow.setFloat("data 17", storeThisRow[receive_Stat.TIMER.getValue()]);  // counter
    newRow.setFloat("data 17", motionATTSequenceIndicator);    
  }
  else if(run_ATT){
    //newRow.setFloat("Time[cntr]", storeThisRow[receive_Stat.TIMER.getValue()]);
    //newRow.setFloat("Current", storeThisRow[receive_Stat.CURRENT.getValue()]);
    //newRow.setFloat("Arm Position", storeThisRow[receive_Stat.POSITION.getValue()]);
    //newRow.setFloat("Torque", storeThisRow[receive_Stat.TORQUE.getValue()]);
    //newRow.setFloat("Speed", storeThisRow[receive_Stat.SPEED.getValue()]);
    //newRow.setFloat("inputSig", targetIref);
    newRow.setFloat("data 1",currentStimuli_mA);  // calculated current
    newRow.setFloat("data 2",currentStimuli_mNm); // converted torque    
    newRow.setInt  ("data 3",sequence_flx);
    newRow.setInt  ("data 4",sequence_ext);
    newRow.setFloat("data 5",absTorqExt_mNm);    // for each side, absolute calculated torque 
    newRow.setFloat("data 6",absTorqFlx_mNm);
    newRow.setInt  ("data 7",response); 
    newRow.setInt  ("data 8",seriesNum_flx);
    newRow.setInt  ("data 9",seriesNum_ext);  
    newRow.setFloat("data 10", avgdTransitionTorqExt_mNm); // converging value
    newRow.setFloat("data 11", avgdTransitionTorqFlx_mNm); // converging value 
    newRow.setInt  ("data 12", sequenceIndicator.getValue()); // state number 
    newRow.setFloat("data 13", storeThisRow[receive_Stat.TORQUE.getValue()]);         
    newRow.setFloat("data 14", storeThisRow[receive_Stat.CURRENT.getValue()]);  // current feedback
    newRow.setFloat("data 15", storeThisRow[receive_Stat.POSITION.getValue()]); // position feedback [rev]
    newRow.setFloat("data 16", storeThisRow[receive_Stat.SPEED.getValue()]);    // speed feedback
    newRow.setFloat("data 17", storeThisRow[receive_Stat.TIMER.getValue()]);    // counter
  }
  else {
    // Logging section for the tracking experiment
    if(logLoadCell){
      newRow.setFloat("data 1", storeThisRow[receive_Stat.TORQUE.getValue()]);
      //newRow.setFloat("data 3", storeThisRow[receive_Stat.CURRENT.getValue()]);  // current feedback
      newRow.setFloat("data 2", storeThisRow[receive_Stat.SPEED.getValue()]);  // current feedback      
      newRow.setFloat("data 3", storeThisRow[receive_Stat.TIMER.getValue()]);    // data to log
      newRow.setFloat("data 4", storeThisRow[receive_Stat.POSITION.getValue()]);    // data to log      
    }
  }
} // end of storeData() function

// ----------------------------------------------<< Interrupts >>---------------------------------------------
void serialEvent(Serial P)
{
  if (myPort.available() > 0){
    short incomingChar = (short)myPort.readChar();
    //if (parityCheck(incomingChar) == 0) 
    //{
      if(assy16Bit)
      {
        switch(incomingDAT_status)
        {
          case CURRENT:
          if(HL){
            MSB = incomingChar;
            assy16Bit = false;
            short dataRead = (short)(LSB | (MSB<<8) );
            incomingCurrent = ((float)dataRead); // /SCALE_FACTOR; to make it in [mA]
            storeThisRow[incomingDAT_status.getValue()] = incomingCurrent;
          } else LSB = incomingChar; HL = true; // LSB first.
          break;
          
          case POSITION:
          if(HL){
            MSB = incomingChar;
            assy16Bit = false;
            short dataRead = (short)(LSB | (MSB<<8) );
            incomingPosition = ((float)dataRead)/SCALE_FACTOR;
            storeThisRow[incomingDAT_status.getValue()] = incomingPosition;
          } else LSB = incomingChar; HL = true;
          break;
          
          case TORQUE: 
          if(HL){
            MSB = incomingChar;
            assy16Bit = false;
            short dataRead = (short)(LSB | (MSB<<8) );
            incomingTorque = ((float)dataRead); // /SCALE_FACTOR; to make it in [mNm]
            storeThisRow[incomingDAT_status.getValue()] = incomingTorque;            
          } else LSB = incomingChar; HL = true;
          break;
          
          case SPEED: 
          if(HL){
            MSB = incomingChar;
            assy16Bit = false;
            short dataRead = (short)(LSB | (MSB<<8) );
            incomingSpeed = ((float)dataRead)/EXTRA_SCALE_FACTOR;
            storeThisRow[incomingDAT_status.getValue()] = incomingSpeed;                        
          } else LSB = incomingChar; HL = true;
          break;
          
          case TIMER: // DUTY CYCLE C
          if(HL){
            MSB = incomingChar;
            assy16Bit = false;
            short dataRead = (short)(LSB | (MSB<<8) );
            
            if(!logLoadCell){                                         // Modifed to be four FSR indicators
              incomingTimeStamp = dataRead;
              storeThisRow[incomingDAT_status.getValue()] = incomingTimeStamp;
            } else {incomingLogData = ((float)dataRead)/SCALE_FACTOR; // No longer timer count. Additional data.
                    storeThisRow[incomingDAT_status.getValue()] = incomingLogData;}                          
            
            if (enable_Recording) {storeData();} // do not use thread here: data messed up.
  
            myPort.write('c');
            myPort.write(LSB_t);
            myPort.write(MSB_t);
          } 
          else {
            LSB = incomingChar; HL = true; 
            LSB_t = (short)(((int)(targetIref)) & 0xFF); // send the current in [mA]
            MSB_t = (short)((((int)(targetIref))>>8) & 0xFF);
          }
          break;
          
        } // end of switch
      } // end of if
      else{
        switch (incomingChar)
        {
          case 'c':
            incomingDAT_status = receive_Stat.CURRENT;
            break;
          case 'p':
            incomingDAT_status = receive_Stat.POSITION;
            break;
          case 't':
            incomingDAT_status = receive_Stat.TORQUE;
            break;
          case 's':
            incomingDAT_status = receive_Stat.SPEED;
            break;
          case 'T':
            incomingDAT_status = receive_Stat.TIMER;
            break;
          
          // data ID's MSB is 0. This triggers the actual data assembly followed by IDing.
          case 0: 
            assy16Bit = true;
            MSB = LSB = 0;
            HL = false;
            break;
        } // end of switch
      } // end of else
    //} // end of if(paritycheck)
  } // end of if(myPort.available)  
}

void keyPressed() {
   // debugging purpose: manually access the experiment or change the input
   if (key == CODED) {
    if (keyCode == UP) { targetIref = targetIref+inputResolution; }
    else if (keyCode == DOWN) { targetIref = targetIref-inputResolution; }   
  } // end of pressing up & down
  
  else if (key == 's') { //--- skipping the step
    if (enablePART_I) {
      if ((testProcedure == 1)||(testProcedure == 2)){testProcedure = 3;     resetParameters(); sequenceIndicator = SEQUENCE_INDICATOR.APPLYandLISTEN;println("Moving to the pre-load session"); }
      else if(testProcedure == 3){testProcedure = 4;resetParameters(); sequenceIndicator = SEQUENCE_INDICATOR.IDLE;println("Moving to the constant speed session"); }
    }
  } // end of pressing 's'
  
  // == user reponse ==
  else if (key == 'r') {response_ext = 1;} // iq_ref(-)
  else if (key == 'f') {response_flx = 1;} // iq_ref(+)

  // == user input for the experimental setup
  else if (key == '1') {
    if (enablePART_II) {
      pathType = 1; println("path # 1 selected");
      readingTrajectoryData = true;
      dataReadFinished = false;
    } else {enablePART_I = true; println("PART 1 selected");}
  } // specify the trajectories we want to test
  else if (key == '2') {
    if (enablePART_II) {
      pathType = 2; println("path # 2 selected");
      readingTrajectoryData = true;
      dataReadFinished = false;
    } else {enablePART_II = true; println("PART 2 selected");}
  }
  else if (key == '3') {
    if (enablePART_II) {
      pathType = 3; println("path # 3 selected");
      readingTrajectoryData = true;
      dataReadFinished = false;
    }
  }

  // == initiate the experiment ==
  else if (key == ENTER | key == RETURN) {
    if (enablePART_I) {
      beginTest = true;
      if((testProcedure==2)||(testProcedure==3)||(testProcedure==5)){enable_Recording = true; println("Initiate the experiment with recording");}
      else {println("Start experiment");}
    }
    else /*eablePART_II == 1*/ {
      beginTest = true;
    }
    //enable_Recording = true;
  }

  // == for PART 2: terminate the experiment (not the program itself) & save the table == 
  if (key == ' ') {     //--- Terminate when key is pressed
    stopRecordingAndSave();
  }
}   // end of KeyPressed()

//void keyReleased(){   }

// ----------------------------------------------<< MISCs >>--------------------------------------------- 

//! def\ Update the recent average value and terminate the experiment
void exitConditionMet(){
  myPort.write(124);  
  myPort.stop();
  exit();
}

void stopRecordingAndSave(){  
  
  // == save the data == 
  // Recall: method 1(M1) is included inside the testProcedure of 2. testProcedure of 1 is practice session.
  // calculate the average for: the ATT and JND:
  avgdTransitionTorqExt_mNm      = 0.5 * (transitionTorqExt_mNm_prev + transitionTorqExt_mNm_prevPrev);
  avgdTransitionTorqFlx_mNm      = 0.5 * (transitionTorqFlx_mNm_prev + transitionTorqFlx_mNm_prevPrev);

  String date       = "021820";
  String subFolder  = "log/";
  String labNum     = "lab12b_";
  String SubjectName= "_bemf1_";
  String methodNum  = "";
  if ((testProcedure == 2) || (testProcedure == 3)){
    switch (ATTcondition){
      case 0:
        methodNum  = "M1";
        break;
      case 1:
        methodNum  = "M2W1E";
        break;
      case 2:
        methodNum  = "M2W2E";
        break;
      case 3:
        methodNum  = "M2W1F";
        break;      
      case 4:
        methodNum  = "M2W2F";
        break;              
    }
  } // case for ATT
  else if(testProcedure == 5) {methodNum  = "M3" + str(ATTMcondition);}    
  String extension  = ".csv";
  String saveFilePath = subFolder+date+"/"+labNum+date+SubjectName+methodNum+extension;
  saveTable(logTable,saveFilePath); logTable.clearRows(); // save and clear for the future work
  println("test condition: "+testProcedure);
  println("ATT itr: "+ATTcondition);
  println("ATTM itr: "+ATTMcondition);  
  println("data saved: "+methodNum);
  println("========================");
  
  // == Every experiment cycle is done, determine where to move from here. (this parameter never reset once program runs)
  // in part I:
  if (enablePART_I) {
    if (run_ATT) {
      if ((testProcedure == 2) || (testProcedure ==3)) {
        if (ATTcondition < 4) {ATTcondition++;} 
        else {testProcedure = 4;}
      }
    }// end of run_ATT
    if (run_cnstSPDTorq) {
      if (testProcedure == 5){
        if (ATTMcondition < 1) {ATTMcondition++;}
        else {testProcedure = 6;}
      }
    }// end of run_cnstSPDTorq
  } 
  
  // == Determine if the program has completed all sequences and reset
  if (testProcedure == 6) { exitConditionMet();}
  resetParameters(); // reset all the preset values for the continuous running.
  println("Parameters reset");
}

void resetParameters(){
   /*Init Val*/absTorqExt_mNm = 700;
   /*Init Val*/absTorqFlx_mNm = 700;
   /*Init Val*/absStepSizeExt_mNm = 100;
   /*Init Val*/absStepSizeFlx_mNm = 100;
   targetIref = 0;
   currentStimuli_mNm = 0;                             // current stimuli, torque
   //**{Stair Case Method}**
   run_ATT = false;
   currentStimuli_mA = 0;                              // current stimuli, current
   mainCounter = 0;
   response_ext = 0;                                    // initial 2, depends on the user's response 0 or 1
   response_flx = 0;                                    // initial 2, depends on the user's response 0 or 1 
   response = 0;                                        // initial 2, depends on the user's response 0 or 1
   responseExt_reCnstrctd_prev = 0;                                   // initial 2, depends on the user's response 0 or 1
   responseFlx_reCnstrctd_prev = 0;
   responseExt_reCnstrctd = 0;
   responseFlx_reCnstrctd = 0;
   seriesNum_flx = 1;
   seriesNum_ext = 1;
   sequence_flx = 1; 
   sequence_ext = 1;
  
   inputDirection = ' ';                                      // state variable 
   inputDirection_prev = ' ';
   inputDirection_extMotion = ' ';
   inputDirection_extMotion_prev = ' ';
   inputDirection_flxMotion = ' ';
   inputDirection_flxMotion_prev = ' ';
   absTorqExt_mNm_prev = 0;                          
   absTorqFlx_mNm_prev = 0;
   isFlexionDone = false;
   isExtensionDone = false;
   randomCounterNumber = 0;                              // counter storing the randomly picked delaying time
   userCue_toggle = 0;
   userCue_transition = false;
   tempTorCnst = 0;
  
  //**{Constant speed perturbation }**
   /*Init Val*/absTorqFlx_aid_mNm  = 700;
   /*Init Val*/absTorqFlx_hurt_mNm = 700;
   /*Init Val*/absTorqExt_aid_mNm  = 700;
   /*Init Val*/absTorqExt_hurt_mNm = 700;
   /*Init Val*/absStepSizeExt_AID_mNm = 100;
   /*Init Val*/absStepSizeFlx_AID_mNm = 100;
   /*Init Val*/absStepSizeExt_HURT_mNm = 100;
   /*Init Val*/absStepSizeFlx_HURT_mNm = 100;
   
   targetPos4GUI_deg = 0;                                    // Target position input: 0 ~ 120 degree 
   run_cnstSPDTorq = false;
   aVelocity_deg_per_frameRate = 0.0;
   aAcceleration_deg_per_frameRateSquare = 0.0;
   farmAngle = 0.0;                   // converted encoder reading, in [rad]
                                                      // default, false => section 2~3, activate the torque input; when the error is big, becomes true and bypass the torque input
   outOfThreshold_flag = 0;       // flag that allows to apply the torque: flag on during the section 1~2 or 2~3, and reset on APPLYandLISTEN 
   stateMachineToggle = 0;        // toggle to make sure to hit the state machine only once during the sequence
   applyTorq_toggle = 0;          // toggle on by APPLYandLISTEN state -> off by the end of intervalCounts_constant_speed
   method_three_Counter = 0;              // counter for running the interval counter for the constant speed -> only starts when the conditons are met: 
                                                      // 1) constant speed region, 2) flag on during the section 1~2 or 2~3 tells ok
   responseFlx_reCnstrctd_aid = 0;
   responseFlx_reCnstrctd_hurt = 0;
   responseExt_reCnstrctd_aid = 0;
   responseExt_reCnstrctd_hurt = 0;
   responseFlx_reCnstrctd_aid_prev = 0;
   responseFlx_reCnstrctd_hurt_prev = 0;
   responseExt_reCnstrctd_aid_prev = 0;
   responseExt_reCnstrctd_hurt_prev = 0;
   isFlexionDone_aid = false;
   isFlexionDone_hurt = false;
   isExtensionDone_aid = false;
   isExtensionDone_hurt = false;
   absTorqFlx_aid_mNm_prev = 0;
   absTorqFlx_hurt_mNm_prev = 0;
   absTorqExt_aid_mNm_prev = 0;
   absTorqExt_hurt_mNm_prev = 0;
   sequence_flx_aid = 1;
   sequence_flx_hurt = 1;
   sequence_ext_aid = 1;
   sequence_ext_hurt = 1;
   seriesNum_flx_aid = 1;
   seriesNum_flx_hurt = 1;
   seriesNum_ext_aid = 1;
   seriesNum_ext_hurt = 1;
   transitionTorqFlx_mNm_prev = 0;
   transitionTorqFlx_mNm_prevPrev = 0;
   transitionTorqExt_mNm_prev = 0;
   transitionTorqExt_mNm_prevPrev = 0;
   transitionTorqFlx_aid_mNm_prev = 0;
   transitionTorqFlx_aid_mNm_prevPrev = 0;
   transitionTorqFlx_hurt_mNm_prev = 0;
   transitionTorqFlx_hurt_mNm_prevPrev = 0;
   transitionTorqExt_aid_mNm_prev = 0;
   transitionTorqExt_aid_mNm_prevPrev = 0;
   transitionTorqExt_hurt_mNm_prev = 0;
   transitionTorqExt_hurt_mNm_prevPrev = 0;
   avgdTransitionTorqFlx_aid_mNm = 0;
   avgdTransitionTorqFlx_hurt_mNm = 0;
   avgdTransitionTorqExt_aid_mNm = 0;
   avgdTransitionTorqExt_hurt_mNm = 0;
   avgdTransitionTorqExt_mNm = 0;
   avgdTransitionTorqFlx_mNm = 0;
   motionATTSequenceIndicator = 0;      // 1~4; 1-flx aid, 2-flx hurt, 3-ext aid, 4-ext hurt.
  motionATTSequenceIndicator_prev = 0;
  
   dataReadFinished = false;                 // toggles whether the desired trajectory is ready or not.
   pathType = 0;
   readingTrajectoryData = false;
   enable_Recording = false;
   beginTest = false;
   
   miniGame_currentScore = 0;
}

//! Path Threading: threading through the corresponding text files.
//! Trajectory is generated at 100 Hz sampling frequencey which is equivalent to current draw() loop.
void readingTrajectoryData(){
  

    if (pathType == 1){   // 1st path profile
      String[] loadedPath = loadStrings("traj_1.txt");

      float[] temp_array;

      int pathLength = loadedPath.length; // stores the length of the text script
      positionCMDLoaded = new float[pathLength];

      for (int i = 0; i < pathLength; i = i + 1){
        temp_array = float(split(loadedPath[i], ' '));
        positionCMDLoaded[i] = temp_array[1]; // 0 is time, 1 is position, and 2 is speed
      }
      
      dataReadFinished = true;      
    }

    //case -1:  // 1st path profile, backward
    //  String[] loadedPath = loadStrings("traj_1r.txt");

    //  float[] temp_array;

    //  int pathLength = loadedPath.length; // stores the length of the text script
    //  positionCMDLoaded = new float[pathLength];

    //  for (int i = 0; i < pathLength; i = i + 1){
    //    temp_array = float(split(loadedPath[i], ' '));
    //    positionCMDLoaded[i] = temp_array[1]; // 0 is time, 1 is position, and 2 is speed
    //  }
      
    //  dataReadFinished = 1;      
    //  break;

    else if (pathType == 2){   // 2nd path profile
      String[] loadedPath = loadStrings("traj_2.txt");

      float[] temp_array;

      int pathLength = loadedPath.length; // stores the length of the text script
      positionCMDLoaded = new float[pathLength];

      for (int i = 0; i < pathLength; i = i + 1){
        temp_array = float(split(loadedPath[i], ' '));
        positionCMDLoaded[i] = temp_array[1]; // 0 is time, 1 is position, and 2 is speed
      }
      
      dataReadFinished = true;      
    }

    //case -2:   // 2nd path profile, backward
    //  String[] loadedPath = loadStrings("traj_2r.txt");

    //  float[] temp_array;

    //  int pathLength = loadedPath.length; // stores the length of the text script
    //  positionCMDLoaded = new float[pathLength];

    //  for (int i = 0; i < pathLength; i = i + 1){
    //    temp_array = float(split(loadedPath[i], ' '));
    //    positionCMDLoaded[i] = temp_array[1]; // 0 is time, 1 is position, and 2 is speed
    //  }
      
    //  dataReadFinished = 1;      
    //  break;

    else if (pathType == 3){   // 3rd path profile
      String[] loadedPath = loadStrings("traj_3.txt");

      float[] temp_array;

      int pathLength = loadedPath.length; // stores the length of the text script
      positionCMDLoaded = new float[pathLength];

      for (int i = 0; i < pathLength; i = i + 1){
        temp_array = float(split(loadedPath[i], ' '));
        positionCMDLoaded[i] = temp_array[1]; // 0 is time, 1 is position, and 2 is speed
      }
      
      dataReadFinished = true;      
    }      

    //case -3:   // 3rd path profile, backward
    //  String[] loadedPath = loadStrings("traj_3r.txt");

    //  float[] temp_array;

    //  int pathLength = loadedPath.length; // stores the length of the text script
    //  positionCMDLoaded = new float[pathLength];

    //  for (int i = 0; i < pathLength; i = i + 1){
    //    temp_array = float(split(loadedPath[i], ' '));
    //    positionCMDLoaded[i] = temp_array[1]; // 0 is time, 1 is position, and 2 is speed
    //  }
      
    //  dataReadFinished = 1;
    //  break;

}


//! PAD: Position Arm Desired, PFD: Position Frame Desired.
float phaseLEAD(float trajectoryAngle)
{
  PFD_k = cPAD_k*PAD_k + cPAD_k_1*PAD_k_1 + cPAD_k_2*PAD_k_2 + cPFD_k_1*PFD_k_1 + cPFD_k_2*PFD_k_2;  
  PAD_k = trajectoryAngle;
  PAD_k_1 = PAD_k;
  PAD_k_2 = PAD_k_1;
  PFD_k_1 = PFD_k;
  PFD_k_2 = PFD_k_1;
  return PFD_k;  
}

//! \def     Low pass filter 
//! \detail  Tuned for the backgroundFrameRate(100Hz)
float JND_LPF (int inputMag)
{
  ST_OUT_K = ST_OUT_K_1*cST_OUT_K_1 + ST_IN_K*cST_IN_K + ST_IN_K_1*cST_IN_K_1;
  ST_IN_K = inputMag;
  ST_IN_K_1 = ST_IN_K; 
  ST_OUT_K_1 = ST_OUT_K;
  return ST_OUT_K;
}
