# Psuedo code for the Psychophysics tool

import libraries  
define variables  
&nbsp;&nbsp;&nbsp;&nbsp; 1: enablePARTI = 0	&nbsp;&nbsp;// for calling ATT() and cnstSPDTorq()  
&nbsp;&nbsp;&nbsp;&nbsp; 2: enablePARTII = 0	&nbsp;&nbsp;// for calling trajTracking()   
&nbsp;&nbsp;&nbsp;&nbsp; 3: testProcedue = 1	&nbsp;&nbsp;// Indicator for the general state machine  
&nbsp;&nbsp;&nbsp;&nbsp; 4: sequenceIndicator	&nbsp;&nbsp;// Indicator for the ATT() State Machine (S.M.): (CALCULATE(0),APPLYandLISTEN(1),IDLE(2))  
&nbsp;&nbsp;&nbsp;&nbsp; 5: motionATTSequenceIndicator = 0 &nbsp;&nbsp;// Indicator for the cnstSPDTorq() S.M. (1-4)  
&nbsp;&nbsp;&nbsp;&nbsp; 6: ATTcondition = 0	&nbsp;&nbsp;// interal state counter for ATT()  
&nbsp;&nbsp;&nbsp;&nbsp; 7: ATTMcondition = 0	&nbsp;&nbsp;// interal state counter for cnstSPDTorq()  


setup  
{  
&nbsp;&nbsp;&nbsp;&nbsp; 1: Set a base GUI  
&nbsp;&nbsp;&nbsp;&nbsp; 2: Set serial communication  
&nbsp;&nbsp;&nbsp;&nbsp; 3: Set a lookup table for exponential coefficient //determines the converging speed  
&nbsp;&nbsp;&nbsp;&nbsp; 4: Set the logTable  
&nbsp;&nbsp;&nbsp;&nbsp; 5: Set framerate for draw()  
&nbsp;&nbsp;&nbsp;&nbsp; 6: Initialize sequenceIndicator to APPLYandLISTEN  
&nbsp;&nbsp;&nbsp;&nbsp; 7: Initialize SCI communication with CCS //send an ASCII code  
}  
  
draw  
{  
&nbsp;&nbsp;&nbsp;&nbsp; **if** (motionATTSequenceIndicator != motionATTSequenceIndicator_prev) **then** play auditory cue  
&nbsp;&nbsp;&nbsp;&nbsp; **if** (frameCount % 10 == 0) **then** // GUI framerate   
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; **if**(run_cnstSPDTorq = true) **then** // running cnstSPDTorq()  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; 1: Update GUI for Psychophysics tool for motion  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; 2: **if** (testProcedure = 4) **then** // "minigame" of cnstSPDTorq()  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; **switch**(motionATTSequenceIndicator) // cnstSPDTorq() S.M.
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**case** 1 calculate the GUI for flx dir & flx torq  
