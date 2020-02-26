Psuedo code for the Psychophysics tool

import libraries  
define variables  
&nbsp;&nbsp;&nbsp;1: enablePARTI = 0			// for calling ATT() and cnstSPDTorq()  
&nbsp;&nbsp;&nbsp;2: enablePARTII = 0			// for calling trajTracking()   
&nbsp;&nbsp;&nbsp;3: testProcedue = 1			// Indicator for the general state machine  
&nbsp;&nbsp;&nbsp;4: sequenceIndicator			// Indicator for the ATT() state machine (CALCULATE(0),APPLYandLISTEN(1),IDLE(2))  
&nbsp;&nbsp;&nbsp;5: motionATTSequenceIndicator = 0 	// Indicator for the cnstSPDTorq() state machine (1-4)  
&nbsp;&nbsp;&nbsp;6: ATTcondition = 0			// interal state counter for ATT()  
&nbsp;&nbsp;&nbsp;7: ATTMcondition = 0			// interal state counter for cnstSPDTorq()  


setup
{
	1: Set a base GUI 
	2: Set serial communication 
	3: Set a lookup table for exponential coefficient //determines the converging speed
	4: Set the logTable
	5: Set framerate for draw()
	6: Initialize sequenceIndicator to APPLYandLISTEN
	7: Initialize SCI communication with CCS //send an ASCII code
}

draw
{
	if (motionATTSequenceIndicator != motionATTSequenceIndicator_prev) 
		then 
			play auditory cue

	if (frameCount % 10 == 0) // GUI framerate
		then 
			if(run_cnstSPDTorq = true)						// running cnstSPDTorq()
				then
					1: Update GUI for Psychophysics tool for motion
					2: if (testProcedure = 4) 				// "minigame" of cnstSPDTorq()
						then
							switch(motionATTSequenceIndicator) 	// cnstSPDTorq() state machine (1-4)
								case 1 
									then calculate the 