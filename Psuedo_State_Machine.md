Psuedo code for the Psychophysics tool


import libraries
define variables
	1: enablePARTI = 0			// for calling ATT() and cnstSPDTorq()
	2: enablePARTII = 0			// for calling trajTracking() 
	3: testProcedue = 1			// Indicator for the general state machine
	4: sequenceIndicator			// Indicator for the ATT() state machine (CALCULATE(0),APPLYandLISTEN(1),IDLE(2))
	5: motionATTSequenceIndicator = 0 	// Indicator for the cnstSPDTorq() state machine (1-4)
	6: ATTcondition = 0			// interal state counter for ATT()
	7: ATTMcondition = 0			// interal state counter for cnstSPDTorq()


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