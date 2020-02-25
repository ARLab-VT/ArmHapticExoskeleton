# ArmHapticExoskeleton  
Processing.js and CCS file  
  
- 2/19/20 uploaded files to run the GUI  
  
<Setting up: Processing>  
1. To run the Processing.pde file, it would ask to create its own folder.  
2. Place the files in following path:  
	receive_Stat.java and SEQUENCE_INDICATOR.java -> same directory with the Processing.pde  
3. All the .mov, .mp3, .png files:
	create a folder "data" inside the directory created from step 1, and put the medi files in there.
4. Additionally required library for Processing is minim, to play the audio file

<Setting up: CCS>
1. For the files unzipped, put the files in to the correct folder directory:
	Motorware library should go into c:\ti\
2. Create a new workspace at:
	c:\Users\username\
3. When loading CCS, start from the workspace created from step 2. 
4. Impoart existing project. Path should be from whatever you put the Motorware library
	~\motorware_1_01_00_18\sw\solutions\instaspin_motion\boards\boostxldrv8305_revA\f28x\f2806xM\projects\ccs\proj_lab12b
5. The sci.c is not included by default. (which is required for the serial communication)
	~\motorware_1_01_00_18\sw\drivers\sci\src\32b\f28x\f2806x\sci.c
	- Take the file and copy/drag into the project plane in CCS, under the lab12b project. 
	- Then select "link to file". 
	