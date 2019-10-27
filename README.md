# vision-robot
Python program running on a Raspberry Pi with RPi Camera to analyze a video stream for part recognition. The parts' positions are detected, tracked, and then sent to the register memory of an industrial cartesian robot's PLC. The PLC calculates an interception position and sends a series of coordinated pulses to its servo motors to move exactly to the part's current position. This is when a piston and suction cup lift the part off of the moving conveyor.
Note: The PLC's program was written in ladder on Delta ISPSoft, it is uploaded as a PDF.
