# mmd
Multiple motor driver

mmd drives up to 6 stepper-motors in parallel. By now it supports linear movement with speed up and speed down ramps including changing the speed during the movement. You can drive to a given position or make a constant move/stop with the keyboard-keys with every motor. The endswitches (if any) stops the movement.
You can reference each axis to a value you choose, the programm will store this value and the actual position automatically in a file.
The relation between units/steps, the minimum and maximum speed and the acceleration can be given in an xml-file for each axis.
The software is runnig on a raspberry pi with a realtime linux-kernel. 
We use this code in our optics lab to drive linear and rotation stages.

Each motor should be connected by step/dir/stop1/stop2 on the raspbis gpio pins. 
The programm has a simple terminal-interface showing the positions, and let you set target positions or move directly with +/-. 
Per ssh you can drive the motors easily from any computer.
Alternativley one or more positions can be given in a xml-File and be loaded/executed.



