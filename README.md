# mmd
Multiple motor driver

mmd drives up to 6 stepping-motors in parallel. It is runnig on a raspberry pi with a realtime linux. 
We use this software in our optics lab to drive linear and rotation stages.
Each motor can be connected by step/dir/stop1/stop2 on the raspbis gpio pins. The units of motions/step can be given in a configfile.
The programm has a simple terminal-interface showing the positions, and let you set target positions or move directly with +/-. 
Alternativley one or more positions can be given in a xml-File and be loaded/executed.



