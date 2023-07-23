# automatic-mast-system

This repository contains the scrips and virtual enviroment used in the development of an automatic mast system to scan enviroments with a RPLidar A2.

Files:

scripts:
HMI.py - It's the script that runs the HMI for the system, "translate" the graphic signals to instructions that are sent thru the serial port to arduino in order to execute that action. <br />
HMI.ui - The .ui file it's the QtDesigner file that it's loaded in the HMI.py script in order to create the HMI.<br />
mast_main.ino - It's the script that it's loaded to the Arduino Mega, we have the Mast Class that contains all the functions to control the movements and scanning of the mast system.

venv:
- Contains a virtual enviroment running Python 3.11.4 and is loaded up with all the required libraries for this project

External:
- Numpy
- PyQt5
- PyQtGraph

Internal:
- time
- sys
- threading
