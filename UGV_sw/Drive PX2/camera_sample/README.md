This is a camera sample built from the template class built on top of the driveworkssample class.
This code can read up to 8 cameras and should be plugged in from A0 - B3.
The code should be able to be expanded to object detect on the picture by adding the drivenet from driveworks or a custom AI.

The code can be run with commandline prompts and the commands are:
offscreen - this means that the application should be run without a GUI.
record - this saves the video captured by the cameras (for some reason gets a blue tint on the video)
For example:
    ./sample_template --offscreen=1 --record=1

The code needs to be built in a driveworks development environmnet (follow the guide for setting up an environment for PX2) the code should be placed under src/ and added to the CMakeLists.txt file. (The framework folder needs to be built aswell such that driveworksSample etc exists)

There should exist a computer (if not lost in the chaos at C2) with ubuntu 16.04 and driveworks installed which has ssh setup and can be used for development.
