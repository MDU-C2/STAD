
# Camera Sample

This repository contains a camera sample built from the template class which utilizes the driveworkssample class. 

The code is designed to handle up to 8 cameras, which should be connected in order from A0 to B3.

Additionally, the code should be extandable to do object detection on images by incorporating the driveworks drivenet or a custom AI model.


## Running the Code

The application can be executed through command-line prompts with the following options:

-  **offscreen**: Run the application without a GUI.

-  **record**: Save the video captured by the cameras (Note: there may be a blue tint issue in the recorded video).

Example command:

    ./sample_template  --offscreen=1  --record=1


## Compiling the code:

-  The  code  needs  to  be  compiled  within  a  driveworks  development  environment.  If not already set up follow  the  guide  for  setting  up  an  environment  for  PX2.

-  Place  the  code  under  the  `src/`  directory  and  add  it  to  the  `CMakeLists.txt`  file.

-  Ensure  that  the  framework  folder  is  built since driveworksSample from that folder is needed.

Note: There should exist a computer (if not lost in the chaos at C2) with ubuntu 16.04 and driveworks installed which has ssh setup and can be used for development. If there is no computer at c2 use the guide to set up a vm or other computer with Ubuntu 16.04.
  

