APC Low level motion APIs and ROS sensor emitters

***********************************************************************

                             RUNNING THE ROBOT


1. Power up the Baxter and Reflex hands, if using

2. Open a new terminal and start the Baxter Master ROS script 

   cd ~/ros_ws/
   . baxter.sh

3. In new terminal window(s), run the Reflex hand startup script for
   whichever hands are connected to the robot.

   This starts up the service for "reflex_hand1" on USB0.  There is another
   launch script for "reflex_hand2" on USB1.  Note that you may first have to
   change the permissions on the USB port(s) via "sudo chmod 666 /dev/ttyUSB0"

   cd ~/ros_ws
   . baxter.sh
   roslaunch reflex_sf reflex_hand1.launch 

4. To get sensor input from the Kinect, run ./Kinect2_ROS_Emitter.
   This outputs to the /kinect2/{rgb,ir,depth,pc} topics.

   For the RealSense, run ./RealSense_ROS_Emitter and run the
   "Start RealSense Service" link on the Desktop of the Dell Laptop.
   This service will output to the /realsense/{rgb,depth,pc}
   topics.

5. In a new terminal window, you may now run the APC Motion API calls.
   As an example,

   cd ~/ros_ws/
   . baxter.sh
   cd ~/ece590-s2015/hardware_layer
   python joint_printer.py


***********************************************************************

                              INSTALLING

The binaries should be installed on all lab machines.  To install the low
level motion libraries yourself, run:

  cmake -DKLAMPT_ROOT=[path to Klampt] .
  make

You will need to have ROS (Hydro or newer), and the Baxter SDK,
dynamixel-controller, and reflex-sf-apc packages installed on your ROS distribution.

