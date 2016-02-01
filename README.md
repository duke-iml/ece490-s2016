# ece490-s2016
ECE 490 Spring 2016 shared repository
--------------------------------------------------------
Workstation Setup:
--------------------------------------------------------

1. You can only control the robot with Linux OS, so please install Ubuntu 14.04 first. All of the following instructions are based on this.



2. Install ROS indigo. You can follow this link: http://wiki.ros.org/indigo/Installation/Ubuntu


3. Install OpenCV. If you installed ROS, opencv should already be installed. Note that the kinect 2 won't work with opencv 3.0 or higher version, so make sure you have opencv 2.4.x.


4. Install Baxter SDK. Follow this link: http://sdk.rethinkrobotics.com/wiki/Workstation_Setup. You might want to do the last few steps with the baxter robot because you need its serial number (011408P0016). Also if you follow the newest instruction you will get sdk 1.2.0, while the robot has sdk 1.1.1. Please follow this link to downgrade your sdk to 1.1.1: http://sdk.rethinkrobotics.com/wiki/Workstation_Update


5. Install iai-kinect2 ROS package. This is the driver for perception system. Try to follow this link: https://github.com/code-iai/iai_kinect2. 


6. Install python 2.7. I think most system comes with python 2.7.x, so check it first. 


7. Install Scipy, PyOpenGL. With python installed, you can use "sudo apt-get install python-numpy python-scipy python-pip" to install scipy, and then "pip install pyopengl" to install PyOpenGL.


8. Install Klampt and its python bindings. Please follow this link:http://motion.pratt.duke.edu/klampt/tutorial_install.html. I only tried install from source.


9. Install git. Get this repo on your computer. 


10. Some useful softwares, not required but can be helpful: sublime edit, terminator, Pycharm(Try to install the EDU version.The community version somehow is not working...)
