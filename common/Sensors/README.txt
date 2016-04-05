Realsense Camera Windows -> ROS / Linux Sender

Kris Hauser



This pair of programs reads from an Intel RealSense camera on a Windows computer and sends data to a Linux computer, which will then re-publish the data via ROS topics.

Both computers must be on the same network, and the Windows computer needs the Intel Perceptual Computing SDK installed as well as the RealSense drivers.  The Linux machine needs ROS.

First, find the IP address of the Windows computer (call it IPADDR). 

On the Windows machine, open up a command prompt and navigate to RealSenseService/bin/Release.

Launch 'RealSenseService.exe tcp://IPADDR:3457' and make sure it continues to run.  You can also add the '-v' flag to make sure it is capturing data.

Now on the windows machine, build and run './RealSense_ROS_Emitter tcp://IPADDR:3457'.  This will publish on the topics:
- /realsense/pc (PointCloud2): the colored point cloud
- /realsense/rgb (Image): the RGB image
- /realsense/depth (Image): the depth image

If you want to change the published topic, add a second argument on the command line giving the topic prefix.  

