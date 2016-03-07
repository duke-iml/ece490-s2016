# ece490-s2016 (group2)

Spatula_Klampt.STL is the original file. A copy of this file is in the planning/klampt_models directory. The robot model "baxter_with_basic_spatula_col.rob" uses the copy, not this file. 

Spatula_Klampt_part1.STL and Spatula_Klampt_part2.STL are the original files. Copies are in the planning/klampt_models directory. The robot model "baxter_with_spatula_col.rob" uses the copy, not this file. 

Sometimes, Klampt/RobotTest will throw an error importing an STL file (saying that the file is empty). Refer to the following link to fix this issue (https://libraries.io/github/blodow/realtime_urdf_filter). It seems like the error is machine-dependent. 

=======================

Troubleshooting
Every once in a while, assimp fails when importing STL files. If the first 5 bytes are "solid", it treats it as ASCII, however there are several binary STL files around that start with "solid". You'll get an error message along the lines of:

    [ERROR] [1360339850.748534073]: Could not load resource [package://pr2_description/meshes/sensors/kinect_prosilica_v0/115x100_swept_back--coarse.STL]: STL: ASCII file is empty or invalid; no data loaded
You can double check with e.g.:

    hexdump -C bad_stl_file.STL | head
In that case, a simple work around (read: "hack") is to replace the "solid" with "rolid", and assimp loads it as a binary file.

    printf 'r' | dd of=bad_stl_file.STL bs=1 seek=0 count=1 conv=notrunc 
I'm not exactly sure why RViz does not seem to have this problem.
