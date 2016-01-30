import subprocess
import os
import os.path

cmd = ['./bin/WritePCDFile']
p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
p.wait()
if p.returncode!=0:
    print "Error executing c++ program!"
    quit()
else:
    if not os.path.isfile("point_cloud_raw.pcd"):
        print "Some strange error occurred. pcd file not found..."
        quit()
    f = open("point_cloud_raw.pcd")
    num_lines = len(f.readlines())
    f.close()
    fout = open("point_cloud.pcd", "w")
    fin = open("point_cloud_raw.pcd")
    fout.write("VERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n")
    fout.write("WIDTH "+str(num_lines)+"\n")
    fout.write("HEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n")
    fout.write("POINTS "+str(num_lines)+"\n")
    fout.write("DATA ascii\n")
    for l in fin:
        fout.write(l)
    fout.close()
    os.remove("point_cloud_raw.pcd")
    print "Point cloud generated successfully!"
