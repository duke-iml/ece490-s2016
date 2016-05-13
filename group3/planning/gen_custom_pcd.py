POINTS = 100
MAX = -.35

print "# .PCD v0.7 - Point Cloud Data file format"
print "VERSION 0.7"
print "FIELDS x y z"
print "SIZE 4 4 4"
print "TYPE F F F"
print "COUNT 1 1 1"
print "WIDTH 10"
print "HEIGHT 1"
print "VIEWPOINT 0 0 0 1 0 0 0"
print "POINTS " + str(POINTS+1)
print "DATA ascii"
print "0 0 0"

for i in range(POINTS):
	print "0 0 " + str((i+1) * MAX / POINTS)
