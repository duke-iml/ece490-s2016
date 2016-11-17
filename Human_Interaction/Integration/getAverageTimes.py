
import os
import sys
import string

fileOptions = os.listdir('./logs/')

if len(sys.argv) < 2:
	sys.exit(0)
header = 'time'+sys.argv[1]

print fileOptions

myFiles = []
for fileName in fileOptions:
	if fileName.startswith(header):
		myFiles.append(fileName)
#fOut=open('CleanIKlog.txt', 'w')

#fileContents = f.read()


total = {}
total[1] = 0
total[2] = 0
total[3] = 0
total[4] = 0
value = {}


for myfile in myFiles:
	f = open('./logs/'+myfile, 'r')

	counter = 1
	for line in f:
		lineArray = line.split(' ')



		if 'Total' in lineArray[0]:
			#print line 
			value[counter] = float(lineArray[-1])
			total[counter] = total[counter] + value[counter]


			if counter ==4:
				counter = 0

			counter = counter + 1


print 'Average movement 1 time = ', total[1]/len(myFiles)
print 'Average deposit time = ', total[2]/len(myFiles)
print 'Average movement 1 + deposit time = ', total[3]/len(myFiles)
print 'Average total time = ', total[4]/len(myFiles)