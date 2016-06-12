#readFromFile.py

f=open('IKLog.txt', 'r')
#fOut=open('CleanIKlog.txt', 'w')

#fileContents = f.read()

binSuccess = {}
binFailure = {}

for line in f:
	lineArray = line.split(' ')
	if 'bin_' in lineArray[-1]:
		print line 
		if lineArray[-4] == 'Successful':
			if lineArray[-1] in binSuccess:
				binSuccess[lineArray[-1]] = binSuccess[lineArray[-1]]+1
			else:
				binSuccess[lineArray[-1]] = 1
				binFailure[lineArray[-1]] = 0 
		else:
			if lineArray[-1] in binFailure:
				binFailure[lineArray[-1]] = binFailure[lineArray[-1]]+1
			else:
				binFailure[lineArray[-1]] = 1
				binSuccess[lineArray[-1]] = 0

for bin in binSuccess:
	print 'For ', bin, ' number of successes is ', binSuccess[bin], ' and number of failures is ', binFailure[bin] 
