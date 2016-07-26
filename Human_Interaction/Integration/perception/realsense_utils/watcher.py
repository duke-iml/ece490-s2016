import subprocess
from time import sleep

interval = 1

try:
    while True:
        subprocess.call([r'C:\Python27\python.exe', 'server.py'])
        sleep(interval)
        print '!!! task died -> restarting'

except KeyboardInterrupt:
    pass
