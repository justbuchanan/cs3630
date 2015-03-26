import time
from Myro import *

# Run commands to move the robot
def runCommands(log, commands):
    for c in commands:
      start = time.time()
      motors(c[0],c[1])
      while (time.time() - start < c[2]):
        logNow(log, c[0],c[1],0);
        time.sleep(0.1) # Read sensors at 1Hz

# Run commands followed by taking picture        
def runCommandAndTakePicture(log, cmd):

    # Move the robot
    commands = [];
    commands.append(cmd);
    commands.append([0,0,0.1]);
    runCommands(log,commands);
    
    # Take picture
    pic_fname = "pic-%d.jpg" % time.time()
    picture = takePicture();
    logNow(log, 0,0,pic_fname);
    savePicture(picture, pic_fname)

# Log motor commands
def logNow(log, l, r, fname):
    log.write("%s %s %s\n" %(l, r, fname))

fname = "log-%d.txt" % time.time()
log = open(fname, 'w')
init("/dev/tty.Fluke2-0610-Fluke2")

#print 'Connected!'
setIRPower(140)
setForwardness(1)

# Run the robot
runCommandAndTakePicture(log, [1,1,1]);
runCommandAndTakePicture(log, [0,1,0.2]);
runCommandAndTakePicture(log, [0,1,0.2]);
runCommandAndTakePicture(log, [0,1,0.2]);
runCommandAndTakePicture(log, [1,1,1]);
runCommandAndTakePicture(log, [1,0,0.2]);
runCommandAndTakePicture(log, [1,0,0.2]);
runCommandAndTakePicture(log, [1,0,0.2]);
runCommandAndTakePicture(log, [1,1,1]);
runCommandAndTakePicture(log, [0,1,0.2]);
runCommandAndTakePicture(log, [0,1,0.2]);
runCommandAndTakePicture(log, [0,1,0.2]);
runCommandAndTakePicture(log, [1,1,1]);



log.close();
