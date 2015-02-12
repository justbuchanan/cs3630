import time
from Scribbler2 import *

# Connect to the scribbler
# Set timeout to 0 to read instantly, non-blocking

fname = "log-%d.txt" % time.time()

s = Scribbler2('/dev/tty.scribbler',fname)

# Set timeout to zero
print 'Connected!'
s.setIRPower(140)
s.setForwardness(1)
# Create a list of commands
commands = []
 

rotateCmd = [200,-200,0.75]

commands.append([200,200,6.848])
commands.append(rotateCmd)
commands.append([200,200,6.032])
commands.append(rotateCmd)
commands.append([200,200,2])

# Command is a list [leftMotor, rightMotor, time]
# Setting motors to 200 will drive 
# forward with the fluke facing forward
print ("Start!")
for c in commands:
  start = time.time()
  s.setMotors(c[0],c[1])
  while (time.time() - start < c[2]):
    s.getObstacle(1)
    time.sleep(0.1) # Read sensors at 1Hz


s.setMotors(0,0)
