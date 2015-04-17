import serial, threading
import time
import string
import subprocess
import myro
class Scribbler2:

  def __init__(self, port, filename, baud=38400):
    self.port = port
    self.baud = baud
    self.ser = None
    self.debug = False
    self.l = 0
    self.r = 0
    self.pictures = [];

    self.log = open(filename, 'w')

    self.lock = threading.Lock()
    self.dongle = None
    self.connect()
    self.info = {}
    # Keep talking until we can connect in earnest
    while(len(self.info)==0):
      print "Attemping to get info..."
      self.info = self.getInfo()
      time.sleep(1)
    #self.manual_flush()
    myro.init(port);

  def runCommands(self, commands):
    for c in commands:
      start = time.time()
      self.setMotors(c[0],c[1])
      while (time.time() - start < c[2]):
        self.logNow(0);
        time.sleep(0.1) # Read sensors at 1Hz


  image_codes = {"jpeg": "color",
                   "jpeg-fast": "color",
                   "grayjpeg": "rawgray",
                   "grayjpeg-fast": "rawgray"}

  # Take picture
  def takePicture(self, mode=None):
    """ Takes a picture using the camera. Mode can be 'color', 'gray', or 'blob' """
    return myro.takePicture(mode)

  # Save picture
  def savePicture(self, picture, filename):
    """ Takes a picture using the camera. Mode can be 'color', 'gray', or 'blob' """
    self.logNow(filename);
    self.pictures.append(filename);
    return myro.savePicture(picture, filename)

  # Logging
  def logNow(self, filename):
    print "%s %s %s"%(self.l,self.r,filename)

    if filename != 0:
        fname = filename;
        stripped_fname = fname[0:len(fname)-4];
        filename = stripped_fname+'.pnm';
    
    self.log.write("%s %s %s\n" %(self.l, self.r, filename)
)

  # Close
  def close(self):
    self.log.close();
    
    # convert images to pnm  
    for i in self.pictures:
	fname = i;
        stripped_fname = fname[0:len(fname)-4];
        pnm_fname = stripped_fname+'.pnm';
        command = ['convert', fname, pnm_fname];
        subprocess.call(command);

  def setMotors(self, left=0, right=0):
    if(left < -200):
      left = -200;
    elif(left > 200):
      left = 200

    if(right < -200):
      right = -200;
    elif(right > 200):
      right = 200

    self.l = left
    self.r = right

    # 0 to 200
    #-200 to 200
    left = (left + 200)/2
    right = (right + 200)/2

    self._set(109, right, left)

  # Adjust IR power, best results around 140
  # min: 0 max: 200
  def setIRPower(self, power):
    self.ser.write(chr(120))
    self.ser.write(chr(power))

  def setForwardness(self, val):
    self.ser.write(chr(128))
    self.ser.write(chr(val))

# ----------------------------------------------------------------------------
# ------------------------------Serial Fnc------------------------------------
  def connect(self):
    try:
      self.ser = serial.Serial(self.port, timeout=10)
      self.ser.setDTR(0)
    except serial.SerialException:
      print 'Cannot connect to the scribbler!'
    self.ser.baudrate = self.baud

  def read_2byte(self):
    hbyte = ord(self.ser.read(1))
    lbyte = ord(self.ser.read(1))
    lbyte = (hbyte << 8) | lbyte
    return lbyte

  def manual_flush(self):
    old = self.ser.timeout
    self.ser.setTimeout(.5)
    l = 'a'
    count = 0;
    while (len(l) != 0 and count < 50000):
      l = self.ser.read(1)
      count += len(l)
      self.ser.setTimeout(old)

  def setEchoMode(self, value):
    if value:
      self.write([98, 1])
    else:
      self.write([98, 0])
      time.sleep(.25)
      self.ser.flushInput()
      self.ser.flushOutput()
      return

  def _set(self, *values):
    try:
      self.lock.acquire() #print "locked acquired"
      self.write(values)
      test = self._read(9) # read echo
      self._lastSensors = self._read(11) # single bit sensors
      #self.ser.flushInput()
    finally:
      self.lock.release()

  def write(self, rawdata):
    t = map(lambda x: chr(int(x)), rawdata)
    print t
    data = string.join(t, '') + (chr(0) * (9 - len(t)))[:9]
    self.ser.write(data)

  def _read(self, bytes = 1):
    if self.debug:
      print "Trying to read", bytes, "bytes", "timeout =", self.ser.timeout
    c = self.ser.read(bytes)

    if self.debug:
      print "Initially read", len(c), "bytes:",
      print map(lambda x:"0x%x" % ord(x), c)

    # .nah. bug fix
    while (len(c) > 1 and len(c) < bytes):
      c = c + self.ser.read(bytes-len(c))
      if self.debug:
        print map(lambda x:"0x%x" % ord(x), c)

    # .nah. end bug fix
    if self.debug:
      print "_read (%d)" % len(c)
      print map(lambda x:"0x%x" % ord(x), c)

    if bytes == 1:
      x = -1
      if (c != ""):
        x = ord(c)
      elif self.debug:
        print "timeout!"
        return x
    else:
      return map(ord, c)

  def getInfo(self, *item):

    oldtimeout = self.ser.timeout
    self.ser.setTimeout(4)

    self.manual_flush()
    # have to do this twice since sometime the first echo isn't
    # echoed correctly (spaces) from the scribbler

    self.ser.write(chr(80) + (' ' * 8))
    retval = self.ser.readline()
    #print "Got", retval

    time.sleep(.1)

    self.ser.write(chr(80) + (' ' * 8))
    retval = self.ser.readline()
    #print "Got", retval

    # remove echoes
    if retval == None or len(retval) == 0:
      return {}

    if retval[0] == 'P' or retval[0] == 'p':
      retval = retval[1:]

    if retval[0] == 'P' or retval[0] == 'p':
      retval = retval[1:]

    self.ser.setTimeout(oldtimeout)

    retDict = {}
    for pair in retval.split(","):
      if ":" in pair:
        it, value = pair.split(":")
        retDict[it.lower().strip()] = value.strip()
      if len(item) == 0:
        return retDict
      else:
        retval = []
        for it in item:
          retval.append(retDict[it.lower().strip()])
        if len(retval) == 1:
          return retval[0]
        else:
          return retval

