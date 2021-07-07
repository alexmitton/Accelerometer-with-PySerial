# IMPORT LIBRARIES
try:
    import serial
except:
    import pip
    pip.main(['install','pyserial'])
    import serial
try:
    import numpy as np
except:
    import pip
    pip.main(['install','numpy'])
    import serial, time, numpy as np
try:
    import matplotlib.pyplot as plt
except:
    import pip
    pip.main(['install','matplotlib'])
    import matplotlib.pyplot as plt
try:
    from alive_progress import alive_bar
except:
    import pip
    pip.main(['install','alive-progress'])
    from alive_progress import alive_bar
import time, sys

# DEFINE FUNCTIONS
# progressbar() : taken from https://stackoverflow.com/a/34482761 - learn how to implement better!
def progressbar(it, prefix="", size=60, file=sys.stdout):
    count = len(it)
    def show(j):
        x = int(size*j/count)
        file.write("%s[%s%s] %i/%i\r" % (prefix, "#"*x, "."*(size-x), j, count))
        file.flush()
    show(0)
    for i, item in enumerate(it):
        yield item
        show(i+1)
    file.write("\n")
    file.flush()

# getValues() : requests, reads and returns data points
def getValues():
    #ser.write(b'y')
    arduinoData = ser.readline().decode().split('\r\n')
    return int(arduinoData[0])

# collectData() : starts data collection for defined duration
def collectData(duration,delay=0.1,n=3):
    vals = np.zeros((1,n),dtype=int)
    valsSeries = np.zeros((int(duration/delay),n),dtype=int)

    print('-' * 25, '\nCOLLECTING DATA\n' + '-' * 25)
    #i = 0 - uncomment for progress bar
    # progress bar could be implemented by using just getValues instead at runtime?
    #for a in progressbar(range(int(duration/delay)), 'Recording: ', 40):
    for i in range(int(duration/delay)): # comment for progress bar
        print('Collecting reading', i + 1)

        for j in range(n):
            data = getValues()
            vals[0][j] = data

        valsSeries[i] = vals
        #i+=1 - uncomment for progress bar

    print('-' * 25, '\nDONE!\n' + '-' * 25)
    return valsSeries

# plotValues() : plots 3 axis accelerometer values over defined duration
def plotValues(accelValues,duration,delay=0.1):

    tSeries = np.arange(0, duration, step=delay)
    xVals = []
    yVals = []
    zVals = []

    for readings in accelValues:
        xVals.append(readings[0])
        yVals.append(readings[1])
        zVals.append(readings[2])

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1)
    fig.suptitle('Accelerometer Measurements')
    ax1.plot(tSeries, xVals, 'r')
    ax1.set_title('X axis')
    ax1.set_ylabel('Acceleration')
    ax1.set_xlabel('Time (s)')
    ax2.plot(tSeries, yVals, 'g')
    ax2.set_title('Y axis')
    ax2.set_ylabel('Acceleration')
    ax2.set_xlabel('Time (s)')
    ax3.plot(tSeries, zVals, 'b')
    ax3.set_title('Z axis')
    ax3.set_ylabel('Acceleration')
    ax3.set_xlabel('Time (s)')
    plt.show()

    return None

# DEFINE PARAMETERS
Tdelay = 0.1 # this should match value in Arduino code
duration = 10 # desired duration of recording (sort of)
nVals = 3
nIterations = int(duration/Tdelay)

# establishes serial communication port - first for input 1, second for 2
#ser = serial.Serial('/dev/cu.usbmodem142301',baudrate=9600,timeout=1)
ser = serial.Serial('/dev/cu.usbmodem141301',baudrate=9600,timeout=1)

# allow time on start up to be extra cautious to avoid errors
time.sleep(1.5)

accelValsArray = collectData(duration,Tdelay,nVals)

plotValues(accelValsArray,duration,Tdelay)