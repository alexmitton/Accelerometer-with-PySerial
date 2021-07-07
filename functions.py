# FILE CONTAINING FUNCTIONS REQUIRED TO RUN MAIN
import numpy as np

# Plotting accelerometer values
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

