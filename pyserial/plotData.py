import numpy as np
import matplotlib.pyplot as plt
import serial
import time
from scipy.fft import fft, fftfreq
from scipy.signal.windows import blackman
import re
def string_to_float(input_string):
    # Define a regular expression pattern to match floating-point numbers
    float_pattern = re.compile(r'[-+]?\d*\.\d+|\d+')

    # Find all matches in the input string
    matches = float_pattern.findall(input_string)

    # If there are matches, convert the first match to a float
    if matches and len(matches) == 6:
        return np.asarray([float (m) for m in matches])
    else:
        # Return None if no valid floating-point number is found
        return None
tic = time.time()
ser =  serial.Serial()
ser.baudrate = 115200
ser.port = '/dev/ttyUSB0'
# ser.port = "/dev/ttyACM0"
ser.open()
# data="\n"
# LifeTime = time.time()
#
# speed=0
# increment=0.01
# cmd= "s" + str(speed) + "y0"
# yaw = False
N = 60
# sample spacing
T = 1.0 / 800.0
w = blackman(N)
DATA = []
index = -1
data=0
while True:
    index+=1
    
    data = ser.readline().decode("utf-8")
    d = string_to_float(data)
    if d is not None:
        DATA.append(d)
    
    print (DATA)
    if len(DATA)>N:
        DATA.pop(0)
        _D = np.asarray(DATA)
        ax = _D[:,0]
        ay = _D[:,1]
        az = _D[:,2]
    
        # plt.semilogy(xf[1:N//2], 2.0/N * np.abs(yf[1:N//2]), '-b')

        # plt.semilogy(xf[1:N//2], 2.0/N * np.abs(ywf[1:N//2]), '-r')
        if index%10==0:
            plt.subplot(3,1,1)
            plt.cla()
            plt.plot(ax)
            plt.subplot(3,1,2)
            plt.cla()
            plt.plot(ay)
            plt.subplot(3,1,3)
            plt.cla()
            plt.plot(az)
            # plt.legend(['FFT', 'FFT w. window'])

            plt.grid()
            plt.pause(1e-6)

plt.show()