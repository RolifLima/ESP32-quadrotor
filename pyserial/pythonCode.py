import numpy as np
import matplotlib.pyplot as plt
import serial
import time

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
while True:
    data = ser.read(1024).decode("utf-8")
    
    time.sleep(1e-3)
    print (data)
    # if time.time()-tic>0.002:
    #     if not yaw:
    #         cmd = "s" + str(int(speed)) + "y0"
    #     else:
    #         cmd = "s0y" + str(int(speed))
    #     ser.write(cmd.encode('utf-8'))#.encode("ascii"))
    #     speed += increment
    #
    #     if speed <= -100:
    #         speed = -100
    #         increment=0.01
    #         yaw=not yaw
    #     if speed >= 100:
    #         speed =100
    #         increment=-0.01
    #
    #     # print (cmd,"\t",increment)
    #     tic = time.time()

    # if np.any( data[:]==b'\n'):
    #     print ('\n')
    # else:
    #     print (data, end =' ')
