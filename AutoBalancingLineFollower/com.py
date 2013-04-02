import serial,time
ser = serial.Serial('/dev/tty.usbmodem411', 9600)#open the serial port
time.sleep(0.5)#give some time to establish the connection
t = 0.25
dt = 0.005
i = 0
while i <1:#number of led cycles
        for num in range(255,-1,-1):#fade the led from high to low
                ser.write(chr(num))
                time.sleep(dt)               
##        for num in range(0,256):#fade the led low to high
##                ser.write(chr(num))
##                time.sleep(dt)
        i = i+1
#off = 0
#ser.write(chr(off))#turn the led off
ser.close()#close the serial port
