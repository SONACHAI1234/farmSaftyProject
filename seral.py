import serial
import time
import RPi.GPIO as GPIO
from threading import Thread, Lock
from curses import ascii

# Enable Serial Communication
ser = serial.Serial()
ser.port = "/dev/ttyUSB0"
ser.baudrate = 9600
ser.timeout = 1
channel = 26

print ("### setmode done ###")
GPIO.setmode(GPIO.BCM)
print ("### setGPIO as OUT done ###")
GPIO.setup(channel, GPIO.OUT)
def doRead(ser, lock):
    while True:

        lock.acquire()

        try:
            rcv = ser.readline().decode().strip('\n')
        except:
            pass
        else:
            while rcv != '':
                if(rcv.find('***FARM***BIRD***FARM***') != -1):
                    print("BATHI ON")
                    GPIO.output(channel, GPIO.HIGH)
                elif (rcv.find('***FARM***OFF***FARM***') != -1):
                    GPIO.output(channel, GPIO.LOW)
                    print("BATHI OFF")
                else:
                    print(rcv)
                rcv = ser.readline().decode().strip('\n').strip('\r')
        lock.release()
        time.sleep(.15)

ser.open()
ser_lock = Lock()


th = Thread(target=doRead, args=(ser, ser_lock))
th.daemon = True
th.start()


gotlock = ser_lock.acquire()

#ser.write(b'AT+CPMS="ME","SM","ME"\r')
ser.write(b'AT+CMGF=1\r')
ser.write(b'AT+CMGR=1\r')
ser.write(b'AT+CPMS="SM"\r')
ser.write(b'AT+CNMI=1,2,0,0,0\r')

ser_lock.release()
time.sleep(.15)


try:
    ser_lock.acquire()
except:
    time.sleep(.1)
else:
    ser.write(b'AT+CPIN?\r')
    ser_lock.release()
    time.sleep(.15)

while True:
    try:
        cmd = input()
    except:
        pass
    else:
        ser_lock.acquire()

        if '^z' in cmd:
            ser.write(bytes('{}\r'.format(ascii.ctrl('z')), 'utf-8'))
        else:
            ser.write(bytes('{}\r'.format(cmd), 'utf-8'))
        ser_lock.release()
        time.sleep(.15)
