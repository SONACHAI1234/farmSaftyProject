import serial
import time
import os
import RPi.GPIO as GPIO
from threading import Thread, Lock
from curses import ascii
import subprocess
import pygame

def playFile1(file):
    global player
    player = subprocess.Popen(["omxplayer", file, "-ss", "30"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
def exitPlayer1():
    global player
    player.stdin.write("q")

def playFile(file):
    pygame.mixer.init()
    pygame.mixer.music.load(file)
    pygame.mixer.music.play()
    while pygame.mixer.music.get_busy() == True:
        continue


# Enable Serial Communication
ser = serial.Serial()
ser.port = "/dev/ttyUSB1"
ser.baudrate = 9600
ser.timeout = 1
channel = 26

print ("### setmode done ###")
GPIO.setmode(GPIO.BCM)
print ("### setGPIO as OUT done ###")
GPIO.setup(channel, GPIO.OUT)
def doRead(ser, lock):
    global pid
    while True:

        lock.acquire()

        try:
            rcv = ser.readline().decode().strip('\n')
        except:
            pass
        else:
            while rcv != '':
                if(rcv.find('***KISHANSTEEL***ON***KISHANSTEEL***') != -1):
                    print("BATHI ON")
                    pid = os.fork()
                    if (pid == 0) :
                        playFile1("file_example_MP3_700KB.mp3")
                    #GPIO.output(channel, GPIO.HIGH)
                elif (rcv.find('***KISHANSTEEL***OFF***KISHANSTEEL***') != -1):
                    #GPIO.output(channel, GPIO.LOW)
                    os.kill(pid,9)
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
