#!/usr/bin/env python3

import sys
import datetime
import os
import socket
import serial
from threading import Thread
import time


host = ''
port = 5760

serverSocket = socket.socket()
serverSocket.bind((host, port))
serverSocket.listen(1)

con, addr = serverSocket.accept()
print("GUI Drone connected")

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0)
print("Connected to ACM0")

def launch():
    global serverSocket, message, con, addr, ser

    while True:
        global string_msg
        message = con.recv(1024)
        #con.send(str.encode("HELLO"))
        if message is not None:
            ser.write(message)
            print(message)
            

    con.close()

th = Thread(target = launch)
th.start()

while True:
  try:
    
    data_from_serial = ser.readline()
    if data_from_serial != '':
      con.sendall(data_from_serial)
      print("Данные из сериала: " + str(data_from_serial))
      time.sleep(0.1)

  except KeyboardInterrupt:
    serverSocket.close()
    con.close()
    sys.exit(0)