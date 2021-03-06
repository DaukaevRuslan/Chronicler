#!/usr/bin/env python3

import sys
import datetime
import os
import socket
import serial
from threading import Thread
import time
from pymavlink import mavutil


host = ''
port = 5760

serverSocket = socket.socket()
serverSocket.bind((host, port))
serverSocket.listen(1)

con, addr = serverSocket.accept()
print("GUI Drone connected")

master = mavutil.mavlink_connection('/dev/ttyACM0', 115200)

for i in range(0, 3):
    master.mav.request_data_stream_send(master.target_system, master.target_component,
                                        mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

master.wait_heartbeat()

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0)
print("Connected to ACM0")


if not os.path.exists("LOGS"):
    os.mkdir("LOGS")

arming_flag = False

def launch():
    global serverSocket, message, con, addr, ser

    while True:
        global string_msg
        message = con.recv(1024)
        #con.send(str.encode("HELLO"))
        if message is not None:
            ser.write(message)
            # print(message)
            

    con.close()

th = Thread(target = launch)
th.start()

while True:
  try:

    data_from_serial = ser.readline() # узнать размер строки 
    msg = master.recv_match(blocking=True)
    string_msg = str(msg)
    #if data_from_serial != '':
    #con.sendall(data_from_serial)
    #print("Данные из сериала: " + str(data_from_serial))
    print(string_msg)
    #time.sleep(0.1)

    # if msg.get_type() == "BAD_DATA":
    #   if mavutil.all_printable(msg.data):
    #     sys.stdout.write(str(msg.data))
    #     sys.stdout.flush()
    # else:    

    #   if string_msg.find("Arming motors") != -1 and not arming_flag:
    #     print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
    #     arming_flag = True
    #     file = open("LOGS/" + str(datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')) + ".txt", "w")

    #   if arming_flag:
    #     file.write(string_msg + "\n")

    #   if string_msg.find("Disarming motors") != -1 and arming_flag:
    #     arming_flag = False
    #     file.close()

  except KeyboardInterrupt:
    serverSocket.close()
    con.close()
    sys.exit(0)      