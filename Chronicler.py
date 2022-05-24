#!/usr/bin/env python3

from __future__ import print_function
from builtins import range
from pymavlink import mavutil
import sys, datetime
import os
from threading import Thread
from argparse import ArgumentParser
import serial
import select
import socket

host_nanopi = ''
port_nanopi = 5760
addr_nanopi = (host_nanopi, port_nanopi)

serverSocket = None
message = None
con = None
addr = None
string_msg = None
flag_connect = False

parser = ArgumentParser(description=__doc__)
parser.add_argument("--baudrate", type=int, help="master port baud rate", default=115200)
parser.add_argument("--device", required=True, help="serial device")
parser.add_argument("--rate", default=4, type=int, help="requested stream rate")
args = parser.parse_args()

ser = serial.Serial('/dev/ttyACM0')
print("Connected to ACM0")




if not os.path.exists("LOGS"):
    os.mkdir("LOGS")

arming_flag = False

def wait_heartbeat(m):
    print("Waiting for APM heartbeat")
    m.wait_heartbeat()
    print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_system))

def show_messages(m, string_msg):

    global arming_flag, message, arming_flag, con
    count = 0
    MessageString = ''
    while True:
        msg = m.recv_match(blocking=True)
        string_msg = str(msg)
        MessageString += string_msg
        count = count + 1
        if count == 20:
            con.send(str.encode(MessageString))
            MessageString = ''
            count = 0
        print(string_msg)
        
        if not msg:
            return
        if msg.get_type() == "BAD_DATA":
            if mavutil.all_printable(msg.data):
                sys.stdout.write(str(msg.data))
                sys.stdout.flush()
        else:    

            if string_msg.find("Arming motors") != -1 and not arming_flag:
                arming_flag = True
                file = open("LOGS/" + str(datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')) + ".txt", "w")

            if arming_flag:
                file.write(string_msg + "\n")

            if string_msg.find("Disarming motors") != -1 and arming_flag:
                arming_flag = False
                file.close()          



def launch():
    global serverSocket, message, con, addr, string_msg, flag_connect

    serverSocket = socket.socket()
    serverSocket.bind((host_nanopi, port_nanopi))
    serverSocket.listen(1)

    con, addr = serverSocket.accept()
    #print("DroneGui connected")
    flag_connect = True
    # con.send(str.encode("HELLO"))
    # serverSocket.send(str.encode("HELLO"))

    while True:
        global string_msg
        message = con.recv(1024)
        #con.send(str.encode("HELLO"))
        if message is not None:
            ser.write(message)
            

        if string_msg is not None:

            con.send(str.encode(string_msg))
            serverSocket.send(str.encode(string_msg))

    con.close()


th = Thread(target = launch)
th.start()

master = mavutil.mavlink_connection(args.device, baud=args.baudrate)
wait_heartbeat(master)

for i in range(0, 3):
    master.mav.request_data_stream_send(master.target_system, master.target_component,
                                        mavutil.mavlink.MAV_DATA_STREAM_ALL, args.rate, 1)

if flag_connect:
    show_messages(master, string_msg)