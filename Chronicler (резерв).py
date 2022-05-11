#!/usr/bin/env python3

from __future__ import print_function
from builtins import range
from pymavlink import mavutil
import sys, datetime
import os
from threading import Thread
from argparse import ArgumentParser
from socketserver import *
import serial
from socket import *

host = '192.168.1.69'
port = 5770
addr = (host,port)

host2 = '192.168.1.68'
port2 = 5770
addr2 = (host2, port2)

tcp_socket = socket(AF_INET, SOCK_STREAM)
tcp_socket.connect(addr2)

parser = ArgumentParser(description=__doc__)
parser.add_argument("--baudrate", type=int, help="master port baud rate", default=115200)
parser.add_argument("--device", required=True, help="serial device")
parser.add_argument("--rate", default=1, type=int, help="requested stream rate")
args = parser.parse_args()

ser = serial.Serial('/dev/ttyACM0')

class MyTCPHandler(StreamRequestHandler):

    def handle(self):    
        self.data = self.request.recv(1024)
        print ('client send: '+str(self.data))
        ser.write(self.data)
        self.request.sendall(b'Hello from server!')

    def sending(self, messages):
        self.request.sendall(messages)

if not os.path.exists("LOGS"):
    os.mkdir("LOGS")

arming_flag = False

def wait_heartbeat(m):
    print("Waiting for APM heartbeat")
    m.wait_heartbeat()
    print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_system))

def show_messages(m):

    global arming_flag
    global tcp_socket
    i = 1

    while True:
        msg = m.recv_match(blocking=True)

        string_msg = str(msg)
        print(string_msg)
        data = str.encode(string_msg)
        tcp_socket.send(str.encode(str(i)))
        i += 1

        if not msg:
            return
        if msg.get_type() == "BAD_DATA":
            if mavutil.all_printable(msg.data):
                sys.stdout.write(msg.data)
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

            

server = None
def launch():   
    global server
    server = TCPServer(addr, MyTCPHandler)
    server.serve_forever()


th = Thread(target=launch)
th.start()

master = mavutil.mavlink_connection(args.device, baud=args.baudrate)
wait_heartbeat(master)

print("Sending all stream request for rate %u" % args.rate)
# for i in range(0, 3):
#     master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, args.rate, 1)


show_messages(master)


