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

host_nanopi = '192.168.0.116'
port_nanopi = 5770
addr_nanopi = (host_nanopi, port_nanopi)

host_serv = '192.168.0.150'
port_serv = 5770
addr_serv = (host_serv, port_serv)

tcp_socket = socket(AF_INET, SOCK_STREAM)
tcp_socket.connect(addr_serv)

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

    while True:
        msg = m.recv_match(blocking=True)

        string_msg = str(msg)
        print(string_msg)
        data = str.encode(string_msg)
        tcp_socket.send(data)

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
    server = TCPServer(addr_nanopi, MyTCPHandler)
    server.serve_forever()


th = Thread(target=launch)
th.start()
master = mavutil.mavlink_connection(args.device, baud=args.baudrate)
wait_heartbeat(master)
show_messages(master)

