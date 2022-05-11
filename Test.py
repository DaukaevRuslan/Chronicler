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

host = '192.168.1.69'
port = 5770
addr = (host,port)

parser = ArgumentParser(description=__doc__)
parser.add_argument("--baudrate", type=int, help="master port baud rate", default=115200)
parser.add_argument("--device", required=True, help="serial device")
parser.add_argument("--rate", default=1, type=int, help="requested stream rate")
args = parser.parse_args()

ser = serial.Serial('/dev/ttyACM0')