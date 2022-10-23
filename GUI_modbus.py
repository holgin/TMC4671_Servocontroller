#!/usr/bin/env python3

import tkinter as tk
import binascii
import minimalmodbus
import serial
from struct import pack, unpack
import logging
import time
from modbus import *


events=[]

window = tk.Tk()
Mainline = tk.Label(
    text="TMC4671 servocontroller GUI",
    #foreground="white",  # Set the text color to white
    #background="black",  # Set the background color to black
    #width=10,
    #height=10
    )
Mainline.pack()

button = tk.Button(
    text="DISABLE PWM",
    width=25,
    height=5,
    bg="RED",
    fg="BLACK",
)
button.pack()

def handle_click(event):    
    print("The button was clicked!")
    disablePWM(0)

button.bind("<Button-1>", handle_click)

window.mainloop()
