#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk
from tkinter.messagebox import showinfo

import binascii
import minimalmodbus
import serial
from struct import pack, unpack
import logging
import time

from modbus import *
from COMport_list import*

#use the external function to check, load and print list of COM ports
serial_ports = serial_ports()
print(serial_ports)



events=[]

border_effects = {
    "flat": tk.FLAT,
    "sunken": tk.SUNKEN,
    "raised": tk.RAISED,
    "groove": tk.GROOVE,
    "ridge": tk.RIDGE,
}

window = tk.Tk()
#COMframe = tk.Tk()
window.title('TMC4671 servocontroller GUI')
window.geometry('1500x600')

#Create the emergency button
STOPbutton = tk.Button(text="DISABLE PWM", width=25, height=5, bg="RED", fg="BLACK")
STOPbutton.pack(fill=tk.X, padx=10, pady=5)

def handle_click(event):    
    print("PWM disabled!")
    disablePWM(0)

STOPbutton.bind("<Button-1>", handle_click)

#demo notebook
notebook=ttk.Notebook(window)
notebook.pack(pady=10, expand=True)

#First frame - config and connect
ConfigAndConnectFrame = ttk.Frame(notebook, width=1400, height=600)
ConfigAndConnectFrame.pack(fill='both', expand=True)
notebook.add(ConfigAndConnectFrame, text='Config and Connect')
#second frame - Velocity mode
VelocityModeFrame = ttk.Frame(notebook, width=1400, height=600)
VelocityModeFrame.pack(fill='both', expand=True)
notebook.add(VelocityModeFrame, text='Velocity mode')

#Create a description (text) for COM port setup
COMlabel = tk.Label(ConfigAndConnectFrame, text="Please select a COM port used for MODBUS communication:",
    #width=100
    #height=10
    )
COMlabel.pack(anchor=tk.W)

# create a combobox - a Widget
selected_COM = tk.StringVar()
COMlist = ttk.Combobox(ConfigAndConnectFrame, textvariable=selected_COM)
COMlist['values'] = serial_ports
COMlist['state'] = 'readonly'
COMlist.pack(anchor=tk.NW, padx=5, pady=5)

def COM_changed(event):
    """ handle the COM port changed event """
    showinfo(title='Result', message=f'You selected {selected_COM.get()}!')
COMlist.bind('<<ComboboxSelected>>', COM_changed)

#Create the connect button
Connectbutton = tk.Button(ConfigAndConnectFrame, text="Connect", width=10, height=1)
Connectbutton.pack(anchor=tk.NW, padx=5, pady=5)
def handle_click(event):    
    print("Modbus connected")
    #to be defined
Connectbutton.bind("<Button-1>", handle_click)

window.mainloop()
