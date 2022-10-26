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
serial_ports_list = serial_ports()
#print(serial_ports_list)

events=[]

window = tk.Tk()
#COMframe = tk.Tk()
window.title('TMC4671 servocontroller GUI')
window.geometry('512x600')

#"global" variables for periodic update
tmcTargetVelocityReadout = tk.StringVar()
mbVelocityTarget = tk.StringVar()
ActualVelocityReadout = tk.StringVar()

tmcTargetPositionReadout = tk.StringVar()
ActualPositionReadout = tk.StringVar()
TargetPositionReadout = tk.StringVar()
deltaTargetPosition = tk.StringVar()



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
ConfigAndConnectFrame = ttk.Frame(notebook, width=500, height=600)
ConfigAndConnectFrame.pack(fill='both', expand=True)
notebook.add(ConfigAndConnectFrame, text='Config and Connect')
#second frame - Velocity mode
VelocityModeFrame = ttk.Frame(notebook, width=500, height=600)
VelocityModeFrame.pack(fill='both', expand=True)
notebook.add(VelocityModeFrame, text='Velocity mode')
#third frame - Torque mode
TorqueModeFrame = ttk.Frame(notebook, width=1000, height=600)
TorqueModeFrame.pack(fill='both', expand=True)
notebook.add(TorqueModeFrame, text='Torque mode')
#forth frame - Position mode
PositionModeFrame = ttk.Frame(notebook, width=1000, height=600)
PositionModeFrame.pack(fill='both', expand=True)
notebook.add(PositionModeFrame, text='Position mode')


#-----------------------------------------------------------------
#Create a description (text) for COM port setup
COMlabel = tk.Label(ConfigAndConnectFrame, text="Please select a COM port used for MODBUS communication:",
    #width=100
    #height=10
    )
COMlabel.pack(anchor=tk.W)

# create a combobox - a Widget
selected_COM = tk.StringVar()
COMlist = ttk.Combobox(ConfigAndConnectFrame, textvariable=selected_COM)
COMlist['values'] = serial_ports_list
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

#Create the refresh button
RefreshCOMbutton = tk.Button(ConfigAndConnectFrame, text="Refresh", width=10, height=1)
RefreshCOMbutton.pack(anchor=tk.NW, padx=5, pady=5)
def handle_click(event):    
    serial_ports_list = serial_ports()
    print(serial_ports_list)
    COMlist['values'] = serial_ports_list #update the list
RefreshCOMbutton.bind("<Button-1>", handle_click)

#-----------------------------------------------------------------
#Velocity Frame
#-----------------------------------------------------------------

Velocitylabel = tk.Label(VelocityModeFrame, text="Please input the desired speed in RPM:")
Velocitylabel.pack(anchor=tk.W)

#TargetVelocity = tk.StringVar()
#Velocity = ttk.Entry(VelocityModeFrame, textvariable = TargetVelocity)
#Create the Entry space to enter the speed
mbVelocityTarget = ttk.Entry(VelocityModeFrame)
mbVelocityTarget.pack(anchor=tk.NW)
mbVelocityTarget.insert(0,0)

#Create the Send Velocity button
SendVelocitybutton = tk.Button(VelocityModeFrame, text="Send Velocity", width=20, height=1)
SendVelocitybutton.pack(anchor=tk.NW, padx=5, pady=5)
def handle_click(event):
    Vel = int(mbVelocityTarget.get())
    #Velocity.delete(0, tk.END)
    #Velocity.insert(0,0)
    print("Target velocity: ", Vel)
    tmc4671_setTargetVelocity(0, Vel)
SendVelocitybutton.bind("<Button-1>", handle_click)

#Create the RampDown button
RampDownbutton = tk.Button(VelocityModeFrame, text="Ramp Down", width=20, height=1)
RampDownbutton.pack(anchor=tk.NW, padx=5, pady=5)
def handle_click(event):
    Vel = int(mbVelocityTarget.get())
    step = int(Vel/10)
    #print(step)
    for x in range(10):
        tmc4671_setTargetVelocity(0, Vel-x*step)
        #print(Vel-x*step)
        time.sleep(0.1)
    tmc4671_setTargetVelocity(0, 0)
    mbVelocityTarget.delete(0, tk.END)
    mbVelocityTarget.insert(0,0)
RampDownbutton.bind("<Button-1>", handle_click)

#Display actual speed target
tk.Label(VelocityModeFrame, text="Actual speed target: ").pack(anchor=tk.W)
tk.Label(VelocityModeFrame, textvariable=tmcTargetVelocityReadout, bg="white", width=10).pack(anchor=tk.W)
#Display actual speed
tk.Label(VelocityModeFrame, text="Actual speed: ").pack(anchor=tk.W)
tk.Label(VelocityModeFrame, textvariable=ActualVelocityReadout, bg="white", width=10).pack(anchor=tk.W)


#-----------------------------------------------------------------
#Torque Frame
#-----------------------------------------------------------------
#WIP

#-----------------------------------------------------------------
#Position Frame
#-----------------------------------------------------------------
#Create the Entry space to enter the position

tk.Label(PositionModeFrame, text="Please input the desired Absolute position:").pack(anchor=tk.W)
mbPositionTarget = ttk.Entry(PositionModeFrame)
mbPositionTarget.pack(anchor=tk.NW)
mbPositionTarget.insert(0,0)

#Create the Send Target Position button
SendmbPositionTarget = tk.Button(PositionModeFrame, text="Send Target Position", width=20, height=1)
SendmbPositionTarget.pack(anchor=tk.NW, padx=5, pady=5)
def handle_click(event):
    tmc4671_setAbsoluteTargetPosition(0, int(mbPositionTarget.get()))
SendmbPositionTarget.bind("<Button-1>", handle_click)

#display actual position
tk.Label(PositionModeFrame, text="Current Position: ").pack(anchor=tk.W)
tk.Label(PositionModeFrame, textvariable=ActualPositionReadout, bg="white", width=10).pack(anchor=tk.W)

#display TMC target position
tk.Label(PositionModeFrame, text="Target Position: ").pack(anchor=tk.W)
tk.Label(PositionModeFrame, textvariable=tmcTargetPositionReadout, bg="white", width=10).pack(anchor=tk.W)

#Create buttons and Entry for relative position control (Jog)
tk.Label(PositionModeFrame, text="Relative position control:").pack(anchor=tk.W)

deltaTargetPosition = ttk.Entry(PositionModeFrame)
deltaTargetPosition.pack(anchor=tk.NW)
deltaTargetPosition.insert(0,0)

SendmbPositionTarget = tk.Button(PositionModeFrame, text="+", width=3, height=3)
SendmbPositionTarget.pack(anchor=tk.NW)
def handle_click(event):
    tmc4671_incrementTargetPosition(0, int(deltaTargetPosition.get()))
SendmbPositionTarget.bind("<Button-1>", handle_click)

SendmbPositionTarget = tk.Button(PositionModeFrame, text="-", width=3, height=3)
SendmbPositionTarget.pack(anchor=tk.NW)
def handle_click(event):
    tmc4671_decrementTargetPosition(0, int(deltaTargetPosition.get()))
SendmbPositionTarget.bind("<Button-1>", handle_click)


#-----------------------------------------------------------------
#Other
#-----------------------------------------------------------------
#global periodic update of data
def update_readout():
    tmcTargetVelocityReadout.set(tmc4671_getTargetVelocity(0)) #read Servocontroler's target speed, it may be different from GUI
    ActualVelocityReadout.set(tmc4671_getActualVelocity(0))
    ActualPositionReadout.set(tmc4671_getActualPosition(0))    
    tmcTargetPositionReadout.set(tmc4671_gettmcTargetPosition(0))
    window.after(500, update_readout)
    
update_readout()

window.mainloop()
