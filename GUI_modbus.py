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

COM_initialized = True

events=[]

window = tk.Tk()
window.title('TMC4671 servocontroller GUI')
window.geometry('550x600')
window.resizable(False, False)

#"global" variables for periodic update
tmcTargetVelocityReadout = tk.StringVar()
mbTargetVelocity = tk.StringVar()
ActualVelocityReadout = tk.StringVar()

tmcTargetPositionReadout = tk.StringVar()
ActualPositionReadout = tk.StringVar()
TargetPositionReadout = tk.StringVar()
deltaTargetPosition = tk.StringVar()

tmcTargetTorqueReadout = tk.StringVar()
mbTargetTorque = tk.StringVar()
ActualTorqueReadout = tk.StringVar()


#Create the emergency button
STOPbutton = tk.Button(text="DISABLE PWM", width=25, height=5, bg="RED", fg="BLACK")
STOPbutton.pack(fill=tk.X, padx=10, pady=5)

def handle_click(event):    
    #print("PWM disabled!")
    disablePWM(0)

STOPbutton.bind("<Button-1>", handle_click)

#demo notebook
notebook=ttk.Notebook(window)
notebook.pack(fill='both', pady=10, expand=True)

#(fill='both', expand=True)

#First frame - Init and Status
InitAndStatusFrame = ttk.Frame(notebook, width=500, height=600)
InitAndStatusFrame.pack(fill='both', expand=True)
notebook.add(InitAndStatusFrame, text='Init and Status')
#second frame - Velocity mode
VelocityModeFrame = ttk.Frame(notebook, width=501, height=600)
VelocityModeFrame.pack(fill='both', expand=True)
notebook.add(VelocityModeFrame, text='Velocity mode')
#third frame - Torque mode
TorqueModeFrame = ttk.Frame(notebook, width=501, height=600)
TorqueModeFrame.pack(fill='both', expand=True)
notebook.add(TorqueModeFrame, text='Torque mode')
#forth frame - Position mode
PositionModeFrame = ttk.Frame(notebook, width=501, height=600)
PositionModeFrame.pack(fill='both', expand=True)
notebook.add(PositionModeFrame, text='Position mode')


#-----------------------------------------------------------------
#Create a description (text) for COM port setup
#--------------ROW 0
tk.Label(InitAndStatusFrame, text="Please select a COM port used for MODBUS communication:").grid(row=0, column=0, columnspan=4, sticky="W", padx=2, pady=2)
#--------------ROW 1

# create a combobox - a Widget
selected_COM = tk.StringVar()
COMlist = ttk.Combobox(InitAndStatusFrame, textvariable=selected_COM)
COMlist['values'] = serial_ports_list
COMlist['state'] = 'readonly'
COMlist.grid(row=1, column=1, padx=2, pady=2)

#Create the connect button
Connectbutton = tk.Button(InitAndStatusFrame, text="Connect", width=20, height=1)
Connectbutton.grid(row=1, column=0, padx=2, pady=2)
def handle_click(event):    
    print("Modbus connected")
    #instrument = InitializeModbusCOM("COM11")    
Connectbutton.bind("<Button-1>", handle_click)

#Create the refresh button
RefreshCOMbutton = tk.Button(InitAndStatusFrame, text="Refresh", width=20, height=1)
RefreshCOMbutton.grid(row=1, column=2, padx=2, pady=2)
def handle_click(event):    
    serial_ports_list = serial_ports()
    #print(serial_ports_list)
    COMlist['values'] = serial_ports_list #update the list
RefreshCOMbutton.bind("<Button-1>", handle_click)

#--------------ROW 2 - 4
tk.Label(InitAndStatusFrame, text="Enter desired UD_EXT value and perform the initialisation.").grid(row=2, columnspan=3, sticky="W")
tk.Label(InitAndStatusFrame, text="Please remember that the motor should not be loaded and phase currents should be observed.").grid(row=3, columnspan=3, sticky="W")
tk.Label(InitAndStatusFrame, text="For high voltage motors, value of 500-1000 is recommended, for low voltage above 2000 up to 5000.").grid(row=4, columnspan=3, sticky="W")

#--------------ROW 5
#Create the Entry space to enter the Encoder Init UQ value
targetUD_EXT = tk.StringVar()
targetUD_EXT = ttk.Entry(InitAndStatusFrame)
targetUD_EXT.grid(row=5,column=1)
targetUD_EXT.insert(0,0)

InitEncoderUD = tk.Button(InitAndStatusFrame, text="Initialize Motor", width=20, height=1)
InitEncoderUD.grid(row=5, column=0, padx=2, pady=2)
def handle_click(event):
    #print("Starting initialization!")
    tmc4671_performEncoderInitUD(0, int(targetUD_EXT.get()))
InitEncoderUD.bind("<Button-1>", handle_click)

#-----------------------------------------------------------------
#Velocity Frame
#-----------------------------------------------------------------
#--------------ROW 0
tk.Label(VelocityModeFrame, text="Please input the desired speed in RPM:").grid(row=0, column=0, columnspan=4, sticky="W", padx=2, pady=2)

#--------------ROW 1
#Create the Entry space to enter the speed
mbTargetVelocity = ttk.Entry(VelocityModeFrame)
mbTargetVelocity.grid(row=1,column=2)
mbTargetVelocity.insert(0,0)

#Create the Send Velocity button
SendVelocitybutton = tk.Button(VelocityModeFrame, text="Send Velocity", width=20, height=1)
SendVelocitybutton.grid(row=1, column=0, padx=2, pady=2, columnspan=2)
def handle_click(event):
    Vel = int(mbTargetVelocity.get())
    #Velocity.delete(0, tk.END)
    #Velocity.insert(0,0)
    #print("Target velocity: ", Vel)
    tmc4671_setTargetVelocity(0, Vel)
SendVelocitybutton.bind("<Button-1>", handle_click)

#Create the RampDown button
RampDownbutton = tk.Button(VelocityModeFrame, text="Ramp Down", width=20, height=1)
RampDownbutton.grid(row=1, column=3, padx=2, pady=2, columnspan=2)
def handle_click(event):
    Vel = int(mbTargetVelocity.get())
    step = int(Vel/10)
    #print(step)
    for x in range(10):
        tmc4671_setTargetVelocity(0, Vel-x*step)
        #print(Vel-x*step)
        time.sleep(0.1)
    tmc4671_setTargetVelocity(0, 0)
    mbTargetVelocity.delete(0, tk.END)
    mbTargetVelocity.insert(0,0)
RampDownbutton.bind("<Button-1>", handle_click)

#--------------ROW 2
#Display actual speed target
tk.Label(VelocityModeFrame, text="Actual speed target: ").grid(row=2, column=0, padx=2, pady=2, columnspan=2)
tk.Label(VelocityModeFrame, textvariable=tmcTargetVelocityReadout, bg="white", width=10).grid(row=2, column=2, padx=2, pady=2)

#--------------ROW 3
#Display actual speed
tk.Label(VelocityModeFrame, text="Actual speed: ").grid(row=3, column=0, padx=2, pady=2, columnspan=2)
tk.Label(VelocityModeFrame, textvariable=ActualVelocityReadout, bg="white", width=10).grid(row=3, column=2, padx=2, pady=2)


#-----------------------------------------------------------------
#Torque Frame
#-----------------------------------------------------------------
   
#--------------ROW 0
#Create the Entry space to enter the Torque
tk.Label(TorqueModeFrame, text="Please input the desired Torque - RMS motor current - in mA:").grid(row=0, column=0, columnspan=4, sticky="W", padx=2, pady=2)

#--------------ROW 1
mbTargetTorque = ttk.Entry(TorqueModeFrame)
mbTargetTorque.grid(row=1,column=2)
mbTargetTorque.insert(0,0)
#Create the Send Target Torque button
SendmbTargetTorque = tk.Button(TorqueModeFrame, text="Send Target Torque", width=20, height=1)
SendmbTargetTorque.grid(row=1, column=0, padx=2, pady=2, columnspan=2)
def handle_click(event):
    if (int(mbTargetTorque.get()) > 32768):
        showinfo(title='Error', message="Value over the limit, please enter again")
        mbTargetTorque.delete(0, tk.END)
        mbTargetTorque.insert(0,0)
    else:
        tmc4671_setTargetTorque(0, int(mbTargetTorque.get()))
SendmbTargetTorque.bind("<Button-1>", handle_click)

#--------------ROW 2
#display actual Torque
tk.Label(TorqueModeFrame, text="Current Torque [mA]: ").grid(row=2, column=0, padx=2, pady=2, columnspan=2)
tk.Label(TorqueModeFrame, textvariable=ActualTorqueReadout, bg="white", width=10).grid(row=2, column=2, padx=2, pady=2)

#--------------ROW 3
#display TMC target Torque
tk.Label(TorqueModeFrame, text="Target Torque [mA]: ").grid(row=3, column=0, padx=2, pady=2, columnspan=2)
tk.Label(TorqueModeFrame, textvariable=tmcTargetTorqueReadout, bg="white", width=10).grid(row=3, column=2, padx=2, pady=2)


#-----------------------------------------------------------------
#Position Frame
#-----------------------------------------------------------------


#Create the Entry space to enter the target absolute position
#--------------ROW 0
tk.Label(PositionModeFrame, text="Please input the desired Absolute position:").grid(row=0, column=0, columnspan=3, sticky="W", padx=2, pady=2)

#--------------ROW 1
mbPositionTarget = ttk.Entry(PositionModeFrame, width=20)
mbPositionTarget.grid(row=1,column=2)
mbPositionTarget.insert(0,0)

#Create the Send Target Position button
SendmbPositionTarget = tk.Button(PositionModeFrame, text="Set Target Position:", width=20, height=1)
SendmbPositionTarget.grid(row=1, column=0, padx=2, pady=2, columnspan=2)
def handle_click(event):
    tmc4671_setAbsoluteTargetPosition(0, int(mbPositionTarget.get()))
SendmbPositionTarget.bind("<Button-1>", handle_click)


#--------------ROW 2
#display actual position
tk.Label(PositionModeFrame, text="Current Position: ").grid(row=2, column=0, padx=2, pady=2, columnspan=2)
tk.Label(PositionModeFrame, textvariable=ActualPositionReadout, bg="white", width=20).grid(row=2, column=2, padx=2, pady=2)

#--------------ROW 3
#display TMC target position
tk.Label(PositionModeFrame, text="Target Position: ").grid(row=3, column=0, padx=2, pady=2, columnspan=2)
tk.Label(PositionModeFrame, textvariable=tmcTargetPositionReadout, bg="white", width=20).grid(row=3, column=2, padx=2, pady=2)

#--------------ROW 4
#Create buttons and Entry for relative position control (Jog)
tk.Label(PositionModeFrame, text="Relative position control:").grid(row=4, column=0, columnspan=3, sticky="W", padx=2, pady=2)

#--------------ROW 5
deltaTargetPosition = ttk.Entry(PositionModeFrame)
deltaTargetPosition.grid(row=5, column=2, padx=2, pady=2)
deltaTargetPosition.insert(0,0)

SendmbPositionTarget = tk.Button(PositionModeFrame, text="->>", width=6, height=3)
SendmbPositionTarget.grid(row=5, column=1, padx=2, pady=2, sticky="W")
def handle_click(event):
    tmc4671_incrementTargetPosition(0, int(deltaTargetPosition.get()))
SendmbPositionTarget.bind("<Button-1>", handle_click)

SendmbPositionTarget = tk.Button(PositionModeFrame, text="<<-", width=6, height=3)
SendmbPositionTarget.grid(row=5, column=0, padx=2, pady=2, sticky="E")
def handle_click(event):
    tmc4671_decrementTargetPosition(0, int(deltaTargetPosition.get()))
SendmbPositionTarget.bind("<Button-1>", handle_click)


#-----------------------------------------------------------------
#Other
#-----------------------------------------------------------------
#global periodic update of data
def update_readout():
    if (COM_initialized == True): 
        tmcTargetVelocityReadout.set(tmc4671_getTargetVelocity(0)) #read Servocontroler's target speed, it may be different from GUI
        ActualVelocityReadout.set(tmc4671_getActualVelocity(0))
        ActualPositionReadout.set(tmc4671_getActualPosition(0))    
        tmcTargetPositionReadout.set(tmc4671_gettmcTargetPosition(0))
        ActualTorqueReadout.set(tmc4671_getActualTorque(0))
        tmcTargetTorqueReadout.set(tmc4671_getTargetTorque(0))    
        window.after(500, update_readout)

if (COM_initialized == True):    
    update_readout()

window.mainloop()


