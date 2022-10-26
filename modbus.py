#!/usr/bin/env python3
import binascii
import minimalmodbus
import serial
from struct import pack, unpack
import logging
import time

COMport = 9

#def InitializeModbusCOM(COMport):    
instrument = minimalmodbus.Instrument('com'+str(COMport), slaveaddress=1, debug=0)
instrument.serial.baudrate = 115200         #
instrument.serial.bytesize = 8
instrument.serial.parity = serial.PARITY_NONE
instrument.serial.stopbits = 1
instrument.serial.timeout = 0.05          # seconds
instrument.mode = minimalmodbus.MODE_RTU   # rtu or ascii mode
instrument.clear_buffers_before_each_transaction = True

#    return

#InitializeModbusCOM(9)

functionID = {
    "Read_Coils": 1,
    "Read_Discrete_Inputs": 2,
    "Read_Holding_Registers": 3,
    "Read_Input_Registers": 4,
    "Write_Single_Coil": 5,
    "Write_Single_Register": 6,
    "Write_Multiple_Registers": 16,
    "customFunction": 0x64,
}

CustomFunctionID = {
    "readInt": 1,
    "writeInt": 2,
    "enablePWM": 3,
    "disablePWM": 4,
#    "motorInitUQ_Flux": 5,
#    "setTargetTorque": 6,
#    "getTargetTorque": 7,
    "setTargetVelocity": 5,
    "getTargetVelocity": 6,
    "getActualVelocity": 7,
    "setAbsoluteTargetPosition": 8,
    "incrementTargetPosition": 9,
    "decrementTargetPosition": 10,
    "getActualPosition": 11,
    "getActualTargetPosition": 12,
    "setDebugLedState": 13,
    "getDebugLedState": 14,
    "getRawADC2Measurements": 15

}


def tmc4671_writeInt(motor, address, value):
    frame = pack(
        '<BBBI', CustomFunctionID["writeInt"], motor, address, value)
    frame = "".join(map(chr, frame))
    result = instrument._perform_command(functionID["customFunction"], frame)
    result = bytes([ord(c) for c in result[1:]])
    result = list(unpack('<B', result))
    logging.debug("writeInt set: {}, {}".format(hex(address), hex(value)))
    logging.debug("writeInt get: {}".format(hex(result[0])))

    return result

def tmc4671_readInt(motor, address):
    frame = pack('<BBI', CustomFunctionID["readInt"], motor,
                 address)
    frame = "".join(map(chr, frame))
    result = instrument._perform_command(functionID["customFunction"], frame)
    result = bytes([ord(c) for c in result[1:]])
    result = list(unpack('<I', result))
    logging.debug("readInt set: {}".format(hex(address)))
    logging.debug("readInt get: {} ".format(hex(result[0])))

    return result

def getChipInfo():
    TMC4671_CHIPINFO_ADDR = 0x01
    TMC4671_CHIPINFO_DATA = 0x00
    tmc4671_writeInt(0x00, TMC4671_CHIPINFO_ADDR, 0x00000000)
    chipInfo = tmc4671_readInt(0x00, TMC4671_CHIPINFO_DATA)
    logging.info("getChipInfo get: {}".format(hex(chipInfo[0])))

def enablePWM(state):
    frame = pack(
        '<BB', CustomFunctionID["enablePWM"], state)
    frame = "".join(map(chr, frame))
    result = instrument._perform_command(functionID["customFunction"], frame)
    result = bytes([ord(c) for c in result[1:]])
    result = list(unpack('<B', result))
    logging.info("enablePWM set: {}".format(state))
    logging.debug("enablePWM get: {}".format(result[0]))

    return result

def disablePWM(state):
    frame = pack(
        '<BB', CustomFunctionID["disablePWM"], state)
    frame = "".join(map(chr, frame))
    result = instrument._perform_command(functionID["customFunction"], frame)
    result = bytes([ord(c) for c in result[1:]])
    result = list(unpack('<B', result))
    logging.info("disablePWM set: {}".format(state))
    logging.debug("disablePWM get: {}".format(result[0]))

    return result 

def tmc4671_setTargetVelocity(motor, targetVelocity):
    frame = pack(
        '<BBI', CustomFunctionID["setTargetVelocity"], motor, targetVelocity)
    frame = "".join(map(chr, frame))
    result = instrument._perform_command(
        functionID["customFunction"], frame)
    result = bytes([ord(c) for c in result[1:]])
    result = list(unpack('<B', result))
    logging.info("setTargetVelocity set: {}".format(targetVelocity))
    logging.debug("setTargetVelocity get: {}".format(result[0]))

    return result

def tmc4671_getTargetVelocity(motor):
    frame = pack(
        '<BB', CustomFunctionID["getTargetVelocity"], motor)
    frame = "".join(map(chr, frame))
    result = instrument._perform_command(functionID["customFunction"], frame)
    result = bytes([ord(c) for c in result[1:]])
    result = list(unpack('<I', result))
    logging.info("getTargetVelocity: get {}".format(result[0]))

    return result

def tmc4671_getActualVelocity(motor):
    frame = pack(
        '<BB', CustomFunctionID["getActualVelocity"], motor)
    frame = "".join(map(chr, frame))
    result = instrument._perform_command(functionID["customFunction"], frame)
    result = bytes([ord(c) for c in result[1:]])
    result = list(unpack('<I', result))
    logging.info("getActualVelocity: get {}".format(result[0]))

    return result

def tmc4671_setAbsoluteTargetPosition(motor, AbsoluteTargetPosition):
    frame = pack(
        '<BBI', CustomFunctionID["setAbsoluteTargetPosition"], motor, AbsoluteTargetPosition)
    frame = "".join(map(chr, frame))
    result = instrument._perform_command(
        functionID["customFunction"], frame)
    result = bytes([ord(c) for c in result[1:]])
    result = list(unpack('<B', result))
    logging.info("setAbsoluteTargetPosition set: {}".format(AbsoluteTargetPosition))
    logging.debug("setAbsoluteTargetPosition get: {}".format(result[0]))

    return result

def tmc4671_incrementTargetPosition(motor, PositionIncrement):
    frame = pack(
        '<BBI', CustomFunctionID["incrementTargetPosition"], motor, PositionIncrement)
    frame = "".join(map(chr, frame))
    result = instrument._perform_command(
        functionID["customFunction"], frame)
    result = bytes([ord(c) for c in result[1:]])
    result = list(unpack('<B', result))
    logging.info("incrementTargerPosition set: {}".format(PositionIncrement))
    logging.debug("incrementTargerPosition get: {}".format(result[0]))

    return result

def tmc4671_decrementTargetPosition(motor, PositionDecrement):
    frame = pack(
        '<BBI', CustomFunctionID["decrementTargetPosition"], motor, PositionDecrement)
    frame = "".join(map(chr, frame))
    result = instrument._perform_command(
        functionID["customFunction"], frame)
    result = bytes([ord(c) for c in result[1:]])
    result = list(unpack('<B', result))
    logging.info("decrementTargerPosition set: {}".format(PositionDecrement))
    logging.debug("decrementTargerPosition get: {}".format(result[0]))

    return result

def tmc4671_getActualPosition(motor):
    frame = pack(
        '<BB', CustomFunctionID["getActualPosition"], motor)
    frame = "".join(map(chr, frame))
    result = instrument._perform_command(functionID["customFunction"], frame)
    result = bytes([ord(c) for c in result[1:]])
    result = list(unpack('<I', result))
    logging.info("getActualPosition: get {}".format(result[0]))

    return result

def tmc4671_gettmcTargetPosition(motor):
    frame = pack(
        '<BB', CustomFunctionID["getActualTargetPosition"], motor)
    frame = "".join(map(chr, frame))
    result = instrument._perform_command(functionID["customFunction"], frame)
    result = bytes([ord(c) for c in result[1:]])
    result = list(unpack('<I', result))
    logging.info("getActualTargetPosition: get {}".format(result[0]))

    return result

def setDebugLedState(state):
    frame = pack(
        '<BB', CustomFunctionID["setDebugLedState"], state)
    frame = "".join(map(chr, frame))
    result = instrument._perform_command(functionID["customFunction"], frame)
    result = bytes([ord(c) for c in result[1:]])
    result = list(unpack('<B', result))
    logging.info("setDebugLedState set: {}".format(state))
    logging.debug("setDebugLedState get: {}".format(result[0]))

    return result


def getDebugLedState():
    frame = pack(
        '<B', CustomFunctionID["getDebugLedState"])
    frame = "".join(map(chr, frame))
    result = instrument._perform_command(functionID["customFunction"], frame)
    result = bytes([ord(c) for c in result[1:]])
    result = list(unpack('<B', result))
    logging.info("getDebugLedState: get {}".format(result[0]))

    return result


def getRawADC2Measurements():
    frame = pack(
        '<B', CustomFunctionID["getRawADC2Measurements"])
    frame = "".join(map(chr, frame))
    result = instrument._perform_command(functionID["customFunction"], frame)
    result = bytes([ord(c) for c in result[1:]])

    adc = list(unpack('<IIIII', result))
    logging.info("getRawADC2Measurements: get {}".format(adc))

    return result


logging.basicConfig(level=logging.INFO)
logging.disable()

#getChipInfo()

print("Testing commands start")
#Testing velocity commands
#tmc4671_setTargetVelocity(0, 2000)
#time.sleep(0.5)
#tmc4671_setTargetVelocity(0, 0)

#Testing position commands
#tmc4671_setAbsoluteTargetPosition(0, 12500)
#time.sleep(0.4)
#tmc4671_incrementTargetPosition(0, 5000)
#time.sleep(0.4)
#tmc4671_decrementTargetPosition(0, 7000)
#time.sleep(0.4)
#tmc4671_incrementTargetPosition(0, 5000)
#time.sleep(0.4)
#tmc4671_setAbsoluteTargetPosition(0, 12500)

#Testing misc commands
#enablePWM(0)
#time.sleep(3)
#disablePWM(0)
#time.sleep(3)
#enablePWM(0)
#time.sleep(3)
#setDebugLedState(1)
#getDebugLedState()
#time.sleep(0.1)
#setDebugLedState(0)
#getDebugLedState()
print("Testing commands end")
