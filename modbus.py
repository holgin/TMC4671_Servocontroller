#!/usr/bin/env python3
import binascii
import minimalmodbus
import serial
from struct import pack, unpack
import logging
import time


def InitializeModbusCOM(COMport):    
    instrument = minimalmodbus.Instrument(COMport, slaveaddress=1, debug=0)
    instrument.serial.baudrate = 115200         #
    instrument.serial.bytesize = 8
    instrument.serial.parity = serial.PARITY_NONE
    instrument.serial.stopbits = 1
    instrument.serial.timeout = 0.05          # seconds
    instrument.mode = minimalmodbus.MODE_RTU   # rtu or ascii mode
    instrument.clear_buffers_before_each_transaction = True

    return instrument

instrument = InitializeModbusCOM("COM16")

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
    "setTargetTorque": 5,
    "getTargetTorque": 6,
    "getActualTorque": 7,
    "setTargetVelocity": 8,
    "getTargetVelocity": 9,
    "getActualVelocity": 10,
    "setAbsoluteTargetPosition": 11,
    "incrementTargetPosition": 12,
    "decrementTargetPosition": 13,
    "getActualPosition": 14,
    "getActualTargetPosition": 15,
    "setDebugLedState": 16,
    "getDebugLedState": 17,
    "getHSTemp": 18,
    "getTemperatures": 19,
    "performEncoderInitUD": 20

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
#--------------------------------------------------------------------------------------
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
#--------------------------------------------------------------------------------------
def tmc4671_setTargetTorque(motor, targetTorque):
    frame = pack(
        '<BBI', CustomFunctionID["setTargetTorque"], motor, targetTorque)
    frame = "".join(map(chr, frame))
    result = instrument._perform_command(
        functionID["customFunction"], frame)
    result = bytes([ord(c) for c in result[1:]])
    result = list(unpack('<B', result))
    logging.info("setTargetTorque set: {}".format(targetTorque))
    logging.debug("setTargetTorque get: {}".format(result[0]))

    return result

def tmc4671_getTargetTorque(motor):
    frame = pack(
        '<BB', CustomFunctionID["getTargetTorque"], motor)
    frame = "".join(map(chr, frame))
    result = instrument._perform_command(functionID["customFunction"], frame)
    result = bytes([ord(c) for c in result[1:]])
    result = list(unpack('<i', result))
    logging.info("getTargetTorque: get {}".format(result[0]))

    return result

def tmc4671_getActualTorque(motor):
    frame = pack(
        '<BB', CustomFunctionID["getActualTorque"], motor)
    frame = "".join(map(chr, frame))
    result = instrument._perform_command(functionID["customFunction"], frame)
    result = bytes([ord(c) for c in result[1:]])
    result = list(unpack('<i', result))
    logging.info("getActualTorque: get {}".format(result[0]))

    return result
#--------------------------------------------------------------------------------------
def tmc4671_setTargetVelocity(motor, targetVelocity):
    frame = pack(
        '<bbi', CustomFunctionID["setTargetVelocity"], motor, targetVelocity)
    frame = "".join(map(chr, frame))
    result = instrument._perform_command(
        functionID["customFunction"], frame)
    result = bytes([ord(c) for c in result[1:]])
    result = list(unpack('<b', result))
    logging.info("setTargetVelocity set: {}".format(targetVelocity))
    logging.debug("setTargetVelocity get: {}".format(result[0]))

    return result

def tmc4671_getTargetVelocity(motor):
    frame = pack(
        '<BB', CustomFunctionID["getTargetVelocity"], motor)
    frame = "".join(map(chr, frame))
    result = instrument._perform_command(functionID["customFunction"], frame)
    result = bytes([ord(c) for c in result[1:]])
    result = list(unpack('<i', result))
    logging.info("getTargetVelocity: get {}".format(result[0]))

    return result

def tmc4671_getActualVelocity(motor):
    frame = pack(
        '<BB', CustomFunctionID["getActualVelocity"], motor)
    frame = "".join(map(chr, frame))
    result = instrument._perform_command(functionID["customFunction"], frame)
    result = bytes([ord(c) for c in result[1:]])
    result = list(unpack('<i', result))
    logging.info("getActualVelocity: get {}".format(result[0]))

    return result
#--------------------------------------------------------------------------------------
def tmc4671_setAbsoluteTargetPosition(motor, AbsoluteTargetPosition):
    frame = pack(
        '<BBI', CustomFunctionID["setAbsoluteTargetPosition"], motor, AbsoluteTargetPosition)
    frame = "".join(map(chr, frame))
    result = instrument._perform_command(
        functionID["customFunction"], frame)
    result = bytes([ord(c) for c in result[1:]])
    result = list(unpack('<b', result))
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
    result = list(unpack('<i', result))
    logging.info("getActualPosition: get {}".format(result[0]))

    return result

def tmc4671_gettmcTargetPosition(motor):
    frame = pack(
        '<BB', CustomFunctionID["getActualTargetPosition"], motor)
    frame = "".join(map(chr, frame))
    result = instrument._perform_command(functionID["customFunction"], frame)
    result = bytes([ord(c) for c in result[1:]])
    result = list(unpack('<i', result))
    logging.info("getActualTargetPosition: get {}".format(result[0]))

    return result
#--------------------------------------------------------------------------------------
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

def getHSTemp(sensor):
    frame = pack(
        '<BB', CustomFunctionID["getHSTemp"], sensor)
    frame = "".join(map(chr, frame))
    result = instrument._perform_command(functionID["customFunction"], frame)
    result = bytes([ord(c) for c in result[1:]])
    result = list(unpack('<i', result))
    logging.info("getHSTemp: get {}".format(result[0]))

    return result

def getTemperatures():
    frame = pack(
        '<B', CustomFunctionID["getTemperatures"])
    frame = "".join(map(chr, frame))
    result = instrument._perform_command(functionID["customFunction"], frame)
    result = bytes([ord(c) for c in result[1:]])

    adc = list(unpack('<iiiii', result))
    logging.info("getTemperatures: get {}".format(adc))

    return result

def tmc4671_performEncoderInitUD(motor, UD_EXT):
    frame = pack(
        '<BBI', CustomFunctionID["performEncoderInitUD"], motor, UD_EXT)
    frame = "".join(map(chr, frame))
    result = instrument._perform_command(
        functionID["customFunction"], frame)
    result = bytes([ord(c) for c in result[1:]])
    result = list(unpack('<B', result))
    logging.info("performEncoderInitUD set: {}".format(UD_EXT))
    logging.debug("performEncoderInitUD get: {}".format(result[0]))

    return result

logging.basicConfig(level=logging.INFO)
logging.disable()

#getChipInfo()

#print("Testing commands start")
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
#print("Testing commands end")
