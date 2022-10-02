#!/usr/bin/env python3
import minimalmodbus
import serial
from struct import pack, unpack
import logging
import time

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
    "readInt":  1,
    "writeInt": 2,
    "setTargetVelocity": 3,
    "getTargetVelocity": 4,
    "setDebugLedState": 5,
    "getDebugLedState": 6
}


def tmc4671_writeInt(motor, address, value):
    frame = str(pack(
        '<bbbI', CustomFunctionID["writeInt"], motor, address, value).decode("utf8"))

    result = instrument._perform_command(functionID["customFunction"], frame)
    result = bytes(result[1:], 'utf8')
    result = hex(unpack('<b', result)[0])
    logging.debug("writeInt: set({}, {})".format(hex(address), hex(value)))
    logging.debug("writeInt: get({})".format(result))

    return result


def tmc4671_readInt(motor, address):
    frame = str(pack('<bbI', CustomFunctionID["readInt"], motor,
                     address).decode("utf8"))

    result = instrument._perform_command(functionID["customFunction"], frame)
    result = bytes(result[1:], 'utf8')
    result = hex(unpack('<I', result)[0])
    logging.debug("readInt: set({})".format(hex(address)))
    logging.debug("readInt: get({})".format(result))

    return result


def getChipInfo():
    TMC4671_CHIPINFO_ADDR = 0x01
    TMC4671_CHIPINFO_DATA = 0x00
    tmc4671_writeInt(0x00, TMC4671_CHIPINFO_ADDR, 0x00000000)
    chipInfo = tmc4671_readInt(0x00, TMC4671_CHIPINFO_DATA)
    logging.info("getChipInfo: get{}".format(chipInfo))


def tmc4671_setTargetVelocity(motor, targetVelocity):
    frame = str(pack(
        '<bbI', CustomFunctionID["setTargetVelocity"], motor, targetVelocity).decode("utf8"))

    result = instrument._perform_command(functionID["customFunction"], frame)
    result = bytes(result[1:], 'utf8')
    result = hex(unpack('<b', result)[0])
    logging.info("setTargetVelocity set: {}".format(hex(targetVelocity)))
    logging.debug("setTargetVelocity get: {}".format(result))

    return result


def tmc4671_getTargetVelocity(motor):
    frame = str(pack(
        '<bI', CustomFunctionID["getTargetVelocity"], motor).decode("utf8"))

    result = instrument._perform_command(functionID["customFunction"], frame)
    result = bytes(result[1:], 'utf8')
    result = hex(unpack('<I', result)[0])
    logging.info("getTargetVelocity: get {}".format(result))

    return result

def setDebugLedState(state):
    frame = str(pack(
        '<bb', CustomFunctionID["setDebugLedState"], state).decode("utf8"))

    result = instrument._perform_command(functionID["customFunction"], frame)
    result = bytes(result[1:], 'utf8')
    result = hex(unpack('<b', result)[0])
    logging.info("setDebugLedState set: {}".format(hex(state)))
    logging.debug("setDebugLedState get: {}".format(result))

    return result


def getDebugLedState():
    frame = str(pack(
        '<b', CustomFunctionID["getDebugLedState"]).decode("utf8"))

    result = instrument._perform_command(functionID["customFunction"], frame)
    result = bytes(result[1:], 'utf8')
    result = hex(unpack('<b', result)[0])
    logging.info("getDebugLedState: get {}".format(result))

    return result


if __name__ == "__main__":
    instrument = minimalmodbus.Instrument('com4', slaveaddress=1, debug=0)
    instrument.serial.baudrate = 115200         #
    instrument.serial.bytesize = 8
    instrument.serial.parity = serial.PARITY_NONE
    instrument.serial.stopbits = 1
    instrument.serial.timeout = 0.05          # seconds
    instrument.mode = minimalmodbus.MODE_RTU   # rtu or ascii mode
    instrument.clear_buffers_before_each_transaction = True

    logging.basicConfig(level=logging.INFO)
    # logging.disable()
    getChipInfo()
    tmc4671_setTargetVelocity(0, 1)
    tmc4671_getTargetVelocity(0)
    tmc4671_setTargetVelocity(0, 2)
    tmc4671_getTargetVelocity(0)

    setDebugLedState(1)
    getDebugLedState()
    time.sleep(0.5)
    setDebugLedState(0)
    getDebugLedState()

