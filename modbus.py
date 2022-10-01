#!/usr/bin/env python3
import minimalmodbus
import serial
from struct import pack, unpack
import logging
# port name, slave address (in decimal)


## Read temperature (PV = ProcessValue) ##
# Registernumber, number of decimals
# temperature = instrument.read_register(289, 1)
# print(temperature)


# temperature1 = instrument.read_registers(289, 2, 4)
# print(temperature1)


# ## Change temperature setpoint (SP) ##
# NEW_TEMPERATURE = 95
# # Registernumber, value, number of decimals for storage

# instrument.write_register(24, NEW_TEMPERATURE, 1)
# WRITEINT = 0x02
# READINT = 0x01


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
    "getTargetVelocity": 4
}


def tmc4671_writeInt(motor, address, value):
    frame = str(pack(
        '<bbbI', CustomFunctionID["writeInt"], motor, address, value).decode("utf8"))

    result = instrument._perform_command(functionID["customFunction"], frame)
    result = bytes(result[1:], 'utf8')
    result = hex(unpack('<b', result)[0])
    logging.debug("writeInt: sent({}, {})".format(hex(address), hex(value)))
    logging.debug("writeInt: recv({})".format(result))

    return result


def tmc4671_readInt(motor, address):
    frame = str(pack('<bbI', CustomFunctionID["readInt"], motor,
                     address).decode("utf8"))

    result = instrument._perform_command(functionID["customFunction"], frame)
    result = bytes(result[1:], 'utf8')
    result = hex(unpack('<I', result)[0])
    logging.debug("readInt: sent({})".format(hex(address)))
    logging.debug("readInt: recv({})".format(result))

    return result


def getChipInfo():
    TMC4671_CHIPINFO_ADDR = 0x01
    TMC4671_CHIPINFO_DATA = 0x00
    tmc4671_writeInt(0x00, TMC4671_CHIPINFO_ADDR, 0x00000000)
    chipInfo = tmc4671_readInt(0x00, TMC4671_CHIPINFO_DATA)
    logging.info("getChipInfo: {}".format(chipInfo))


def tmc4671_setTargetVelocity(motor, targetVelocity):
    frame = str(pack(
        '<bbI', CustomFunctionID["setTargetVelocity"], motor, targetVelocity).decode("utf8"))

    result = instrument._perform_command(functionID["customFunction"], frame)
    result = bytes(result[1:], 'utf8')
    result = hex(unpack('<b', result)[0])
    logging.info("setTargetVelocity sent: {}".format(hex(targetVelocity)))
    logging.debug("setTargetVelocity recv: {}".format(result))

    return result


def tmc4671_getTargetVelocity(motor):
    frame = str(pack(
        '<bI', CustomFunctionID["getTargetVelocity"], motor).decode("utf8"))

    result = instrument._perform_command(functionID["customFunction"], frame)
    result = bytes(result[1:], 'utf8')
    result = hex(unpack('<I', result)[0])
    logging.info("getTargetVelocity: {}".format(result))

    return result


if __name__ == "__main__":
    instrument = minimalmodbus.Instrument('com4', slaveaddress=1, debug=0)
    instrument.serial.baudrate = 57600         #
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
