# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import serial
import threading
import logging

logging.basicConfig(level=logging.INFO)


class dh_device(object):
    def __init__(self):
        r"""Initialize the dh_device class with a threading lock."""
        self.lock = threading.Lock()
        self.serialPort = serial.Serial(timeout=1.0)

    def connect_device(self, portname, Baudrate):
        r"""Connect to the device via the specified serial port and baud rate.

        Args:
            portname (str): The name of the serial port to connect to.
            Baudrate (int): The baud rate for the serial communication.

        Returns:
            int: 0 if the connection is successful, -1 otherwise.
        """
        with self.lock:
            try:
                self.serialPort.port = portname
                self.serialPort.baudrate = Baudrate
                self.serialPort.bytesize = 8
                self.serialPort.parity = "N"
                self.serialPort.stopbits = 1
                self.serialPort.set_output_flow_control = False
                self.serialPort.set_input_flow_control = False

                self.serialPort.open()
                if self.serialPort.isOpen():
                    logging.info("Serial Open Success")
                    return 0
                else:
                    logging.error("Serial Open Error")
                    return -1
            except serial.SerialException as e:
                logging.error(f"Serial Open Exception: {e}")
                return -1

    def disconnect_device(self):
        r"""Disconnect the device by closing the serial port."""
        with self.lock:
            if self.serialPort.isOpen():
                self.serialPort.close()
                logging.info("Serial Port Closed")
            else:
                logging.warning("Serial Port Already Closed")

    async def device_write(self, write_data):
        r"""Write data to the device.

        Args:
            write_data (bytes): The data to write to the device.

        Returns:
            int: The number of bytes written if successful, 0 if there is an error, -1 if the serial port is not open.
        """
        with self.lock:
            if self.serialPort.isOpen():
                try:
                    write_length = self.serialPort.write(write_data)
                    if write_length == len(write_data):
                        return write_length
                    else:
                        logging.debug(f"Write error! send_buff: {write_data}")
                        return 0
                except serial.SerialTimeoutException as e:
                    logging.debug(f"Write Timeout Exception: {e}")
                    return 0
            else:
                logging.error("Serial Port Not Open")
                return -1

    async def device_read(self, wlen):
        r"""Read data from the device.

        Args:
            wlen (int): The number of bytes to read from the device.

        Returns:
            bytes: The data read from the device if successful, -1 if the serial port is not open.
        """
        with self.lock:
            if self.serialPort.isOpen():
                try:
                    responseData = self.serialPort.read(wlen)
                    return responseData
                except serial.SerialTimeoutException as e:
                    logging.error(f"Read Timeout Exception: {e}")
                    return -1
            else:
                logging.error("Serial Port Not Open")
                return -1
