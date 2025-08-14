# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import serial
import threading
import logging

logging.basicConfig(level=logging.DEBUG)


class dh_device:
    def __init__(self):
        r"""Initialize the dh_device class with a threading lock."""
        self._lock = threading.Lock()
        self.serialPort = serial.Serial(timeout=1.0)

    def connect_device(self, portname, Baudrate):
        r"""Connect to the device via the specified serial port and baud rate.

        Args:
            portname (str): The name of the serial port to connect to.
            Baudrate (int): The baud rate for the serial communication.

        Returns:
            int: 0 if the connection is successful, -1 otherwise.
        """
        with self._lock:
            try:
                self.serialPort.port = portname
                self.serialPort.baudrate = Baudrate
                self.serialPort.bytesize = 8
                self.serialPort.parity = 'N'
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
        with self._lock:
            if self.serialPort.isOpen():
                self.serialPort.close()
                logging.info("Serial Port Closed")
            else:
                logging.warning("Serial Port Already Closed")

    def device_write(self, write_data):
        r"""Write data to the device.

        Args:
            write_data (bytes): The data to write to the device.

        Returns:
            int: The number of bytes written if successful, 0 if there is an error, -1 if the serial port is not open.
        """
        with self._lock:
            if self.serialPort.isOpen():
                write_length = self.serialPort.write(write_data)
                if write_length == len(write_data):
                    return write_length
                else:
                    logging.error(f"Write error! send_buff: {write_data}")
                    return 0
            else:
                logging.error("Serial Port Not Open")
                return -1

    def device_read(self, wlen):
        r"""Read data from the device.

        Args:
            wlen (int): The number of bytes to read from the device.

        Returns:
            bytes: The data read from the device if successful, -1 if the serial port is not open.
        """
        with self._lock:
            if self.serialPort.isOpen():
                responseData = self.serialPort.read(wlen)
                if len(responseData) != wlen:
                    logging.warning(f"Read incomplete: expected {wlen}, got {len(responseData)}")
                    return None
                return responseData
            else:
                logging.error("Serial Port Not Open")
                return None
