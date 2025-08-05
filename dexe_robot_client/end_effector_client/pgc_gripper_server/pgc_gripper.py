# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import threading
import logging
from pgc_device import dh_device

logging.basicConfig(level=logging.DEBUG)


class dh_modbus_gripper:
    _instance = None
    _lock = threading.Lock()

    def __new__(cls, *args, **kwargs):
        with cls._lock:
            if not cls._instance:
                cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        r"""Initialize the gripper with a lock for thread safety."""
        self.device = dh_device()

    def CRC16(self, nData, wLength):
        r"""Calculate the CRC16 checksum for the given data.

        Args:
            nData (list): The data to calculate the checksum for.
            wLength (int): The length of the data.

        Returns:
            int: The calculated CRC16 checksum.
        """
        if nData==0x00:
            return 0x0000
        wCRCWord=0xFFFF
        poly=0xA001
        for num in range(wLength):
            date = nData[num]
            wCRCWord = (date & 0xFF)^ wCRCWord
            for bit in range(8) : 
                if(wCRCWord&0x01)!=0:
                    wCRCWord>>=1
                    wCRCWord^= poly
                else:
                    wCRCWord>>=1
        return wCRCWord

    def open(self, PortName, BaudRate):
        r"""Open the connection to the gripper device.

        Args:
            PortName (str): The name of the port to connect to.
            BaudRate (int): The baud rate for the connection.

        Returns:
            int: 0 if successful, negative value if failed.
        """
        ret = self.device.connect_device(PortName, BaudRate)
        if ret < 0:
            logging.error("Failed to open connection")
        else:
            logging.info("Connection opened successfully")
        return ret

    def close(self):
        r"""Close the connection to the gripper device."""
        self.device.disconnect_device()
        logging.info("Connection closed")

    def WriteRegisterFunc(self, index, value, gripper_id=0x01):
        r"""Write a value to a specific register of the gripper.

        Args:
            index (int): The register index to write to.
            value (int): The value to write to the register.

        Returns:
            bool: True if the write operation was successful, False otherwise.
        """
        send_buf = [0,0,0,0,0,0,0,0]
        send_buf[0] = gripper_id
        send_buf[1] = 0x06
        send_buf[2] = (index >> 8) & 0xFF
        send_buf[3] = index & 0xFF
        send_buf[4] = (value >> 8) & 0xFF
        send_buf[5] = value & 0xFF
        
        crc = self.CRC16(send_buf,len(send_buf)-2)
        send_buf[6] = crc & 0xFF
        send_buf[7] = (crc >> 8) & 0xFF

        send_temp = send_buf
        ret = False
        retrycount = 3

        while ( ret == False ):
            ret = False

            if(retrycount < 0) :
                break
            retrycount = retrycount - 1

            wdlen = self.device.device_write(send_temp)
            if(len(send_temp) != wdlen) :
                logging.error(f"Write error! Sent: {send_temp}")
                continue

            rev_buf = self.device.device_read(8)
            if rev_buf is None:
                ret = False
            elif(len(rev_buf) == wdlen) :
                ret = True
        logging.error("Failed to communicate with device after retries")
        return ret

    def ReadRegisterFunc(self, index, gripper_id=0x01):
        r"""Read a value from a specific register of the gripper.

        Args:
            index (int): The register index to read from.

        Returns:
            int: The value read from the register.
        """
        send_buf = [0,0,0,0,0,0,0,0]
        send_buf[0] = gripper_id
        send_buf[1] = 0x03
        send_buf[2] = (index >> 8) & 0xFF
        send_buf[3] = index & 0xFF
        send_buf[4] = 0x00
        send_buf[5] = 0x01

        crc = self.CRC16(send_buf,len(send_buf)-2)
        send_buf[6] = crc & 0xFF
        send_buf[7] = (crc >> 8) & 0xFF

        send_temp = send_buf
        ret = False
        retrycount = 3

        while ( ret == False ):
            ret = False

            if(retrycount < 0) :
                break
            retrycount = retrycount - 1

            wdlen = self.device.device_write(send_temp)
            if(len(send_temp) != wdlen) :
                logging.error(f"Write error! Sent: {send_temp}")
                continue

            rev_buf = self.device.device_read(7)
            if(len(rev_buf) == 7) :
                value = ((rev_buf[4]&0xFF)|(rev_buf[3] << 8))
                ret = True
        return value

    def Initialization(self, gripper_id=0x01):
        """Initialize the gripper by writing to the initialization register."""
        self.WriteRegisterFunc(0x0100, 0xA5, gripper_id)

    def SetTargetPosition(self, refpos, gripper_id=0x01):
        r"""Set the target position of the gripper.

        Args:
            refpos (int): The target position to set.
        """
        self.WriteRegisterFunc(0x0103, refpos, gripper_id)

    def SetTargetForce(self, force, gripper_id=0x01):
        r"""Set the target force of the gripper.

        Args:
            force (int): The target force to set.
        """
        self.WriteRegisterFunc(0x0101, force, gripper_id)

    def SetTargetSpeed(self, speed, gripper_id=0x01):
        r"""Set the target speed of the gripper.

        Args:
            speed (int): The target speed to set.
        """
        self.WriteRegisterFunc(0x0104, speed, gripper_id)

    def GetCurrentPosition(self, gripper_id=0x01):
        r"""Get the current position of the gripper.

        Returns:
            int: The current position of the gripper.
        """
        return self.ReadRegisterFunc(0x0202, gripper_id)

    def GetCurrentTargetForce(self, gripper_id=0x01):
        r"""Get the current target force of the gripper.

        Returns:
            int: The current target force of the gripper.
        """
        return self.ReadRegisterFunc(0x0101, gripper_id)

    def GetCurrentTargetSpeed(self, gripper_id=0x01):
        r"""Get the current target speed of the gripper.

        Returns:
            int: The current target speed of the gripper.
        """
        return self.ReadRegisterFunc(0x0104, gripper_id)

    def GetInitState(self, gripper_id=0x01):
        r"""Get the initialization state of the gripper.

        Returns:
            int: The initialization state of the gripper.
        """
        return self.ReadRegisterFunc(0x0200, gripper_id)

    def GetGripState(self, gripper_id=0x01):
        r"""Get the grip state of the gripper.

        Returns:
            int: The grip state of the gripper.
        """
        return self.ReadRegisterFunc(0x0201, gripper_id)
