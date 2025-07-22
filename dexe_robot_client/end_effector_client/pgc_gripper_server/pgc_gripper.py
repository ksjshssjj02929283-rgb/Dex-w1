# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import threading
import logging
from pgc_device import dh_device

logging.basicConfig(level=logging.INFO)


class dh_modbus_gripper(object):
    def __init__(self):
        r"""Initialize the gripper with default ID and create a lock for thread safety."""
        self.gripper_ID = 0x01
        self._transaction_lock = threading.Lock()
        self.device = dh_device()

    def CRC16(self, nData, wLength):
        r"""Calculate the CRC16 checksum for the given data.

        Args:
            nData (list): The data to calculate the checksum for.
            wLength (int): The length of the data.

        Returns:
            int: The calculated CRC16 checksum.
        """
        if nData == 0x00:
            return 0x0000
        wCRCWord = 0xFFFF
        poly = 0xA001
        for num in range(wLength):
            date = nData[num]
            wCRCWord = (date & 0xFF) ^ wCRCWord
            for bit in range(8):
                if (wCRCWord & 0x01) != 0:
                    wCRCWord >>= 1
                    wCRCWord ^= poly
                else:
                    wCRCWord >>= 1
        return wCRCWord

    def open(self, PortName, BaudRate):
        r"""Open the connection to the gripper device.

        Args:
            PortName (str): The name of the port to connect to.
            BaudRate (int): The baud rate for the connection.

        Returns:
            int: 0 if successful, negative value if failed.
        """
        with self._transaction_lock:
            ret = self.device.connect_device(PortName, BaudRate)
            if ret < 0:
                logging.error("Failed to open connection")
            else:
                logging.info("Connection opened successfully")
            return ret

    def close(self):
        r"""Close the connection to the gripper device."""
        with self._transaction_lock:
            self.device.disconnect_device()
            logging.info("Connection closed")

    async def _send_command(self, send_buf, expected_length):
        r"""Send a command to the device and read the response.

        Args:
            send_buf (list): The command to send.
            expected_length (int): The expected length of the response.

        Returns:
            list: The response from the device.
        """
        send_temp = send_buf
        retrycount = 3

        while retrycount > 0:
            wdlen = await self.device.device_write(send_temp)
            if len(send_temp) != wdlen:
                logging.debug(f"Write error! Sent: {send_temp}")
                retrycount -= 1
                continue

            rev_buf = await self.device.device_read(expected_length)
            if len(rev_buf) == expected_length:
                return rev_buf

            retrycount -= 1

        logging.debug("Failed to communicate with device after retries")
        return None

    def _prepare_command(self, function_code, index, value=None, gripper_id=0x01):
        r"""Prepare a command buffer with CRC16 checksum.

        Args:
            function_code (int): The function code for the command.
            index (int): The register index.
            value (int, optional): The value to write (for write commands).

        Returns:
            list: The prepared command buffer.
        """
        send_buf = [gripper_id, function_code, (index >> 8) & 0xFF, index & 0xFF]
        if value is not None:
            send_buf.extend([(value >> 8) & 0xFF, value & 0xFF])
        else:
            send_buf.extend([0x00, 0x01])
        crc = self.CRC16(send_buf, len(send_buf))
        send_buf.extend([crc & 0xFF, (crc >> 8) & 0xFF])
        return send_buf

    async def WriteRegisterFunc(self, index, value, gripper_id=0x01):
        r"""Write a value to a specific register of the gripper.

        Args:
            index (int): The register index to write to.
            value (int): The value to write to the register.

        Returns:
            bool: True if the write operation was successful, False otherwise.
        """
        with self._transaction_lock:
            send_buf = self._prepare_command(0x06, index, value, gripper_id)
            response = await self._send_command(send_buf, 8)
            return response is not None

    async def ReadRegisterFunc(self, index, gripper_id=0x01):
        r"""Read a value from a specific register of the gripper.

        Args:
            index (int): The register index to read from.

        Returns:
            int: The value read from the register.
        """
        with self._transaction_lock:
            send_buf = self._prepare_command(0x03, index, gripper_id)
            response = await self._send_command(send_buf, 7)
            if response:
                return (response[3] << 8) | (response[4] & 0xFF)
            return None

    async def Initialization(self, gripper_id=0x01):
        """Initialize the gripper by writing to the initialization register."""
        await self.WriteRegisterFunc(0x0100, 0xA5, gripper_id)

    async def SetTargetPosition(self, refpos, gripper_id=0x01):
        r"""Set the target position of the gripper.

        Args:
            refpos (int): The target position to set.
        """
        await self.WriteRegisterFunc(0x0103, refpos, gripper_id)

    async def SetTargetForce(self, force, gripper_id=0x01):
        r"""Set the target force of the gripper.

        Args:
            force (int): The target force to set.
        """
        await self.WriteRegisterFunc(0x0101, force, gripper_id)

    async def SetTargetSpeed(self, speed, gripper_id=0x01):
        r"""Set the target speed of the gripper.

        Args:
            speed (int): The target speed to set.
        """
        await self.WriteRegisterFunc(0x0104, speed, gripper_id)

    async def GetCurrentPosition(self, gripper_id=0x01):
        r"""Get the current position of the gripper.

        Returns:
            int: The current position of the gripper.
        """
        return await self.ReadRegisterFunc(0x0202, gripper_id)

    async def GetCurrentTargetForce(self, gripper_id=0x01):
        r"""Get the current target force of the gripper.

        Returns:
            int: The current target force of the gripper.
        """
        return await self.ReadRegisterFunc(0x0101, gripper_id)

    async def GetCurrentTargetSpeed(self, gripper_id=0x01):
        r"""Get the current target speed of the gripper.

        Returns:
            int: The current target speed of the gripper.
        """
        return await self.ReadRegisterFunc(0x0104, gripper_id)

    async def GetInitState(self, gripper_id=0x01):
        r"""Get the initialization state of the gripper.

        Returns:
            int: The initialization state of the gripper.
        """
        return await self.ReadRegisterFunc(0x0200, gripper_id)

    async def GetGripState(self, gripper_id=0x01):
        r"""Get the grip state of the gripper.

        Returns:
            int: The grip state of the gripper.
        """
        return await self.ReadRegisterFunc(0x0201, gripper_id)
