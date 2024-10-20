import socket
import struct
import pbconstants


# from collections import OrderedDict

class CANSocket(object):
    """    
    Class for communicating with the DESI fiber positioner control electronics
    through Systec module via SocketCAN.  Each CAN socket controls one CAN bus. 
    
    initialization input(s): 
    channel: string specifying the CAN bus name (e.g. 'can10')
    """

    can_frame_fmt = "=IB3x8s"

    def __init__(self, channel):
        try:
            self.channel = channel
            self.timeout = pbconstants.TIMEOUT
            self.buff_size = pbconstants.BUFF_SIZE
            self.s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.s.bind((channel,))
            self.s.settimeout(self.timeout)
        except socket.error:
            print('Error creating and/or binding socket for {}'.format(self.channel))
        return

    def __close__(self):
        self.s.close()

    def _build_can_frame(self, can_id, data):
        """
        Convenience function for CAN frame packing/unpacking (see `struct can_frame` in <linux/can.h>)
        """
        can_dlc = len(data)
        data = data.ljust(8, b'\x00')
        return struct.pack(self.can_frame_fmt, can_id, can_dlc, data)

    def _dissect_can_frame(self, frame):
        """
        Convenience function for CAN frame packing/unpacking (see `struct can_frame` in <linux/can.h>)
        """
        can_id, can_dlc, data = struct.unpack(self.can_frame_fmt, frame)
        can_id = can_id - 0x80000000
        can_id &= 0xEFFFFFFF
        return (can_id, can_dlc, data[:can_dlc])

    def _format_canid(self, canid, cmnd):
        """
       Format CAN id field and return in hex format (CAN identifier field contains both CAN id and firmware command number)
       """
        canid = pbconstants.EXT_ID_PFX + (hex(canid).replace('0x', '') + hex(cmnd).replace('0x', '').zfill(2)).zfill(7)
        return int(canid, 16)

    def send_command(self, canid, cmnd, data):
        """
        Sends a CAN command without waiting for a response.
        
        INPUTS:
            canid: integer specifying the CAN id 
            cmnd:  integer specifying the firmware command number
            data:  string specifiying the contents of the data field in the CAN frame
        RETURN:
            'SUCCESS' or 'FAILED' string
        """
        try:
            canid = self._format_canid(canid, cmnd)
            self.s.send(self._build_can_frame(canid, bytearray.fromhex(data)))
            return 'SUCCESS'
        except socket.error:
            print('Error sending CAN frame in send_command')
            return 'FAILED'

    def send(self, canid, cmnd, data):
        return self.send_command(canid, cmnd, data)

    def recv(self):
        """
        Receive responses from the devices (as many as detected on the canbus)
        until a timeout occurs

        RETURNS:
            recvd_data: dictionary containing returned data by CAN id
                        with  keys:  integer CAN ids
                            values:  byte arrays containing contents of CAN frame data fields
            or 'FAILED' if socket error
        """
        try:
            recved_data = {}
            while True:
                try:
                    cf, addr = self.s.recvfrom(self.buff_size)
                    can_id, can_dlc, data = self._dissect_can_frame(cf)
                    recved_data[can_id] = data
                except socket.timeout:
                    break
            return recved_data
        except socket.error:
            print('Error receiving CAN frame in recv')
            return 'FAILED'

    def send_command_recv(self, canid, cmnd, data):
        """
        Sends a CAN command and waits for a response.
        
        INPUTS:
            canid:  CAN ID (integer)
            cmnd:   firmware command number (integer)
            data:   contents of the data field in the CAN frame (string)
        RETURNS:
            rdata:   received data (byte array)
            or 'FAILED' if socket error
        """
        try:
            canid = self._format_canid(canid, cmnd)
            self.s.send(self._build_can_frame(canid, bytearray.fromhex(data)))
            cf, addr = self.s.recvfrom(self.buff_size)
            can_id, can_dlc, rdata = self._dissect_can_frame(cf)
            return rdata
        except socket.error:
            return 'FAILED'

    def send_recv(self, canid, cmnd, data):
        """
        Same method as send_command_recv but returns a dictionary to be compatible with
        the recv method.

        INPUTS:
            canid:  CAN ID (integer)
            cmnd:   firmware command number (integer)
            data:   contents of the data field in the CAN frame (string)
        RETURNS:
            rdata:   received data (byte array)
            or 'FAILED' if socket error
        """
        recved_data = {}
        try:
            canid = self._format_canid(canid, cmnd)
            self.s.send(self._build_can_frame(canid, bytearray.fromhex(data)))
            cf, addr = self.s.recvfrom(self.buff_size)
            can_id, can_dlc, rdata = self._dissect_can_frame(cf)
            recved_data[can_id] = data
            return recved_data
        except socket.error:
            return 'FAILED'
