import pickle
import pbconstants as pbc

class PetalcontrollerFunctions(object):
    def __init__(self):
        with open('temp_calibration.pkl','rb') as f:
            self.temp_calibration = pickle.load(f)
        self.bitsum = 0

    def posfid_temps_cal(self,value):
        """
        Convert FIPOS ADC readings to temperatures
        NOTE: Catch ValueError and return ADC+5000 for readings outside interpolation table.
        """
        val = self.byte_conv(value)
        try:
            retval = round(float(self.temp_calibration[val]), 2)
        except:
            retval = -(5000 + val)
        return retval

    def flip_byte_order(self, byte_array):
        # Flips the byte order (LSB-> MSB).
        reversed_byte_array = bytearray([byte for byte in reversed(byte_array)])
        return reversed_byte_array

    def byte_conv(self, value):
        return int.from_bytes(self.flip_byte_order(value), byteorder='big')

    def format_byte_str(self, value, zfill):
        # Format integer as a hexadecimal string with specified length.
        return str(hex(value).replace('0x', '')).zfill(zfill)

    def nint(self, value):
        # Round to nearest integer.
        return int(round(value))

    def clamp(self, value, vmin, vmax):
        # clamps value to be wihin the range [vmin,vmax]
        return max(min(vmax, value), vmin)

    def set_bit(self, value, bit):
        # Set specific bit in value.
        return value | (1<<bit)


    def format_table_rows(self, canid, xcode, motor, motor_steps, speed_mode, post_pause):
        """
        Formats move table and checksum commands for sending over CAN (via canhandler).

        INPUTS:
            canid: integer, CAN address (also sometimes called CAN ID)
            xcode: string, specifies how to process each row of move table (0,1,2)
                '0': command is executed immediately, i.e. not stored as part of move table on microcontrolelr
                '1': any command in movetable which is not the last one
                '2': last command, i.e end of move table
            motor: string name of motor 'theta' or 'phi'
            speed_mode: string speed mode 'cruise' or 'creep'
            motor_steps: positive integer
            post_pause: integer time to pause after current command (i.e. before next command is
                                                                     executed) in milliseconds
        RETURNS:
            move_command: tuple command formatted for sending to canhandler,
                            (int canid, int command number, str data)
            checksum_command: tuple command formatted for sending to canhandler,
                    (int canid, int command number, str data), only sent for last row of move table
        """
        xcode = str(xcode)

        select = 0
        if motor_steps < 0:
            motor_steps = abs(motor_steps)
            select = self.set_bit(select, 0)
      
        if speed_mode in ['cruise']:
            select = self.set_bit(select, 1)

        if motor in ['theta']:
            select = self.set_bit(select, 2)

        select = str(select)
        motor_steps = self.format_byte_str(motor_steps, 6)
        post_pause = self.format_byte_str(post_pause, 4)
        hexdata = str(xcode + select + motor_steps + post_pause)
        move_command = (canid, pbc.MOVE_ROW_CMD, hexdata)
        if xcode == '1':
            data = int(xcode + select, 16) + int(motor_steps, 16) + int(post_pause, 16) + 4
            self.bitsum += data
            checksum_command = None
        elif xcode == '2':
            data = int(xcode + select, 16) + int(motor_steps, 16) + int(post_pause, 16) + 4
            self.bitsum += data
            checksum_command = (canid, pbc.CHECKSUM_CMD, self.format_byte_str(self.bitsum, 8))
            self.bitsum = 0
        elif xcode == '0':
            checksum_command = None
        return move_command, checksum_command
