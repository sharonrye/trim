#!/usr/bin/env python3
# pylint: disable=line-too-long, W0703, C0301, C0302, C0103, C0116, R0913, W0702
# fiposcontrol version for BBB (robot control only; no one-wire, no relay control)
#
# can handle multiple can buses
#
# version history
# 1.01  Nov 16, 2022    ms  fixed current setting
# 1.0   Oct 12, 2022    ms  version for FW 6.20 and up
# 0.8   Sep 30, 2022    ms  fixed speed setting; note that command 29 changed after FW 6.04; this has not yet
#                           been implemented
# 0.7.1 May 18, 2022    ms  relay functions now return a value
# 0.7   Apr 29, 2022    ms  added buil_can_frames
# 0.6   Apr 07, 2022    MS  added vibramove
# 0.5   Apr 06, 2022    MS  canhandler now receives a list; added more commands
# 0.4   Mar 24, 2022    MS  fixed a few typos, added relay_on and relay_off
# 0.3   Mar 22, 2022    MS  added delay in send_receive (canhandler)
# 0.2   Mar 19, 2022    MS  added prototype functions for all missing commands;
#                           fixed fiducial setting
# 0.1   Feb 27, 2022    MS
#
#
#
# The FiposControl class enables communication with the fipos FW through Python
#
# Example of usage:
#
# >>> import fiposcontrol; fipos = fiposcontrol.FiposControl(['can0'])
#
#
# >>> fipos.get_bl_version(None,'c')
# (True, 15, {'can30': {1288: '3.8'}})
# >>> fipos.get_bl_version(None,'raw')
# (True, 15, {'can30': {1288: b'\x08\x03'}})
#
import time
import math
import pickle
import numpy as np
#import trenz_onewire as pc_onewire
from intelhex import IntelHex
#import pbconstants as pbc
import canhandler_no_thread as canhandler
import pcfunctions
#import relaycontrol
#import trenz_io

VERSION = '0.8'

SYNC_PULSE_WIDTH = 0.1
BRDCAST_ALL = 20000 # 0x4E20
BRDCAST_POS = 20001 # 0x4E21
BRDCAST_FID = 20002 # 0x4E22
GEAR_RATIO = (46.0 / 14.0 + 1)**4

def clamp(value, min_val, max_val):
    return max(min(max_val, value), min_val)

def nint(value):
    # Round to nearest integer.
    return int(round(value))

def get_bytes(response, byte_range):
    return response[byte_range[0]:byte_range[1]]

def set_bit(value, bit):
    # Set specific bit in value.
    return value | (1<<bit)

def flip_byte_order(byte_array):
    #  Flips the byte order (LSB-> MSB).
    return bytearray([byte for byte in reversed(byte_array)])

def format_byte_str(value, fill):
    # Format integer as a hexadecimal string with specified length.
    # a nibble has length 1 and a byte length 2
    return str(hex(value).replace('0x', '')).zfill(fill)

def imons_conversion(imon):
    """
    Converts positioner/fiducial current monitor ADC readings to float milli-Amps.
    """
    imon_1 = int.from_bytes(flip_byte_order(get_bytes(imon, (0, 4))), byteorder='big')
    imon_2 = int.from_bytes(flip_byte_order(get_bytes(imon, (4, 8))), byteorder='big')
    adc_vals = (imon_1, imon_2)
    return tuple([round((3300/409600)*adc_val/0.13, 2) for adc_val in adc_vals])

def generic_conversion(raw):
    return int.from_bytes(raw, 'little')

class FiposControl():

    def __init__(self, busids, debug_level=0, verbose=False):
        # busids:  list of can buses
        #
        self.debug_level = debug_level
        self.verbose = verbose
        self.nchan = None
        self.vib_factor = None
        if isinstance(busids, list):
            self.busids = busids
            self.nchan = len(busids)
        #else:
        #    return 'FAILED: invalid canbus id format (expected list of strings) '
        #self.canh = canhandler.CANHandler(busids, print, debug_level=0)
        with open('temp_calibration.pkl', 'rb') as fp:
            self.posfid_temp_cal = pickle.load(fp)
        self.pcf = pcfunctions.PetalcontrollerFunctions()
        #self.pt = trenz_io.PtlIO(info_func=print, debug_level=3)
        #self.rc = relaycontrol.RelayControl(self.pt, print)
        #self.pt.switch('6V5_PWR_EN', 1)
        #self.pt.switch('5V25_PWR_EN', 1)
        if self.debug_level:
            print(busids)
        self.canh = canhandler.CANHandler(busids) #, print, debug_level=0)
        #self._pcow = pc_onewire.PC_OneWire(verbose=True)
        #self._pcow.restart_ow_thread()
        self.version = VERSION

    #def __del__(self):
    #    self._pcow.stop_ow_thread()

    #def gentle_exit(self):
    #    try:
    #        self._pcow.stop_ow_thread()
    #        return 'SUCCESS'
    #    except:
    #        return 'FAIL'



    #def set_pospwr(self, state, select=None):
    #    # state is either 'on', 'off, 0, 1'
    #    # select = None: both units, select = 1 or 2
    #    if state not in ['on', 'off', 0, 1]:
    #        return 'FAIL: invalid state'
    #    if select not in [None, 1, 2]:
    #        return 'FAIL: invalid state'
    #    if state in ['on', 1]:
    #        state = 1
    #    else:
    #        state = 0
    #    if not select:
    #        to_switch = [1, 2]
    #    else:
    #        to_switch = [select]
    #    try:
    #        for unit in to_switch:
    #            self.pt.switch('PS'+str(unit)+'_EN', state)
    #        return 'SUCCESS'
    #    except:
    #        return 'FAIL'

    #def get_pospwr(self):
    #    pospwr = {'PS1_FBK':'unknown', 'PS2_FBK':'unknown'}
    #    try:
    #        fbk_status = self.pt.read_HRPG600()
    #        if isinstance(fbk_status, dict):
    #            pospwr['PS1_FBK'] = 'on' if fbk_status['PS1_OK'] else 'off'
    #            pospwr['PS2_FBK'] = 'on' if fbk_status['PS2_OK'] else 'off'
    #            retval = pospwr
    #        else:
    #            retval = 'FAIL'
    #    except:
    #        retval = 'FAIL'
    #    return retval

    def read_bus(self):
        return self.canh.recv()

    def _generic_read_command(self, conversion, command, dframe, return_type='raw', delay=0.):
        try:
            raw_response = self.canh.send_recv_locked(dframe, delay)
            if self.debug_level:
                print('raw_response', raw_response)
            if return_type in ['raw']:
                return (True, command, raw_response)
            #print('return type',return_type)
            formatted_response = {busid:{} for busid in self.busids}
            #print(formatted_response)
            for bus_id, bus_data in raw_response.items():
                for can_id, item in bus_data.items():
                    if self.debug_level:
                        print(bus_id, bus_data, can_id, item)
                    formatted_response[bus_id][can_id] = conversion(item)
            return (True, command, formatted_response)
        except:
            return (False, command, 'Error')

    def make_dframe(self, command=None, devices_by_bus=None, data=''):
        if devices_by_bus:
            dframe = self._make_dframe(command, devices_by_bus, data)
        else:
            dframe = self._make_dframe_broadcast(command, BRDCAST_ALL, data)
        return dframe

    def _make_dframe(self, command=None, devices_by_bus=None, data=''):
        dframe = {}
        if devices_by_bus:
            for bus, device_list in devices_by_bus.items():
                if bus not in dframe:
                    dframe[bus] = []
                for canid in device_list:
                    dframe[bus].append((canid, command, data))
        return dframe

    def _make_dframe_broadcast(self, command=None, broadcast=BRDCAST_ALL, data=''):
        dframe = {}
        for bus in self.busids:
            dframe[bus] = [(broadcast, command, data)]
        return dframe

    # +-------------------------------------------------------------------+
    # |                  here we start with the commands                  |
    # +-------------------------------------------------------------------+

    def stay_alive(self, devices_by_bus=None):
        '''
        Stop the 60 second count down to go into Standby Mode.
        REQUIRED for FW 5.14 and up
        '''
        command = 1
        dframe = self.make_dframe(command, devices_by_bus)
        print(dframe)
        return (True, command, self.canh.send(dframe))

    def stay_alive_r(self, devices_by_bus=None, return_type='c'):
        command = 1
        dframe = self.make_dframe(command, devices_by_bus)
        return self._generic_read_command(generic_conversion, command, dframe, return_type)

    def set_currents(self, devices_by_bus=None, new_currents=None):
        command = 2
        # defaults
        currents = {'theta':{}, 'phi':{}}
        currents['theta'] = {'spin_up':0, 'cruise':0, 'creep':0, 'spin_down':0}
        currents['phi'] = {'spin_up':0, 'cruise':0, 'creep':0, 'spin_down':0}

        if new_currents:
            for m in ['phi', 'theta']:
                if m in new_currents:
                    for p in ['spin_up', 'cruise', 'creep', 'spin_down']:
                        if p in new_currents[m]:
                            currents[m][p] = clamp(round(new_currents[m][p]), 0, 100)
        data = ''
        for m in ['phi', 'theta']:
            for p in ['spin_up','spin_down','cruise', 'creep']:
                data += format_byte_str(currents[m][p], 2)

        dframe = self.make_dframe(command, devices_by_bus, data)
        if self.debug_level:
            print('inside set_currents, dframe: '+str(dframe))
        try:
            return self.canh.send(dframe)
        except:
            return 'FAILED: could not set speeds'
        return 'SUCCESS'

    def get_currents(self, devices_by_bus=None):
        # retrieves current settings
        # set verbose flag to 3
        self.set_verbose_flag(devices_by_bus, 3)
        self.set_currents(devices_by_bus)
        retval = self.canh.recv()
        self.set_verbose_flag(devices_by_bus, 1)        
        return retval

    def set_spin_ramps(self, devices_by_bus=None, new_ramps=None):
        command = 3
        # 0 = phi; 1 = theta
        ramps = {'theta':{}, 'phi':{}}
        ramps['theta'] = {'cw':{'up':0, 'down':0}, 'ccw':{'up':0, 'down':0}}
        ramps['phi'] = {'cw':{'up':0, 'down':0}, 'ccw':{'up':0, 'down':0}}

        #ramps = {'theta_creep':2, 'phi_creep':2, 'spin':12}   # spin is for both, theta and phi
                                                                # and for up / down
        if new_ramps:
            for m in ['phi', 'theta']:
                if m in new_ramps:
                    for d in ['cw', 'ccw']:
                        if d in new_ramps[m]:
                            for s in ['up', 'down']:
                                if s in new_ramps[m][d]:
                                    ramps[m][d][s] = clamp(round(new_ramps[m][d][s]), 0, 255)
        data = (format_byte_str(ramps['phi']['cw']['up'], 2) + \
                format_byte_str(ramps['phi']['ccw']['up'], 2) + \
                format_byte_str(ramps['phi']['cw']['down'], 2) + \
                format_byte_str(ramps['phi']['ccw']['down'], 2) + \
                format_byte_str(ramps['theta']['cw']['up'], 2) + \
                format_byte_str(ramps['theta']['ccw']['up'], 2) + \
                format_byte_str(ramps['theta']['cw']['down'], 2) + \
                format_byte_str(ramps['theta']['ccw']['down'], 2))
        dframe = self.make_dframe(command, devices_by_bus, data)

        if self.debug_level:
            print('inside set_periods, dframe: ' + str(dframe))
        try:
            return self.canh.send(dframe)
        except:
            return 'FAILED: could not set creep periods'

    def get_spin_ramps(self, devices_by_bus=None):
        # retrieves current settings
        # set verbose flag to 3
        self.set_verbose_flag(devices_by_bus, 3)
        self.set_spin_ramps(devices_by_bus)
        retval = self.canh.recv()
        self.set_verbose_flag(devices_by_bus, 1)        
        return retval

    def set_cruise_creep_speed(self, devices_by_bus=None, new_steps=None):
        ''' Sets the step size for cruise and creep

        Arguments:
        devices_by_bus:
        new_steps: None of dictionary of the form
                   speed = {'theta':{'cw':{'cruise':33,'creep':1}, 'ccw':{'cruise':33,'creep':1}},
                            'phi':{'cw':{'cruise':33,'creep':1},'ccw':{'cruise':33,'creep':1}}}

        Phi motor = 0; Theta motor = 1

        If new_steps == None, values are set to the default values as shown.
            CW_Cruise_Step_0      : data[0]    (default = 33)
            CCW_Cruise_Step_0     : data[1]    (default = 33)
            CW_Creep_Step_Size_0  : data[2]    (default = 1)
            CCW_Creep_Step_Size_0 : data[3]    (default = 1)
            CW_Cruise_Step_1      : data[4]    (default = 33)
            CCW_Cruise_Step_1     : data[5]    (default = 33)
            CW_Creep_Step_Size_1  : data[6]    (default = 1)
            CCW_Creep_Step_Size_1 : data[7]    (default = 1)

        Takes integer values between 0 and 254

        The number specifies the number of tenths of a degree of motor rotation
        each time the TIMER interrupt is serviced.

        Cruise steps all default to 33; creep steps all default to 1

        Once set, the values remain after execution of Command 26 until
        they are changed by Command 5
        '''
        command = 5
        # default values
        steps = {'theta':{}, 'phi':{}}
        steps['theta'] = {'cw':{'cruise':0, 'creep':0}, 'ccw':{'cruise':0, 'creep':0}}
        steps['phi'] = {'cw':{'cruise':0, 'creep':0}, 'ccw':{'cruise':0, 'creep':0}}

        if new_steps:
            for m in ['theta', 'phi']:
                if m in new_steps:
                    for d in ['cw', 'ccw']:
                        if d in new_steps[m]:
                            for s in ['cruise', 'creep']:
                                if s in new_steps[m][d]:
                                    steps[m][d][s] = clamp(round(new_steps[m][d][s]), 0, 255)

        data = (format_byte_str(steps['phi']['cw']['cruise'], 2) + \
                format_byte_str(steps['phi']['ccw']['cruise'], 2) + \
                format_byte_str(steps['phi']['cw']['creep'], 2) + \
                format_byte_str(steps['phi']['ccw']['creep'], 2) + \
                format_byte_str(steps['theta']['cw']['cruise'], 2) + \
                format_byte_str(steps['theta']['ccw']['cruise'], 2) + \
                format_byte_str(steps['theta']['cw']['creep'], 2) + \
                format_byte_str(steps['theta']['ccw']['creep'], 2))

        dframe = self.make_dframe(command, devices_by_bus, data)
        try:
            return self.canh.send(dframe)
        except:
            error = 'FAILED: could not set speed'
            success = False
        return (success, command, error)



    def get_cruise_creep_speed(self, devices_by_bus=None):
        # retrieves current settings
        # set verbose flag to 3
        self.set_verbose_flag(devices_by_bus, 3)
        self.set_cruise_creep_speed(devices_by_bus)
        retval = self.canh.recv()
        self.set_verbose_flag(devices_by_bus, 1)        
        return retval


    def set_creep_periods(self, devices_by_bus=None, new_periods=None):

        ''' This command sets creep periods as follows:

        If no command is sent, the defaults are as shown.
            Creep_Period_0      : data[0]    (default = 2)
            Creep_Period_1      : data[1]    (default = 2)

        Takes integer values between 0 and 254

        The number specifies what is essentially a pre-scaler on the rate at
        which the motor is moved during a creep. I.e., if the period is set to 2,
        the motor is stepped only every other TIMER interrupt. If it is set to 7, 
        it steps only on every 7th interrupt. This was included in the original
        firmware to allow very slow creeps

        Both motors default to CreepPeriod == 2

        Once set, the values remain after execution of Command 26 until
        they are changed by Command 6

        '''
        command = 6
        # defaults
        periods = {'theta': 0, 'phi': 0}

        if new_periods:
            for m in ['phi', 'theta']:
                if m in new_periods:
                    periods[m] = clamp(round(new_periods[m]), 0, 254)

        data = format_byte_str(periods['phi'], 2) + format_byte_str(periods['theta'], 2)
        dframe = self.make_dframe(command, devices_by_bus, data)
        try:
            return self.canh.send(dframe)
        except:
            return 'FAILED: could not set periods'
        return 'SUCCESS'


    def get_creep_periods(self, devices_by_bus=None):
        # retrieves current settings
        # set verbose flag to 3
        self.set_verbose_flag(devices_by_bus, 3)
        self.set_creep_periods(devices_by_bus)
        retval = self.canh.recv()
        self.set_verbose_flag(devices_by_bus, 1)        
        return retval


    # Commands 4 - 8: not used

    def get_temperature(self, devices_by_bus=None, return_type='c'):

        ''' Reads out temperatures on fipos board

        Arguments:
        devices_by_bus:
        return_type:
        '''
        command = 9
        def _conversion(raw):
            return self.pcf.posfid_temps_cal(raw)

        dframe = self.make_dframe(command, devices_by_bus)
        if self.debug_level > 1:
            print('get_temperature dframe: '+str(dframe))
        (success, command, data) = self._generic_read_command(_conversion, command, dframe, return_type, delay=10.)
        return (success, command, data)

    def get_can_address(self, devices_by_bus=None, return_type='c'):

        ''' Reads CAN address

        Arguments:
        devices_by_bus:
        return_type:
        '''
        command = 10
        dframe = self.make_dframe(command, devices_by_bus)
        return self._generic_read_command(generic_conversion, command, dframe, return_type)

    def get_fw_version(self, devices_by_bus=None, return_type='c'):

        ''' Reads the FW version

        Arguments:
        devices_by_bus:
        return_type:
        '''
        command = 11
        def _conversion(raw):
            return str(raw[1]) + '.' + str(raw[0])

        dframe = self.make_dframe(command, devices_by_bus)
        (success, command, data) = self._generic_read_command(_conversion, command, dframe, return_type)
        return (success, command, data)

    def get_device_type(self, devices_by_bus=None, return_type='c'):
        command = 12
        dframe = self.make_dframe(command, devices_by_bus)

        (success, command, data) = self._generic_read_command(generic_conversion, command, dframe, return_type)
        return (success, command, data)

    def get_move_status(self, devices_by_bus=None, return_type='c'):
        command = 13
        dframe = self.make_dframe(command, devices_by_bus)
        (success, command, data) = self._generic_read_command(generic_conversion, command, dframe, return_type, delay=0.5)
        return (success, command, data)

    def get_imons(self, devices_by_bus=None, return_type='c'):
        command = 14
        dframe = self.make_dframe(command, devices_by_bus)
        (success, command, data) = self._generic_read_command(imons_conversion, command, dframe, return_type, delay=10.)
        return (success, command, data)

    def get_bl_version(self, devices_by_bus=None, return_type='c'):
        command = 15
        def _conversion(raw):
            return str(raw[1])+'.'+str(raw[0])

        dframe = self.make_dframe(command, devices_by_bus)

        (success, command, data) = self._generic_read_command(_conversion, command, dframe, return_type)
        return (success, command, data)

    def set_fiducial_duty(self, devices_by_bus=None, duty=0.):
        command = 16
        duty = min(100., max(duty, 0.))
        duty = format_byte_str(int(65535.*duty/100), 4)
        if devices_by_bus:
            dframe = self._make_dframe(command, devices_by_bus, data=duty)
        else:
            dframe = self._make_dframe_broadcast(command, BRDCAST_FID, data=duty)
        return (True, command, self.canh.send(dframe))

    def set_fiducial_duty_by_device(self, devices_by_bus=None, duty_by_device=None):
        command = 16
        print(devices_by_bus, duty_by_device)
        return str(command) + ' not yet implemented'

    def _get_sid_generic(self, command, devices_by_bus, return_type):
        dframe = self.make_dframe(command, devices_by_bus)
        try:
            sid = self.canh.send_recv_locked(dframe)
            if return_type in ['r', 'raw']:
                return (True, command, sid)
            formatted_sid = {busid:{} for busid in self.busids}
            for bus_id, bus_data in sid.items():
                for can_id, item in bus_data.items():
                    formatted_sid[bus_id][can_id] = ":".join("{:02x}".format(c) for c in item)
            return (True, command, formatted_sid)
        except:
            return (False, command, 'Error')

    #def read_sid_lower(self, devices_by_bus=None, return_type='c'):
    #    command = 17
    #    return self._get_sid_generic(command, devices_by_bus, return_type)

    #def read_sid_upper(self, devices_by_bus=None, return_type='c'):
    #    command = 18
    #    return self._get_sid_generic(command, devices_by_bus, return_type)

    def read_sid_short(self, devices_by_bus=None, return_type='c'):
        command = 19
        return self._get_sid_generic(command, devices_by_bus, return_type)

    def write_can_address(self, busid, canid, new_canid):
        command = 20
        new_canid = format(int(new_canid), 'x').zfill(4)
        dframe = {}
        dframe[busid] = [(canid, command, new_canid)]
        return (True, command, self.canh.send(dframe))

    def read_can_address(self, devices_by_bus=None, return_type='c'):
        command = 21
        if devices_by_bus:
            dframe = self._make_dframe(command, devices_by_bus)
        else:
            dframe = self._make_dframe_broadcast(command)

        (error_code, command, data) = self._generic_read_command(generic_conversion, command, dframe, return_type)
        return (error_code, command, data)

    def _check_sid(self, command, devices_by_bus=None, data=None):

        if data:
            if devices_by_bus:
                dframe = self._make_dframe(command, devices_by_bus, data)
            else:
                dframe = self._make_dframe_broadcast(command, BRDCAST_ALL, data)

            return (True, command, self.canh.send(dframe))

        return (False, command, 'Failed')

    #def check_sid_lower(self, devices_by_bus=None, sid=None):
    #    command = 22
    #    return self._check_sid(command, devices_by_bus, sid)

    #def check_sid_upper(self, devices_by_bus=None, sid=None):
    #    command = 23
    #    return self._check_sid(command, devices_by_bus, sid)

    def check_sid_short(self, devices_by_bus=None, sid=None):
        command = 24
        return self._check_sid(command, devices_by_bus, sid)

    def update_can_address(self, busid, canid, new_canid):
        dframe = {busid:[canid]}
        sid = self.read_sid_short(dframe)
        if sid[0]:
            sid = sid[2][busid][canid]
            sid = (sid.split(':'))
            sid.reverse()
            sid = "".join(sid)
            self.check_sid_short(dframe, sid)
            retval = self.write_can_address(busid, canid, new_canid)
        else:
            retval = sid
        return retval

    def set_device_type(self, devices_by_bus=None, device_type=0):
        command = 25
        try:
            if int(device_type) not in [0, 1]:
                return 'FAILED: not a valid device type'
        except:
            return 'FAILED: not a valid device type'
        try:
            devtype = format_byte_str(device_type, 2)
            if devices_by_bus:
                dframe = self._make_dframe(command, devices_by_bus, devtype)
            else:
                dframe = self._make_dframe_broadcast(command, BRDCAST_ALL, devtype)
            return self.canh.send(dframe)
        except:
            return 'FAILED: could not set devicetype'

    def execute_movetable(self, devices_by_bus=None, immediate=True):
        command = 27
        if immediate:
            data = format_byte_str(1, 2)
        else:
            data = format_byte_str(0, 2)

        if devices_by_bus:
            dframe = self._make_dframe(command, devices_by_bus, data)
        else:
            dframe = self._make_dframe_broadcast(command, BRDCAST_POS, data)
        #print(dframe)
        return (True, command, self.canh.send(dframe))

    def get_checksum(self, devices_by_bus=None, return_type='c'):
        command = 28

        def _conversion(raw):
            return raw[0]

        dframe = self.make_dframe(command, devices_by_bus)

        (error_code, command, data) = self._generic_read_command(generic_conversion, command, dframe, return_type)
        return (error_code, command, data)

    #def set_currents_legacy(self): #, busid, devices_by_bus=None, currents_by_busid=None):
    #    command = 30
    #    return str(command) + ' not yet implemented'

    #def set_motor_params_legacy(self): #, busid, devices_by_bus=None, motor_params_by_busid=None):
    #    command = 31
    #    return str(command) + ' not yet implemented'

    # commands 32 - 35 are no longer implemented

    #def get_bl_version_alt(self, devices_by_bus=None, return_type='c'):
    #    command = 36
    #    #def _conversion(raw):
    #    #    return raw
    #
    #    dframe = self.make_dframe(command, devices_by_bus)
    #    (error_code, command, data) = self._generic_read_command(generic_conversion, command, dframe, return_type)
    #    return (error_code, command, data)
    #
    #def get_fw_version_alt(self, devices_by_bus=None, return_type='c'):
    #    command = 37
    #    #def _conversion(raw):
    #    #    return raw
    #
    #    dframe = self.make_dframe(command, devices_by_bus)
    #    (error_code, command, data) = self._generic_read_command(generic_conversion, command, dframe, return_type)
    #    return (error_code, command, data)

    def set_need_double_pulse(self, devices_by_bus=None, need_double=False):
        command = 38
        data = 0
        if need_double:
            data = 1
        data = format_byte_str(data, 2)
        dframe = self.make_dframe(command, devices_by_bus, data)
        print('inside set_need_double, dframe: '+str(dframe))
        return (True, command, self.canh.send(dframe))

    def get_runtime(self, devices_by_bus=None, return_type='c'):
        '''
        Reports the elapsed processor running time in increments of one second since the processor was reset.
        This continues to run in Stop Mode.  A soft RESET (Cmd55) or a hard RESET (Pwr On) resets the time to zero.
        '''
        command = 39
        dframe = self.make_dframe(command, devices_by_bus)
        (error_code, command, data) = self._generic_read_command(generic_conversion, command, dframe, return_type)
        return (error_code, command, data)

    def enter_stop_mode_sync(self, devices_by_bus=None):
        command = 40
        dframe = self.make_dframe(command, devices_by_bus)
        return (True, command, self.canh.send(dframe))

    #def enter_stop_mode_can(self, devices_by_bus=None):
    #    command = 41
    #    dframe = self.make_dframe(command, devices_by_bus)
    #    return (True, command, self.canh.send(dframe))

    def soft_reset(self, devices_by_bus=None):
        '''
        Causes the processor to be RESET by setting bit2, SYSRESETREQ, in register SCB_AIRCR.  This is a soft reset,
        but the code causes the counter of Cmd 39 to be reset.
        '''
        command = 42
        dframe = self.make_dframe(command, devices_by_bus)
        return (True, command, self.canh.send(dframe))

    def enter_bootmode(self, devices_by_bus=None):
        command = 43
        dframe = self.make_dframe(command, devices_by_bus)
        return (True, command, self.canh.send(dframe))

    def dump_n_bytes(self, devices_by_bus=None, hex_address=None, n_bytes=8, return_type='raw'):
        command = 44
        def _conversion(raw):
            return raw.hex()

        if hex_address:
            data = hex_address.replace('0x', '').zfill(4) + format_byte_str(n_bytes, 2)
        else:
            return 'FAILED: invalid (hex) address'
        dframe = self.make_dframe(command, devices_by_bus, data)
        (error_code, command, data) = self._generic_read_command(_conversion, command, dframe, return_type, delay=275.)
        return (error_code, command, data)

    def get_fw_checksum(self, devices_by_bus=None, return_type='c'):
        command = 45
        def _conversion(raw):
            return raw.hex()

        dframe = self.make_dframe(command, devices_by_bus)
        (error_code, command, data) = self._generic_read_command(_conversion, command, dframe, return_type, delay=350.)
        return (error_code, command, data)

    def get_sync_status(self, devices_by_bus=None, return_type='r'):
        command = 46
        dframe = self.make_dframe(command, devices_by_bus)
        (error_code, command, data) = self._generic_read_command(generic_conversion, command, dframe, return_type, delay=10.)
        return (error_code, command, data)

    def get_sysclock(self, devices_by_bus=None, return_type='c'):
        command = 47
        dframe = self.make_dframe(command, devices_by_bus)
        (error_code, command, data) = self._generic_read_command(generic_conversion, command, dframe, return_type)
        return (error_code, command, data)

    #def set_fid_pwm_frequency(self): #, devices_by_bus=None, fid_pwm_by_bus=None):
    #    command = 48
    #    return str(command) + ' not yet implemented'

    #def get_fid_pwm_frequency(self, devices_by_bus=None, return_type='r'):
    #    command = 49
    #    dframe = self.make_dframe(command, devices_by_bus)
    #    (error_code, command, data) = self._generic_read_command(generic_conversion, command, dframe, return_type)
    #    return (error_code, command, data)

    # command 50: dc_current_test (not implemented for production)

    # command 51: not used

    def enter_standby(self, devices_by_bus=None):
        command = 52
        dframe = self.make_dframe(command, devices_by_bus)
        return (True, command, self.canh.send(dframe))

    # command 53: not used

    def set_fid_timed(self): #, devices_by_bus=None, fid_settings_by_bus=None):
        command = 54
        return str(command) + ' not yet implemented'

    def get_fid_status(self, devices_by_bus=None, return_type='raw'):
        command = 55
        dframe = self.make_dframe(command, devices_by_bus)
        (error_code, command, data) = self._generic_read_command(generic_conversion, command, dframe, return_type, delay=1.)
        return (error_code, command, data)

    # command 56 - 58: not used

    def store_params(self, devices_by_bus=None, return_type='c'):
        command = 59
        def _conversion(raw):
            return raw.hex()
        dframe = self.make_dframe(command, devices_by_bus)
        (error_code, command, data) = self._generic_read_command(_conversion, command, dframe, return_type, delay=27.)
        return (error_code, command, data)

    def make_stored_params_default(self, devices_by_bus=None):
        command = 60
        dframe = self.make_dframe(command, devices_by_bus)
        return (True, command, self.canh.send(dframe))

    def read_defaults_ram(self, devices_by_bus=None, return_type='r'):
        command = 61
        def _conversion(raw):
            return [r.hex() for r in raw]
        dframe = self.make_dframe(command, devices_by_bus)
        (error_code, command, data) = self._generic_read_command(_conversion, command, dframe, return_type, delay=5.)
        return (error_code, command, data)


    def read_defaults_flash(self, devices_by_bus=None, return_type='r'):
        command = 62
        def _conversion(raw):
            return [r.hex() for r in raw]
        dframe = self.make_dframe(command, devices_by_bus)
        (error_code, command, data) = self._generic_read_command(_conversion, command, dframe, return_type, delay=5.)
        return (error_code, command, data)

    def read_inter_priorities(self, devices_by_bus=None, return_type='r'):
        command = 63
        def _conversion(raw):
            return [r.hex() for r in raw]
        dframe = self.make_dframe(command, devices_by_bus)
        (error_code, command, data) = self._generic_read_command(_conversion, command, dframe, return_type, delay=5.)
        return (error_code, command, data)    

    # command 62 - 63: not used

    #def read_memory(self, devices_by_bus=None, return_type='r'):
    #    command = 64
    #    dframe = self.make_dframe(command, devices_by_bus)
    #    (error_code, command, data) = self._generic_read_command(generic_conversion, command, dframe, return_type)
    #    return (error_code, command, data)

    def write_to_memory(self): #, devices_by_bus=None, data=None):
        command = 65
        if self.verbose:
            print('not implemented yet')
        return str(command) + ' not yet implemented'

    def can_dump_short(self, devices_by_bus=None, return_type='r'):
        command = 66
        dframe = self.make_dframe(command, devices_by_bus)
        (error_code, command, data) = self._generic_read_command(generic_conversion, command, dframe, return_type)
        return (error_code, command, data)

    # command 67: not used

    # command 68 (setup_vibramove) removed from FW

    # command 69 (execute_vibramove) removed from FW

    def clear_movetable(self, devices_by_bus=None):
        command = 70

        if devices_by_bus:
            dframe = self._make_dframe(command, devices_by_bus)
        else:
            dframe = self._make_dframe_broadcast(command)
        return (True, command, self.canh.send(dframe))

    def can_dump_long(self, devices_by_bus=None, return_type='raw'):
        command = 71
        dframe = self.make_dframe(command, devices_by_bus)
        (error_code, command, data) = self._generic_read_command(generic_conversion, command, dframe, return_type)
        return (error_code, command, data)

    def get_interrupt_reg(self, devices_by_bus=None, return_type='raw'):
        command = 72
        dframe = self.make_dframe(command, devices_by_bus)
        (error_code, command, data) = self._generic_read_command(generic_conversion, command, dframe, return_type)
        return (error_code, command, data)

    def set_ack_move(self, devices_by_bus=None, ack_move=False):
        command = 73
        data = 0
        if ack_move:
            data = 1
        data = format_byte_str(data, 2)
        dframe = self.make_dframe(command, devices_by_bus, data)
        return (True, command, self.canh.send(dframe))

    def set_abom(self, devices_by_bus=None, abom=False):
        command = 74
        data = 0
        if abom:
            data = 1
        data = format_byte_str(data, 2)
        dframe = self.make_dframe(command, devices_by_bus, data)
        return (True, command, self.canh.send(dframe))

    # command 75 (set_fast_hclk) was removed from FW

    def set_verbose_flag(self, devices_by_bus=None, verbose_flag=0):
        command = 76
        data = 0
        if verbose_flag in [0, 1, 3, 2, 4]:
            data = format_byte_str(verbose_flag, 2)
            dframe = self.make_dframe(command, devices_by_bus, data)
            return (True, command, self.canh.send(dframe))
        return (False, command, 'invalid verbose flag')

    def set_creep_step_size(self, devices_by_bus=None, step_size_new=None):
        command = 79
        # motor 0 == phi, motor 1 == theta

        step_size = {'phi':1, 'theta':1}
        if step_size_new:
            for m in ['theta', 'phi']:
                if m in step_size_new:
                    step_size[m] = step_size_new[m]
        data = ''
        for m in ['phi', 'theta']:
            data += format_byte_str(step_size[m], 2)
        dframe = self.make_dframe(command, devices_by_bus, data)
        if self.debug_level:
            print('inside set_creep_step_size, dframe: '+str(dframe))
        try:
            return self.canh.send(dframe)
        except:
            return 'FAILED: could not set creep step size'

    def set_first_phase_jump(self, devices_by_bus=None, phase_jmp=None):
        command = 80
        # default values
        if not phase_jmp:
            phase_jmp = {'d0':0, 'd1':0, 'd2':0, 'd3':0, 'd4':0, 'd5':0, 'd6':0, 'd7':0}
        print(phase_jmp)
        data = (format_byte_str(phase_jmp['d0'], 2) + \
                format_byte_str(phase_jmp['d1'], 2) + \
                format_byte_str(phase_jmp['d2'], 2) + \
                format_byte_str(phase_jmp['d3'], 2) + \
                format_byte_str(phase_jmp['d4'], 2) + \
                format_byte_str(phase_jmp['d5'], 2) + \
                format_byte_str(phase_jmp['d6'], 2) + \
                format_byte_str(phase_jmp['d7'], 2))

        dframe = self.make_dframe(command, devices_by_bus, data)
        try:
            return self.canh.send(dframe)
        except:
            error = 'FAILED: could not set phase jump'
            success = False
        return (success, command, error)


    def set_short_spin_ramp_parameters_phi(self, devices_by_bus=None, rotation=None):
        # motor 0 = phi
        command = 81
        if not rotation:
            rotation = {'cw_spinup':0, 'cw_spindown':0, 'ccw_spinup':0, 'ccw_spindown':0}
            
        data = (format_byte_str(nint(rotation['cw_spinup']*GEAR_RATIO*10), 4) + \
                format_byte_str(nint(rotation['cw_spindown']*GEAR_RATIO*10), 4) + \
                format_byte_str(nint(rotation['ccw_spinup']*GEAR_RATIO*10), 4) + \
                format_byte_str(nint(rotation['ccw_spindown']*GEAR_RATIO*10), 4))

        dframe = self.make_dframe(command, devices_by_bus, data)
        try:
            return self.canh.send(dframe)
        except:
            error = 'FAILED: could not set phase jump'
            success = False
        return (success, command, error)

    def set_short_spin_ramp_parameters_theta(self, devices_by_bus=None, rotation=None):
        # motor 1 = theta
        command = 82
        if not rotation:
            rotation = {'cw_spinup':0, 'cw_spindown':0, 'ccw_spinup':0, 'ccw_spindown':0}
            
        data = (format_byte_str(nint(rotation['cw_spinup']*GEAR_RATIO*10), 4) + \
                format_byte_str(nint(rotation['cw_spindown']*GEAR_RATIO*10), 4) + \
                format_byte_str(nint(rotation['ccw_spinup']*GEAR_RATIO*10), 4) + \
                format_byte_str(nint(rotation['ccw_spindown']*GEAR_RATIO*10), 4))

        dframe = self.make_dframe(command, devices_by_bus, data)
        try:
            return self.canh.send(dframe)
        except:
            error = 'FAILED: could not set phase jump'
            success = False
        return (success, command, error)

    def set_alpha_zero(self):
        command = 86
        if self.verbose:
            print('not implemented yet')
        return (False, command, 'not implemented yet')

    def dump_alpha_cw_up(self):
        command = 87
        if self.verbose:
            print('not implemented yet')
        return (False, command, 'not implemented yet')


    def dump_alpha_cw_down(self):
        command = 88
        if self.verbose:
            print('not implemented yet')
        return (False, command, 'not implemented yet')


    def dump_alpha_ccw_up(self):
        command = 89
        if self.verbose:
            print('not implemented yet')
        return (False, command, 'not implemented yet')


    def dump_alpha_ccw_down(self):
        command = 90
        if self.verbose:
            print('not implemented yet')
        return (False, command, 'not implemented yet')

    # +-----------------------------------------------------------------------------+
    # |                  move commands and other composite commands                |
    # +-----------------------------------------------------------------------------+

    #def set_creep_speed(self, devices_by_bus=None, new_creep_settings=None):
    #    # The creep speed can be modified by changing the creep_steps and the creep_period.
    #    # Creep rotation rate (in RPM at motor shaft) is given by: 300 x (creep_step_size/creep_period)
    #    # The default at KPNO (up to at least Fall 2022) is a creep RPM of 150 (creep_period == 2, creep_step_size == 1)
    #    #
    #    # creep_settings == {'periods':{'phi':2,'theta':2},'steps:{'phi':1,'theta':1}}#
    #
    #    #print('new: ',new_creep_settings)
    #    creep_settings = {'periods':{'phi':2, 'theta':2}, 'step_size':{'phi':1, 'theta':1}}
    #
    #    if new_creep_settings:
    #        for p in ['periods', 'step_size']:
    #            for m in ['theta', 'phi']:
    #                if p in new_creep_settings:
    #                    if m in new_creep_settings[p]:
    #                        creep_settings[p][m] = new_creep_settings[p][m]
    #    periods = creep_settings['periods']
    #        retval = self.set_periods(devices_by_bus, periods)
    #
    #    if retval[0:4].upper() in ['FAIL']:
    #        return retval
    #        step_size = creep_settings['step_size']
    #    #print('step_size',step_size)
    #    retval = self.set_creep_step_size(devices_by_bus, step_size)
    #        return retval

    def move(self, devices_by_bus, direction='cw', speed='cruise', motor='theta', angle=15):
        '''
        Preserves the simple (direct) move command using the new move table format.
        Only one axis at a time move.
        '''

        # initialize the 8 data bytes
        header = 0                              # 1 byte
        phi_steps = format_byte_str(0, 5)       # 2.5 bytes
        theta_steps = format_byte_str(0, 5)     # 2.5 bytes
        post_pause = format_byte_str(0, 4)      # 2 bytes

        # end of move table
        header = set_bit(header, 7)
        #header = self.pcf.set_bit(header, 6)

        direction = direction.lower()
        if direction not in ['cw', 'ccw']:
            return 'FAILED: invalid direction argument'

        speed = speed.lower()
        if speed not in ['creep', 'cruise']:
            return 'FAILED: invalid speed mode argument'

        motor = motor.lower()
        if motor not in ['theta', 'phi']:
            return 'FAILED: invalid motor argument'

        if motor in ['phi']:
            #// data[0]: bit_5  -- Phi Move
            #// data[0]: bit_4  -- Phi Cruise
            #// data[0]: bit_3  -- Phi CCW
            header = set_bit(header, 5)
            if direction in ['ccw']:
                header = set_bit(header, 3)
            if speed in ['cruise']:
                header = set_bit(header, 4)
                motor_steps = int(round(angle*GEAR_RATIO/3.3))
                phi_steps = format_byte_str(motor_steps, 5)
            if speed in ['creep']:
                motor_steps = int(round(angle*GEAR_RATIO/.1))
                phi_steps = format_byte_str(motor_steps, 5)

        else: # theta
            #// data[0]: bit_2  -- Theta Move
            #// data[0]: bit_1  -- Theta Cruise
            #// data[0]: bit_0  -- Theta CCW
            header = set_bit(header, 2)
            if direction in ['ccw']:
                header = set_bit(header, 0)
            if speed in ['cruise']:
                header = set_bit(header, 1)
                motor_steps = int(round(angle*GEAR_RATIO/3.3))
                theta_steps = format_byte_str(motor_steps, 5)
            if speed in ['creep']:
                motor_steps = int(round(angle*GEAR_RATIO/.1))
                theta_steps = format_byte_str(motor_steps, 5)
        header = format_byte_str(header, 2)
        data = str(header + theta_steps[0:1] + phi_steps[0:1] + phi_steps[1:] + theta_steps[1:] + post_pause)
        #print('data: ', data)

        command = 26
        dframe = self.make_dframe(command, devices_by_bus, data)
        #print('dframe: ' + str(dframe))
        try:
            retval = self.canh.send(dframe)
            return (True, command, retval)
        except:
            return (False, command, 'Error')

    def move_direct(self, devices_by_bus, direction='cw', speed='cruise', motor='theta', angle=15):
        print('>>> angle: ',angle)
        retval = self.move(devices_by_bus, direction, speed, motor, angle)
        if retval not in ['Error']:
            time.sleep(0.1)
            retval = self.execute_movetable(devices_by_bus)
            return (True, 26, retval)
        return (False, 26, retval)

    def build_can_frames(self, move_tables):
        '''
        move_tables is a list of dictionaries. Each dictionary is the movetable for one positioner.

        The dictionary has the following fields:

        {'canid' : 0, 'busid' : 'can0', 'nrows' : 0, 'motor_steps_T' : [],
        'motor_steps_P' : [], 'speed_mode_T' : [], 'speed_mode_P' : [],
        'move_time' : [], 'postpause' : []}
        The fields have the following types and meanings:

        canid          (unsigned int)         : device ID
        busid          (string      )         : busid (eg. 'can0')
        nrows          (unsigned int)         : number of rows in movetable

        motor_steps_T
        motor_steps_P  (list of ints)         : number of motor steps to rotate
                                                > 0 ... ccw rotation
                                                < 0 ... cw rotation
        speed_mode_T
        speed_mode_P   (list strings)         : 'cruise' or 'creep'
        move_time      (list unsigned floats) : estimated time the row's motion will take
                                                (seconds, not including the postpause)
        postpause      (list unsigned ints)   : pause time after the row's motion, in
                                                millisecs, before executing the next row
        '''
        command_26 = 26
        command_27 = 27
        command_28 = 28

        #DEBUG = self.debug_level

        if not move_tables:
            return 'FAILED: no movetables', {}, {}, {}

        move_commands_by_busid = dict((busid, []) for busid in self.busids)
        checksum_commands_by_busid = dict((busid, []) for busid in self.busids)
        checksums_by_busid = dict((busid, {}) for busid in self.busids)
        arm_commands_by_busid = dict((busid, []) for busid in self.busids)

        mcounter = 0

        table_movetimes = []
        maxmovetime = {}
        for table in move_tables:
            canid = int(table['canid'])

            busid = str(table['busid'])
            if busid not in self.busids:
                msg = 'FAILED: requested bus ID not allowed '+str(busid)
                print(msg)
                return msg, {}, {}, {}, {}

            nrows = table['nrows']

            try:
                table_movetimes.append([sum(table['move_time']) + sum(table['postpause'])/1000., busid, canid])
            except Exception:
                print('<send_tables> error in movetimes.append')

            bitsum = 0
            max_steps = (1<<20) - 1
            for row in range(nrows):
                motor_steps_T = table['motor_steps_T'][row]
                motor_steps_P = table['motor_steps_P'][row]
                speed_mode_T = table['speed_mode_T'][row]
                speed_mode_P = table['speed_mode_P'][row]
                post_pause = nint(table['postpause'][row])

                try:
                    if abs(motor_steps_T) > max_steps:
                        print('FAILED: Theta motor steps exceed maximum of %d: %d' % (max_steps, motor_steps_T))
                        return (msg, {}, {}, {}, {})
                    if abs(motor_steps_P) > max_steps:
                        print('FAILED: Phi motor steps exceed maximum of %d: %d' % (max_steps, motor_steps_P))
                        return (msg, {}, {}, {}, {})
                except Exception as err:
                    print('FAILED: motor_steps error %s' % str(err))
                    return (msg, {}, {}, {}, {})

                # initialize the 8 data bytes

                header = 0                                    # 1 byte
                phi_steps = format_byte_str(0, 5)             # 2.5 bytes
                theta_steps = format_byte_str(0, 5)           # 2.5 bytes
                post_pause = format_byte_str(post_pause, 4)   # 2 bytes

                # end of move table ?
                if row == nrows -1: # yup
                    header = set_bit(header, 7)
                else:
                    header = set_bit(header, 6)

                # first phi
                if motor_steps_P not in [0]:
                    #// data[0]: bit_5  -- Phi Move
                    #// data[0]: bit_4  -- Phi Cruise
                    #// data[0]: bit_3  -- Phi CCW
                    header = set_bit(header, 5)
                    if motor_steps_P < 0:
                        header = set_bit(header, 3)
                    if speed_mode_P in ['cruise']:
                        header = set_bit(header, 4)
                    phi_steps = format_byte_str(abs(motor_steps_P), 5)

                # now theta
                if motor_steps_T not in [0]:
                    #// data[0]: bit_2  -- Theta Move
                    #// data[0]: bit_1  -- Theta Cruise
                    #// data[0]: bit_0  -- Theta CCW
                    header = set_bit(header, 2)

                    if motor_steps_T < 0:
                        header = set_bit(header, 0)
                    if speed_mode_T in ['cruise']:
                        header = set_bit(header, 1)
                    theta_steps = format_byte_str(abs(motor_steps_T), 5)

                header = format_byte_str(header, 2)
                data = str(header + theta_steps[0:1] + phi_steps[0:1] + phi_steps[1:] + theta_steps[1:] + post_pause)
                d = [int(data[i:i+2], 16) for i in range(0, 16, 2)]
                bitsum += d[0] + 65536*d[1] + 256*d[2] + d[3] + 256*d[4] + d[5] + 256*d[6] + d[7] + 26
                dframe = (canid, command_26, data)
                move_commands_by_busid[busid].append(dframe)

            checksum_cmd = (canid, command_28, '')
            checksum_commands_by_busid[busid].append(checksum_cmd)
            checksums_by_busid[busid][canid] = bitsum
            arm_commands_by_busid[busid].append((canid, command_27, '00'))    # activate move table command
            mcounter += 1

        maxtime_index = np.argmax(list(zip(*table_movetimes))[0])
        maxmovetime['time'] = list(zip(*table_movetimes))[0][maxtime_index]
        maxmovetime['busid'] = list(zip(*table_movetimes))[1][maxtime_index]
        maxmovetime['canid'] = list(zip(*table_movetimes))[2][maxtime_index]

        if self.debug_level:
            print('<_build_can_frames> MAX MOVETIME: '+str(round(maxmovetime['time'], 2)))
            print('<_build_can_frames> MOVETABLES: ' + str(move_commands_by_busid))
            print('<_build_can_frames> CHECKSUM REQUESTS:  ' + str(checksum_commands_by_busid))
            print('<_build_can_frames> CHECKSUMS:  ' + str(checksums_by_busid))

        return ('SUCCESS', move_commands_by_busid, checksum_commands_by_busid, checksums_by_busid, arm_commands_by_busid, maxmovetime)

    def move_by_table(self, table):
        # mcbb: move_commands_by_busid
        # ccbb: checksum_commands_by_busid
        # acbb: arm_commands_by_busid

        (retval, mcbb, ccbb, checksums_by_busid, acbb, maxmovetime) = self.build_can_frames(table)

        del ccbb
        del acbb, maxmovetime, checksums_by_busid

        if retval != 'SUCCESS':
            print('Error building can frames')
            return

        retval = self.canh.send(mcbb)
        print('sending movetables. retval: '+str(retval))

        checksums = self.get_checksum()
        print('returned checksums: '+str(checksums))

        retval = self.execute_movetable()
        print('execute movetable. retval: ' + str(retval))

        self.still_moving(.5, 40.)

    def still_moving(self, interval, timeout):
        timeout = min(timeout, 60)
        interval = max(min(1., interval), .1)

        start = time.time()
        while time.time() - start < timeout:
            print(str(round(time.time() - start, 3)) + str(self.get_move_status()))
            time.sleep(interval)

    # +-------------------------------------------------------------------+
    # |                  FW UPLOAD                                        |
    # +-------------------------------------------------------------------+

    def FW_send_codesize(self, devices_by_bus):   #send size of firmware code in words
        try:
            size = str(hex(self.codesize).replace('0x', '')).zfill(8)
            dframe = self._make_dframe(129, devices_by_bus, data=size)
            return self.canh.send(dframe)
        except Exception as exception:
            return 'FAILED: Error sending code size: %s' % str(exception)

    def FW_send_nparts(self, devices_by_bus): #send the number of parts that the code has been divided into
        try:
            parts = str(hex(self.nparts).replace('0x', '')).zfill(8)
            dframe = self._make_dframe(130, devices_by_bus, data=parts)
            return self.canh.send(dframe)
        except Exception as exception:
            return 'FAILED: Error sending number of parts in hex file: %s' % str(exception)

    def FW_send_packet(self, devices_by_bus, partn=1, packetn=1, packet='01234567', chksum=32):
        try:
            chksum = str(hex(chksum).replace('0x', '')).zfill(2)
            partn = str(partn).zfill(2)
            packetn = str(hex(packetn).replace('0x', '')).zfill(4)

            packet_data = str(partn + packetn + packet + chksum)
            dframe = self._make_dframe(132, devices_by_bus, data=packet_data)
            return self.canh.send_recv_locked(dframe)
        except Exception as e:
            return 'FAILED: Error sending firmware packet: %s' % str(e)

    def FW_get_packet(self, partn=1, packetn=0):
        #retrieve packet for given part number and packet number from hex file
        try:
            packet_addr = self.hexfile_minaddr + (partn - 1)*self.part_size  + 4*packetn      #start address of 4 byte packet
            packet = ''
            packet_byte = [0, 0, 0, 0]
            checksum_byte = [0, 0, 0, 0]
            if partn == self.nparts and packetn > self.FW_get_packets_in_n(partn):
                packet = '00000000'
                checksum = 0
            else:
                for i in range(0, 4):
                    checksum_byte[i] = int(hex(self.hexfile[packet_addr + i]), 16)
                    packet_byte[i] = str(hex(self.hexfile[packet_addr + i]).replace('0x', '')).zfill(2)
                    packet = packet + packet_byte[i]
                checksum = checksum_byte[0] ^ checksum_byte[1] ^ checksum_byte[2] ^ checksum_byte[3]
            return packet, checksum
        except Exception as e:
            return 'FAILED: Error retrieving packet from hex file: %s' % str(e)

    def FW_get_packets_in_n(self, partn):
        try:
            if partn == self.nparts:
                packets_n = self.codesize - (self.nparts - 1) * (self.part_size / 4)
            else:
                packets_n = self.part_size / 4
            return int(packets_n)

        except Exception as e:
            return 'FAILED: Error retrieving packets in part n: %s' % str(e)

    def FW_enter_bootmode(self, devices_by_bus=None):
        # send command 43 to enter boot mode
        dframe = self.make_dframe(43, devices_by_bus, '')
        retval = self.canh.send(dframe)
        print(' Sending command 43 ' + str(retval))
        bootmode_code = '4d2e452e4c657669'
        delay = 1.
        # now send command 128 with 'special code'
        print(' Sending command 128 ')
        dframe = self.make_dframe(128, devices_by_bus, bootmode_code)
        try:
            raw_response = self.canh.send_recv_locked(dframe, delay)
            print(' raw response: ' + str(raw_response))
            response = {}
            for bus, devices in raw_response.items():
                for device in devices:
                    if bus not in response:
                        response[bus] = {}
                    if raw_response[bus][device].hex() in ['426f6f544d6f6465']:
                        state = 'ready'
                    else:
                        state = 'not ready'
                    response[bus][device] = state
            return response
        except:
            return 'FAIL'

    def FW_enter_bootmode128(self, devices_by_bus=None, pospower=None):

        #response = input('\nSwitch the power off/on then hit return within 2 seconds ....')

        bootmode_code = '4d2e452e4c657669'

        # now send command 128 with 'special code'
        dframe = self.make_dframe(128, devices_by_bus, bootmode_code)
        print(dframe)
        try:
            print('1')
            raw_response = self.canh.send_recv_locked(dframe)
            print('2')
            #if self.verbose:
            print('Commnd 128 - return_data: ', str(raw_response))

            response = {}
            for bus, devices in raw_response.items():
                for device in devices:
                    if bus not in response:
                        response[bus] = {}
                    if raw_response[bus][device].hex() in ['426f6f544d6f6465']:
                        state = 'ready'
                    else:
                        state = 'not ready'
                    response[bus][device] = state
            return response
        except:
            return 'FAIL'

    def FW_verification(self, devices_by_bus=None):
        dframe = self.make_dframe(131, devices_by_bus, '')
        delay = 0.
        return_type = 'raw'
        (error_code, command, data) = self._generic_read_command(generic_conversion, 131, dframe, delay, return_type)
        return (error_code, command, data)

    def FW_program(self, devices_by_bus, hex_file='/home/msdos/focalplane/pclight/hexfiles/fw604page4.hex'):
        self.verbose = True
        if not hex_file:
            return 'FAILED: no hexfile specified'
        self.part_size = 16000
        if self.verbose:
            print('FW hexfile: ', hex_file)
        try:
            self.hexfile = IntelHex(hex_file)
            self.hexfile_minaddr = self.hexfile.minaddr()
            self.hexfile_maxaddr = self.hexfile.maxaddr()
            #self.hexfile_minaddr = 134225920
            #self.hexfile_maxaddr = 134225920+15999
            #self.hexfile_minaddr = 134225920+16000
            #self.hexfile_maxaddr = 134261151
            #code size in 32-bit words hex
            self.codesize = int(math.ceil(float(self.hexfile_maxaddr - self.hexfile_minaddr + 1) / 4))
            self.nparts = int(math.ceil(float(self.hexfile_maxaddr - self.hexfile_minaddr + 1) / self.part_size))
            if self.verbose:
                print(' hexfile maxaddr: ' + str(self.hexfile_maxaddr))
                print(' hexfile minaddr: ' + str(self.hexfile_minaddr))
                print(' number of packets: '+ str(int(math.ceil(float(self.hexfile_maxaddr - self.hexfile_minaddr + 1) / 4))))
                for p_nr in range(int(self.nparts)):
                    print(' packets_in 1 ' + str(self.FW_get_packets_in_n(p_nr+1)))
                    #print(' packets_in 2 ' + str(self.FW_get_packets_in_n(2)))
                print(' nparts ' + str(self.nparts))
            self.FW_send_codesize(devices_by_bus)
            time.sleep(0.01)

            self.FW_send_nparts(devices_by_bus)
            time.sleep(0.3)
            #loop through parts
            for count in range(1, (self.nparts + 1)):
                if self.verbose:
                    print('programming part ' + str(count))
                time.sleep(1)
                #packet_array = []
                for packet in range(0, self.FW_get_packets_in_n(count)):
                    #packet_array = []
                    packet_np, chksum = self.FW_get_packet(count, packet)
                    #packet_array.append(packet_np)
                    self.FW_send_packet(devices_by_bus, count, packet, packet_np, chksum)
                    time.sleep(0.002)
                time.sleep(.5)
                #print(packet_array)
            return 'SUCCESS'
        except Exception as e:
            return 'FAILED: Error with bootloader programming: %s' % str(e)

if __name__ == '__main__':
    fipos = FiposControl('can0')

    print("... get temperature")
    r = fipos.get_temperature()
    print(r)
