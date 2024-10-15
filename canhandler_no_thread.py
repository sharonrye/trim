'''
# minimalistic canhandler for FW upload
#
'''

import cansocket
import time

class CANHandler(object):
    """
    Class for communicating with multiple CAN buses simultaneously (each bus is implemented as an instance of the CANSocket class).

    initialization input(s):
    busids: list of string CAN bus id names that CANHandler will communicate with
    (eg. ['can10', 'can11', 'can12', 'can13', 'can14', 'can15', 'can16', 'can17', 'can22', 'can23'] )
    """

    def __init__(self, busids, info_func=print, debug_level=1):
        self.busids = busids
        #self.CAN_sockets = dict((busid, cansocket.CANSocket(busid, busids, info_func, debug_level)) for busid in self.busids)
        self.CAN_sockets = dict((busid, cansocket.CANSocket(busid)) for busid in self.busids)
        # create CAN_lock
        self.info = info_func

    def __del__(self):
        return

    def send(self, commands):
        try:
            for busid in self.busids:
                if busid in commands:
                    command_list = commands[busid]
                    for command in command_list:
                        canid, command_num, data = command
                        self.CAN_sockets[busid].send_command(canid, command_num, data)
            return 'SUCCESS'
        except:
            return 'FAIL'

    def recv(self):
        self.responses = dict((busid, []) for busid in self.busids)
        for busid in self.busids:
            self.responses[busid] = self.CAN_sockets[busid].recv()
        return self.responses

    def send_recv_locked(self, commands, delay=0.):
        # convert millisecond delay to seconds for time function and limit to [0,0.5] s
        delay=0.001*min(max(delay,0.),500.) 
        self.send(commands)
        time.sleep(delay)
        return self.recv()

    def can_sleep(self,sleeptime):
        with self.can_lock:
            time.sleep(sleeptime)
        return
