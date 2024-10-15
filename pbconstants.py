# Petal Controller Constants file
# History:
# 08/13/2021  cad Changed TWENTY_CANBUS_TO_TCAN_ID conversion to not use any previousely used can channel names to avoid confusion
# 07/07/2021  cad Added TWENTY_CANBUS_TO_TCAN_ID conversion
# 06/03/2021  cad tcanpy.c is now 1-based rather than 0-based for CAN channels
# 05/05/2021  cad added conversion from traditional canbus name to TCAN ids
# 08/18/2020  cad added DUMP_BYTES_CMD
# 01.14.2020  ms  added header; clean-up (no changes)
#
BROADCAST_ID =   20000		# CAN ID that all devices respond to, used by petalcontroller module for broadcasting commands to all pos/fid
BROADCAST_ID_POS = 20001    # CAN ID that all positioners (and only positioners) respond to
BROADCAST_ID_FID = 20002    # CAN ID that all fiducials (and only fiducials) respond to
EXT_ID_PFX =     '8'        # Extended ID prefix used by cansocket module to specify that we are using an extended CAN id
CAN_WAIT =       10                           #Time to wait for CAN interface to initialize (temporary)
GEAR_RATIO =     (46.0/14.0+1)**4             #Gear ratio used for console move commands (337:1 reduction)
ONOFF_LIST =     ['ON', 'OFF', 'on', 'off', 0, 1] # List of acceptable inputs to methods that deal with GPIO pin states
ON_LIST =        ['ON', 'on', 1]                  # List of acceptable 'on' state inputs for methods dealing with GPIO pin states
POSPWR_WAIT =    5                                # Float/int time (in sec) to wait after power supplies come on before doing anything with them
SYNC_PULSE_WIDTH = 0.1                            # Float/int time (in sec) to hold SYNC line high before setting it low again when using SYNC as trigger
STOP_MODE_BUFFER_TIME = 0.15                      # Time to wait before and after entering stop modes
CHECKSUM_WAIT_TIME = 0.1                          # Time to wait for positioners/fiducials to compute their internal firmware checksums

TIMEOUT = 0.002              # Socket time-out value used by cansocket module, determines how long to wait before reporting a positioner as 'nonresponsive'
BUFF_SIZE = 4096             # Socket buffer size in bytes

FIPOS_CAN_ESR_ADDR = 0x40006418  # Address of CAN Error Status Register in FIPOS board

#Command numbers for communicating with firmware (used by petalcontroller module)
CURRENTS_CMD =    2          # Command for updating positioner current settings
PERIODS_CMD =     3          # Command for setting CREEP and SPIN periods
MOVE_ROW_CMD =    4          # Command for sending move table rows
SYNC_CMD =        7          # Command for triggering move table execution ('soft' sync)
CHECKSUM_CMD =    8          # Command used to send and verify move table checksum
TEMP_CMD =        9          # Command for reading back pos/fid temperature sensor
RD_CANID_CMD =    10         # Command for reading back CAN id from pos/fid flash
FWVR_CMD =        11         # Command for reading back firmware version from pos/fid
RD_DVTYPE_CMD =   12         # Command for reading back device type (positioner or fiducial)
DONE_MOVING_CMD = 13         # Command for checking if positioner is done with its move
IMON_CMD =        14         # Command for reading back current monitor values
BLVR_CMD =        15         # Command for reading back bootloader version from pos/fid
SETFID_CMD =      16         # Command for setting duty percentage for fiducial
SIDL_CMD =        17         # Command for reading back lower bytes of pos/fid silicon id
SIDH_CMD =        18         # Command for reading back upper bytes of pos/fid silicon id
SIDS_CMD =        19         # Command for reading back shortened pos/fid silicon id
WR_CANID_CMD =    20         # Command for writing CAN id into pos/fid flash
CHKSIDL_CMD =     22         # Command used to check lower bytes of silicon id prior to programming new CAN ID
CHKSIDU_CMD =     23         # Command used to check upper bytes of silicon id prior to programming new CAN ID
CHKSIDS_CMD =     24         # Command used to check short silicon id prior to programming new CAN ID
WR_DVTYPE_CMD =   25         # Command for setting device type in flash (pos or fid)
SETBUMP_CMD =     31         # Command for setting CW and CCW bump flags
STOP_MODE_SYNC_CMD = 40      # Command for entering stop mode and exiting via sync signal
STOP_MODE_CAN_CMD =  41      # Command for entering stop mode and exiting via CAN activity
DUMP_BYTES_CMD =     44      # Command for dumping N bytes
MEM_CHECKSUM_CMD =   45      # Command for retrieving firmware computed checksum from each device on the CAN bus
SYNC_STATUS_CMD =    46      # Command for reading back SYNC line status of each device on the CAN bus
SYS_CLK_CMD =        47      # Command for reading back value of the system clock from positinioners/fiducials
FID_SET_PWM_CMD =    48      # Command for setting the fiducial PWM frequency 
FID_GET_PWM_CMD =    49      # Command for retrieving PWM settings from positioners/fiducials
BEE_GEE_CMD =        53      # Command for positioners to stay alive (FW 5.3 and up) 
DUMMY_CMD =          66      # Command that does not do anything in the firmware, just wakes up devices from CAN exit stop mode

# Dictionary for mapping power supplies to the CAN channels that they serve
PS_MAP = {'PS1' : {'can10', 'can11', 'can13', 'can22', 'can23'}, 'PS2' : {'can12', 'can14', 'can15', 'can16', 'can17'}}

# Dictionary mapping SYSTEC CAN boards to CAN channel numbers that they contain
CANBRD_MAP = {'CANBRD1': {'can22', 'can23'}, 'CANBRD2' : {'can10', 'can11', 'can12', 'can13', 'can14', 'can15', 'can16', 'can17'}}

# List of enable lines used for powering up/down
PB_ENABLES = ['PS1_EN', 'PS2_EN', 'CANBRD1_EN', 'CANBRD2_EN', 'BUFF_EN1', 'BUFF_EN2', 'TEC_CTRL', 'GFAPWR_EN']

# Dictionary mapping traditional CAN bus names to TCAH ids.
TRADITIONAL_CANBUS_TO_TCAN_ID = {
        'can10': 12,
        'can11': 11,
        'can12': 7,
        'can13': 9,
        'can14': 4,
        'can15': 3,
        'can16': 2,
        'can17': 1,
        'can22': 6,
        'can23': 5
        }

TWENTY_CANBUS_TO_TCAN_ID = {
        'can30': 1,
        'can31': 2,
        'can32': 3,
        'can33': 4,
        'can34': 5,
        'can35': 6,
        'can36': 7,
        'can37': 8,
        'can38': 9,
        'can39': 10,
        'can40': 11,
        'can41': 12,
        'can42': 13,
        'can43': 14,
        'can44': 15,
        'can45': 16,
        'can46': 17,
        'can47': 18,
        'can48': 19,
        'can49': 20
        }

# Alarm IDs
ALARM = 1211                # Critical alarm (general case) id 
WARN = 1201                 # Warning alarm (general case) id
ALARM_PBINT = 1213          # Critical alarm that cannot be mitigated by petalbox
ALARM_GFA = 1212            # Critical alarm related to GFA
ALARM_PBINT_GFA = 1214      # Critical alarm related to GFA that cannot be mitigated by petalbox
EVENT = 1200
