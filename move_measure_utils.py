import numpy as np
import time
import datetime
from astropy.io import fits
import os
import shutil


# TODO: store IDs of the positioners that did not move?
class MoveNone(Exception):
    pass


class MoveError(Exception):
    def __init__(self, moved):
        self.moved = moved


def angle_between(c, p1, p2):
    # p1, p2 are points; c is center
    a = np.array(p1)
    b = np.array(c)
    c = np.array(p2)
    ba = a - b
    bc = c - b
    cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    angle = np.arccos(cosine_angle)
    return np.degrees(angle)


def get_angle(direction, center, start_point, end_point):
    # this deals correctly with angles > 180 deg
    if direction in ['cw']:
        return calculate_angle_abby(center, start_point, end_point)
    return calculate_angle_abby(center, end_point, start_point)


def calculate_angle(center, a, b):
    # Calculate angle between two points """
    ba = np.array(a) - np.array(center)
    bc = np.array(b) - np.array(center)
    ang_a = np.arctan2(*ba[::-1])
    ang_b = np.arctan2(*bc[::-1])
    return 360 - np.rad2deg((ang_a - ang_b) % (2 * np.pi))


def calculate_angle_abby(center, a, b):
    # calculate angle between 2 points
    ac = np.array(a) - np.array(center)
    bc = np.array(b) - np.array(center)

    dot = ac.dot(bc)

    ac_norm = np.sqrt(ac.dot(ac))
    bc_norm = np.sqrt(bc.dot(bc))

    angle_rad = np.arccos(dot / (ac_norm * bc_norm))

    return np.rad2deg(angle_rad)


def get_datetime_string():
    ts = time.time()
    st = datetime.datetime.fromtimestamp(ts).strftime('%Y%m%d %H:%M:%S')
    st = st.split(' ')
    _date_str = st[0][2:]
    _time_str = st[1].replace(':', '')
    return _date_str, _time_str


def round2(value):
    return str(round(float(value), 2))


def is_point_inside_box(point, box_bottom_left, box_top_right):
    # checks if a point (x,y) is inside a box
    p = point
    bl = box_bottom_left
    tr = box_top_right

    if (p[0] > bl[0] and p[0] < tr[0] and p[1] > bl[1] and p[1] < tr[1]):
        return True
    return False


def reverse_direction(direction):
    if direction in ['cw']:
        reverse_direction = 'ccw'
    elif direction in ['ccw']:
        reverse_direction = 'cw'
    else:
        return 'FAILED: invalid direction'
    return reverse_direction


def get_speed_parameters_cruise(cruise_speed_mode):
    speed_settings = {}
    speed_settings['null'] = {'cruise': 0, 'spinup': 0, 'spindown': 0}
    speed_settings['default'] = {'cruise': 33, 'spinup': 12, 'spindown': 12}
    speed_settings['speed_3'] = {'cruise': 66, 'spinup': 1, 'spindown': 1}
    speed_settings['speed_8'] = {'cruise': 99, 'spinup': 1, 'spindown': 1}
    speed_settings['speed_9'] = {'cruise': 99, 'spinup': 2, 'spindown': 2}
    speed_settings['speed_12'] = {'cruise': 116, 'spinup': 1, 'spindown': 1}
    speed_settings['speed_14'] = {'cruise': 84, 'spinup': 1, 'spindown': 1}
    speed_settings['speed_18'] = {'cruise': 33, 'spinup': 1, 'spindown': 1}
    speed_settings['speed_122'] = {'cruise': 116, 'spinup': 2, 'spindown': 2}  # 34k rpm with ramp angle 4.023 each
    speed_settings['speed_123'] = {'cruise': 116, 'spinup': 2, 'spindown': 1}  # 34k rpm with ramp up 4.023, down 2.012
    speed_settings['speed_124'] = {'cruise': 116, 'spinup': 1, 'spindown': 2}  # 34k rpm with ramp up 2.012, down 4.023
    speed_settings['speed_20'] = {'cruise': 66, 'spinup': 2, 'spindown': 3}
    speed_settings['speed_21'] = {'cruise': 62, 'spinup': 2, 'spindown': 3}

    speed_settings['speed_30'] = {'cruise': 82, 'spinup': 1, 'spindown': 2}
    speed_settings['speed_31'] = {'cruise': 82, 'spinup': 2, 'spindown': 2}
    speed_settings['speed_32'] = {'cruise': 82, 'spinup': 2, 'spindown': 3}

    speed_mode_list = list(speed_settings.keys())
    if cruise_speed_mode not in speed_mode_list:
        return 'FAILED: selected mode not in list'

    for s in speed_mode_list:
        cruise_speed = speed_settings[s]['cruise']
        speed_settings[s]['spin_up_angle'] = cruise_speed * (cruise_speed + 1) * speed_settings[s][
            'spinup'] / 2 / 10 / 337.358433
        speed_settings[s]['spin_down_angle'] = cruise_speed * (cruise_speed + 1) * speed_settings[s][
            'spindown'] / 2 / 10 / 337.358433
        speed_settings[s]['rpm'] = cruise_speed * 300
    return speed_settings[cruise_speed_mode]


def get_speed_parameters_creep(creep_speed_mode):
    speed_settings = {}
    speed_settings['null'] = {'period': 0, 'step_size': 0}
    speed_settings['default'] = {'period': 2, 'step_size': 1}  # 150
    speed_settings['creep_0'] = {'period': 2, 'step_size': 10}  # 1500
    speed_settings['creep_1'] = {'period': 1, 'step_size': 11}  # 3000
    speed_settings['creep_2'] = {'period': 1, 'step_size': 22}
    speed_settings['creep_3'] = {'period': 1, 'step_size': 33}
    speed_settings['creep_4'] = {'period': 2, 'step_size': 3}  # 450

    speed_mode_list = list(speed_settings.keys())
    if creep_speed_mode not in speed_mode_list:
        return 'FAILED: selected mode not in list'

    for s in speed_mode_list:
        try:
            speed_settings[s]['rpm'] = speed_settings[s]['step_size'] * 300 / speed_settings[s]['period']
        except:
            speed_settings[s]['rpm'] = 0
    return speed_settings[creep_speed_mode]


def movetime(rpm=9900, angle=360):
    # for a move: 0.017778 degree / sec / RPM
    # example:
    # a 90 deg move at default cruise speed (9900 RPM) takes 90 / (0.017778 * 9900) = 0.51 seconds
    return angle / (0.017778 * rpm)


def distance(a, b):
    return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


"""Takes in a list of (x,y) points and computes and returns the standard deviation of those points"""


def point_spread(points):
    stds = np.std(points, axis=0)
    return np.sqrt(stds[0] ** 2 + stds[1] ** 2)


def write_fits(image, filename, save_dir):
    fits.writeto(filename, image)
    if not os.path.isdir(f"./data/{save_dir}"):
        os.mkdir(f"./data/{save_dir}")
    shutil.move(filename, f"./data/{save_dir}")


def write_region(filename, save_dir, xCenSub, yCenSub, FWHMSub):
    f = open(filename, 'w')
    f.write('global color=magenta font="helvetica 13 normal"\n')
    for i in range(len(xCenSub)):
        text = f"circle {xCenSub[i] + 1} {yCenSub[i] + 1} {FWHMSub[i] / 2}\n"
        f.write(text)
    f.close()
    if not os.path.isdir(f"./data/{save_dir}"):
        os.mkdir(f"./data/{save_dir}")
    shutil.move(filename, f"./data/{save_dir}")


def listify(zip_obj):
    list_obj = []
    for obj in zip_obj:
        list_obj.append(list(obj))
    return list_obj
