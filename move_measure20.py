#!/usr/bin/env python3
#
# move_measure.py
#
# pylint: disable=line-too-long, W0703, C0301, C0302, C0103, C0116, R0913, W0702
#
#
# requires FW 6.20 and up
#
#
# Command line example
#
'''
# get access to camera and high level command interface
# this sets up the device(s)
dev_by_bus={'can0':[4733]}
import move_measure as movemeasure
mm=movemeasure.MoveMeasure(dev_by_bus,'STi') # camear is STi or ST8300
# the following gives access to lower level command interface (net required)
import fiposcontrol; fipos = fiposcontrol.FiposControl(['can0'])
#
# read out FW version
fipos.get_fw_version()
#
# change the speed mode for cruise speed
mm.set_speed_mode_cruise(speed_mode_cruise_phi='default', speed_mode_cruise_theta='default')
#
# direct access to measured centroids; nspots is the number of expected (and max number of returned) centroids;
movemeasure.get_centroids(mm.camera,nspots=2,fboxsize=7)
#
# calibrate the phi axis; reads stored data from file ix those exist unless explicitely forced otherwise
mm.calibrate_phi(use_stored_if_available=True)
#
# This moves and mesures a given number of creep steps; here the requested angle is 4 deg.
# The default for nsteps is 1; angle is required argument
mm.creep_step_sequence_phi( 4, nsteps=15)
#
# This moves and mesures a given number of cruise steps; here the requested angle is 20 deg.
# Unlike for the creep method, here the full range is divided into the maximum number of steps
mm.creep_step_sequence_phi( 20)
#
#
###############################################################################################
#
# change log
#
# 0.2   241002  ms  changed teh return of get_centroids so that all centroids are returned. This was limited to
#                   the first only. I can't recall why itwas implemented that way.
# 0.1   220926  ms  started change log; added method to query version; added speed_12 and speed_14 to cruise speeeds;
#                   limited number of correction moves in positioning_sequence_phi
#                   added use_fiducials() and identify_fiducials(centroids) methods
#
# 0.1   221114  ab  porting changes from move_measure.py to this move_measure20.py (not sure why they didn't copy???);
#                   changes include adding lifetime_sequence_phi() function back in (line 1022), and updating move_to_hardstop_phi (line 891)
#
# 0.1   221114  ab  add option for spin up/down angle of 4.023 deg each (2 on the top row of that large spreadsheet)
#
#
#
#
'''

import sys
import os
import ast
import time
import datetime
import pickle
import shutil

from os.path import exists
import argparse

import configobj
from configobj import ConfigObj
from scipy import optimize
import numpy as np

cwd = os.getcwd()
sys.path.append(cwd + '/sbig')
import sbigcam
import multicens
import fiposcontrol20 as fiposcontrol
from astropy.io import fits

VERSION = '0.1'


# TODO: store IDs of the positioners that did not move?
class MoveNone(Exception):
    pass


class MoveError(Exception):
    def __init__(self, moved):
        self.moved = moved


# TODO: change to prewritten library?
def fitcircle(xy):
    # C.f. http://scipy.github.io/old-wiki/pages/Cookbook/Least_Squares_Circle
    """Return a best-fit circle for (x,y) data.
    Input:    xy ... cartesian points of the form [[x1,y1], [x2,y2], ...]
    Returns:  xy_ctr ... best-fit circle center, in the form [x_center,y_center]
              radius ... best-fit circle radius
    """
    x = [pt[0] for pt in xy]
    y = [pt[1] for pt in xy]

    def _calc_R(xc, yc):
        """calculate the distance of each 2D points from the center (xc, yc) """
        return ((x - xc) ** 2 + (y - yc) ** 2) ** 0.5

    def _func(c):
        """calculate the algebraic distance between the data points and the mean circle centered at c=(xc, yc) """
        Ri = _calc_R(*c)
        return Ri - Ri.mean()

    xm = np.mean(x)
    ym = np.mean(y)
    center_estimate = xm, ym
    center, _ = optimize.leastsq(_func, center_estimate)
    xc, yc = center
    Ri = _calc_R(*center)
    R = Ri.mean()
    xy_ctr = [xc, yc]
    return xy_ctr, R


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


class MoveMeasure:
    def __init__(self, dev_by_bus=None, camera_name='ST8300', verbose=False, debug_level=0):
        self.verbose = verbose
        self.camera_name = camera_name
        self.camera = None
        # camera
        self.fipos = fiposcontrol.FiposControl(['can0'], debug_level, verbose)
        self.dev_by_bus = dev_by_bus
        self.arc_calibration_available_phi = False
        self.arc_calibration_available_theta = False
        self.range_calibration_available_phi = False
        self.range_calibration_available_theta = False
        self.microns_per_pixel = 0

        self.number_of_correction_moves = 3
        self.center_phi = None  # tupel with (x,y)
        self.radius_phi = None
        self.range_limits_phi = {'cw': None,
                                 'ccw': None}  # list with ((x,y)_start, (x,y)_end); start is the hardstop that is reached when moving ccw
        self.range_angle_phi = None  # full range angle in degree; the direction from start to end counts positive
        self.gain_phi_cruise = {'cw': 1., 'ccw': 1.}
        self.gain_phi_creep = {'cw': 1., 'ccw': 1.}
        self.gain_theta_cruise = {'cw': 1., 'ccw': 1.}
        self.gain_theta_creep = {'cw': 1., 'ccw': 1.}
        self.backlash_angle = 3.

        self.window = False
        # window values have been set based on the full image size of the ST8300 camera
        self.window_x = [0, 3352]
        self.window_y = [0, 2532]
        self.fid_window_x = [1330, 1400]
        self.fid_window_y = [1660, 1730]
        self.movement_threshold = 0.1
        self.speed_mode_cruise_phi = 'default'
        self.speed_mode_creep_phi = 'default'
        self.speed_mode_cruise_theta = 'default'
        self.speed_mode_creep_theta = 'default'

        self.creep_threshold_theta = 4.
        self.creep_threshold_phi = 4.
        self.exp_time = 20.
        self.target_distance = 0.2

        self.use_fiducials = True
        self.fiducial_center = [0, 0]
        self.fiducial_search_radius = 3  # we look inside a circle of this size around the fiducial centroid
        self.init_camera()
        # self.set_speed_mode_creep(speed_mode_creep_phi='default', speed_mode_creep_theta='default')
        # self.set_speed_mode_cruise(speed_mode_cruise_phi='default', speed_mode_cruise_theta='default')

    @staticmethod
    def get_version():
        return str(VERSION)

    def init_camera(self):
        if self.camera_name in ['STi', 'ST8300']:
            self.camera = sbigcam.SBIGCam()
            self.camera.select_camera(self.camera_name)
            # Time of exposure in between 90ms and 3600000ms
            if not self.camera.open_camera():
                print("Can't establish connection to camera")
                sys.exit()
            self.camera.set_exposure_time(self.exp_time)
        if self.camera_name in ['zwo']:
            import zwoasi as asi
            asi.init('/home/msdos/fvc/asi/ASI_linux_mac_SDK_V1.20.1/lib/x64/libASICamera2.so')
            self.camera = asi.Camera(0)
            self.camera.set_control_value(asi.ASI_EXPOSURE, 5000)
            self.camera.set_image_type(2)

    def close_camera(self):
        if self.camera_name in ['sti']:
            self.camera.close_camera()
        if self.camera_name in ['zwo']:
            self.camera.close_camera()

    def set_camera_scale(self, microns_per_pixel=131):
        self.microns_per_pixel = microns_per_pixel
        print('Camera scale: ' + str(microns_per_pixel) + ' microns per pixel')
        return

    def get_camera_scale(self):
        # self.microns_per_pixel = microns_per_pixel
        print('Camera scale: ' + str(self.microns_per_pixel) + ' microns per pixel')
        return self.microns_per_pixel

    def set_target_distance(self, target_distance):
        self.target_distance = target_distance

    def get_target_distance(self):
        return self.target_distance

    # TODO: remove function?
    def get_all_centroids(self, camera, nspots=1, fboxsize=7):
        image = camera.start_exposure()
        nspots = 1 + len(self.fiducial_reference_centroids)
        # image = image[imx[0]:imx[1],imy[0]:imy[1]]
        xCenSub, yCenSub, peaks, FWHMSub, _ = multicens.multiCens(image, n_centroids_to_keep=nspots, verbose=False,
                                                                  write_fits=False, size_fitbox=fboxsize)
        print(' get centroids >> ' + str(xCenSub[0]) + ' ' + str(yCenSub[0]))
        centroids = zip(xCenSub[0:nspots], yCenSub[0:nspots])
        print('centroids', centroids)
        return centroids

    """Sets a window such that any future calls to get_centroids will only generate a FITS file of the specified
    window, with all coordinates adjusted relative to the window size"""

    # TODO: need to reset window mode between images!!!
    def set_window(self, mode='Full', win_x=None, win_y=None):
        if mode.lower() in ['full'] or not win_x or not win_y:
            self.window = False
            print('Window mode off')
        else:
            self.window = True
            self.window_x[0], self.window_x[1] = win_x
            self.window_y[0], self.window_y[1] = win_y
            print('Window mode on: x_win: ' + str(self.window_x) + '  y_win: ' + str(self.window_y))

    """Sets class attributes for the bounds of the window where the fiducial is"""
    def set_fiducial_window(self, win_x, win_y):
        self.fid_window_x = win_x
        self.fid_window_y = win_y

    """Returns true if the point p is inside the bounds of the window where the fiducial is"""

    def in_fiducial_window(self, p):
        if self.fid_window_x[0] < p[0] < self.fid_window_x[1] and self.fid_window_y[0] < p[1] < self.fid_window_y[1]:
            return True
        else:
            return False

    """Given the window where the fiducial centroids are, returns the centroids"""

    def get_fiducial_centroids(self, write_fits_file=False, write_region_file=False, write_conf_file=False
                               , save_full=False, dir_name=None, nspots=4, fboxsize=7):
        image = self.camera.start_exposure()
        crop = image[self.fid_window_y[0]:self.fid_window_y[1], self.fid_window_x[0]:self.fid_window_x[1]]
        date_str, time_str = get_datetime_string()
        directory = './data'
        if dir_name:
            directory = f"./data/{dir_name}"
        if write_fits_file:
            # save full FITS file
            if save_full:
                outfile = f"{date_str}_{time_str}_full.fits"
                fits.writeto(outfile, image)
                shutil.move(outfile, directory)
            # save cropped FITS file (just the window)
            outfile = f"{date_str}_{time_str}_crop.fits"
            fits.writeto(outfile, crop)
            shutil.move(outfile, directory)

        xCenSub, yCenSub, peaks, FWHMSub, _ = multicens.multiCens(crop, n_centroids_to_keep=nspots, verbose=False,
                                                                  write_fits=False, size_fitbox=fboxsize)
        if write_region_file:
            outfile = f"{date_str}_{time_str}_crop.reg"
            f = open(outfile, 'w')
            f.write('global color=magenta font="helvetica 13 normal"\n')
            for i in range(len(xCenSub)):
                text = f"circle {xCenSub[i] + 1} {yCenSub[i] + 1} {FWHMSub[i] / 2}\n"
                f.write(text)
            f.close()
            shutil.move(outfile, directory)

        if write_conf_file:
            config = ConfigObj()
            config.filename = f"{date_str}_{time_str}_fiducials.conf"
            for i in range(len(xCenSub)):
                key = f"fid_{i}"
                config[key] = {}
                config[key]['x_coord'] = xCenSub[i] + self.fid_window_x[0]
                config[key]['y_coord'] = yCenSub[i] + self.fid_window_y[0]
                config[key]['peak'] = peaks[i]
                config[key]['FWHM'] = FWHMSub[i]
            config.write()
            shutil.move(config.filename, directory)

        return list(zip(xCenSub, yCenSub)), peaks, FWHMSub

    @staticmethod
    def unpack_fiducial_conf(self, filename):
        config = ConfigObj(filename)
        fid_list = []
        for fid in config.values():
            fid_list.append([float(fid['x_coord']), float(fid['y_coord'])])
        return sorted(fid_list, key=lambda x: x[0])

    """Takes in list of centroid coordinates of a fiducial and computes the microns to pixel ratio"""

    @staticmethod
    def camera_scale_from_fiducial(fiducials):
        dist_0_1 = distance(fiducials[0], fiducials[1])  # should be 1.2mm
        dist_0_3 = distance(fiducials[0], fiducials[3])  # should be 1.6mm
        dist_1_3 = distance(fiducials[1], fiducials[3])  # should be 2mm
        return np.mean([1200 / dist_0_1, 1600 / dist_0_3, 2000 / dist_1_3])

    """Takes in a list of (x,y) points and computes and returns the standard deviation of those points"""

    @staticmethod
    def point_spread(points):
        stds = np.std(points, axis=0)
        return np.sqrt(stds[0] ** 2 + stds[1] ** 2)

    """Takes repeat number of images of the fiducial, computes, returns, and sets class attributes for the fiducial 
    offset and camera scale"""

    def calibrate_fiducial(self, win_x, win_y, repeat=1, write_files=False):
        self.set_fiducial_window(win_x, win_y)
        date_str, time_str = get_datetime_string()
        dir_name = f"{date_str}_{time_str}_fiducial_calibration"
        if write_files:
            os.makedirs(f"./data/{dir_name}")
        fid_0 = []
        fid_1 = []
        fid_2 = []
        fid_3 = []
        camera_scales = []
        for i in range(repeat):
            centroids, peaks, FWHM = self.get_fiducial_centroids(write_files, write_files, write_files,
                                                                 dir_name=dir_name)
            centroids = sorted(centroids, key=lambda x: x[0])
            camera_scales.append(self.camera_scale_from_fiducial(centroids))
            fid_0.append(centroids[0])
            fid_1.append(centroids[1])
            fid_2.append(centroids[2])
            fid_3.append(centroids[3])
        # perform sanity check to make sure fiducial does not move between images
        threshold = 0.1
        if self.point_spread(fid_0) > threshold or self.point_spread(fid_1) > threshold \
                or self.point_spread(fid_2) > threshold or self.point_spread(fid_3) > threshold:
            print("Fiducials move between images")
            exit(1)
        # set camera scale
        self.microns_per_pixel = np.mean(camera_scales)
        # compute and set fiducial center
        fid_0_center = np.mean(fid_0, axis=0)
        fid_1_center = np.mean(fid_1, axis=0)
        fid_2_center = np.mean(fid_2, axis=0)
        fid_3_center = np.mean(fid_3, axis=0)
        fid_centers = [fid_0_center, fid_1_center, fid_2_center, fid_3_center]
        fid_center = np.mean(fid_centers, axis=0)
        self.fiducial_center = [fid_center[0] + win_x[0], fid_center[1] + win_y[0]]
        return self.fiducial_center, self.microns_per_pixel

    """Takes an image and returns the specified number of centroids within that image"""

    def get_centroids(self, write_fits_file=False, write_region_file=False, write_conf_file=False, nspots=1, fboxsize=7,
                      flip_image=False, save_dir=''):
        image = self.camera.start_exposure()
        if flip_image:
            # horizontal flip (left-right)
            image = image[:, :: -1]
            # vertical flip
            # image = image[: : -1]
        if self.window:
            # remember that in Python it is [y,x]
            image = image[self.window_y[0]:self.window_y[1], self.window_x[0]:self.window_x[1]]

        date_str, time_str = get_datetime_string()
        file_name = 'centroids_zwo_' + date_str + '_' + time_str
        if write_fits_file:
            print('Writing out FITS file')
            outfile = file_name + '.fits'
            fits.writeto(outfile, image)
            # TODO: check if data directory already exists, if not create it
            shutil.move(outfile, f"./data/{save_dir}")

        xCenSub, yCenSub, peaks, FWHMSub, _ = multicens.multiCens(image, n_centroids_to_keep=nspots, verbose=False,
                                                                  write_fits=False, size_fitbox=fboxsize)
        centroids = None
        if len(xCenSub) > 0:
            if self.window:
                xCenSub = [x + self.window_x[0] for x in xCenSub]
                yCenSub = [y + self.window_y[0] for y in yCenSub]
            try:
                centroids = zip(xCenSub, yCenSub)
            except:
                print("WARNING: No Centroids!")

        if write_region_file:
            print('Writing out region file')
            outfile = file_name + '.reg'
            f = open(outfile, 'w')
            f.write('global color=magenta font="helvetica 13 normal"\n')
            for i in range(len(xCenSub)):
                text = f"circle {xCenSub[i] + 1 - self.window_x[0]} {yCenSub[i] + 1 - self.window_y[0]} {FWHMSub[i] / 2}\n"
                f.write(text)
            f.close()
            shutil.move(outfile, f"./data/{save_dir}")

        if write_conf_file:
            config = ConfigObj()
            config.filename = f"{file_name}.conf"
            for i in range(len(xCenSub)):
                key = f"centroid_{i}"
                config[key] = {}
                config[key]['x_coord'] = xCenSub[i] + self.window_x[0]
                config[key]['y_coord'] = yCenSub[i] + self.window_y[0]
                config[key]['peak'] = peaks[i]
                config[key]['FWHM'] = FWHMSub[i]
            config.write()
            shutil.move(config.filename, f"./data/{save_dir}")

        return list(centroids), peaks, FWHMSub

    """Given a 2 lists of positions of positioners, finds which positioner moved/changed positions"""

    def find_moved(self, centroids_before, centroids_after):
        moved = []
        for after in centroids_after:
            match = False
            for before in centroids_before:
                # NO MAGIC NUMBER ???
                if distance(before, after) < self.movement_threshold:
                    match = True
                    break
            if not match:
                moved.append(after)
        if len(moved) == 1 and not self.in_fiducial_window((moved[0])):
            return moved
        elif not moved:
            raise MoveNone()
        else:
            raise MoveError(moved)

    def move_error_handler(self, moved, pos_conf, pos_number, pos_id):
        if len(moved) == 1 and self.in_fiducial_window(moved[0]):
            print(f"No positioners moved but fiducial points {moved} appeared to move")
            return pos_number
        if len(moved) > 1:
            pos_moved = moved
            fid_moved = []
            for move in moved:
                if self.in_fiducial_window(move):
                    pos_moved.remove(move)
                    fid_moved.append(move)
            if len(pos_moved) == 1:
                print(f"Fiducial points {fid_moved} appeared to move")
                return self.set_pos_config(pos_conf, f"positioner_{pos_number}", pos_number, pos_moved, pos_id)
            elif not pos_moved:
                print(f"No positioners moved but fiducial points {fid_moved} moved")
                return pos_number
            else:
                print(f"{pos_moved} moved while attempting to move positioner {pos_id}")
                return pos_number

    @staticmethod
    def set_pos_config(pos_conf, key, pos_number, moved, pos_id):
        pos_conf[key] = {}
        pos_conf[key]['can_id'] = pos_id
        pos_conf[key]['x_coord'] = moved[0][0]
        pos_conf[key]['y_coord'] = moved[0][1]
        return pos_number + 1

    """Matches CAN addresses with positioners, create a conf file with all the positioners and their current 
    positioners"""

    def match_positioners(self, write_fits_file=False, write_region_file=False, nspots=1, fboxsize=7):
        fp = fiposcontrol.FiposControl(['can0'])
        can_command_out = fp.get_can_address()
        pos_ids = can_command_out[2]['can0'].keys()
        date_str, time_str = get_datetime_string()
        dir_name = f"{date_str}_{time_str}_positioner_matching"
        os.makedirs(f"./data/{dir_name}")
        pos_conf = ConfigObj()
        pos_conf.filename = f"positioners.conf"
        pos_number = 0
        # take an image before moving anything
        centroids_before, peaks_before, FWHM_before = self.get_centroids(write_fits_file, write_region_file,
                                                                         False, nspots, fboxsize,
                                                                         save_dir=dir_name)
        for pos_id in pos_ids:
            # TODO: NO MAGIC NUMBERS
            # ID 10000 corresponds to fiducial
            if pos_id != 10000:
                # take image and compute centroids after moving
                to_move = {'can0': {pos_id: pos_id}}
                fp.move_direct(to_move, 'cw', motor='phi')
                centroids_after, peaks_after, FWHM_after = self.get_centroids(write_fits_file, write_region_file,
                                                                              False, nspots, fboxsize,
                                                                              save_dir=dir_name)
                try:
                    moved = self.find_moved(centroids_before, centroids_after)
                    pos_number = self.set_pos_config(pos_conf, f"positioner_{pos_number}", pos_number, moved, pos_id)
                except MoveNone:
                    # try moving other direction
                    try:
                        fp.move_direct(to_move, 'ccw', motor='phi')
                        print('Tried moving other direction!')
                        centroids_after, peaks_after, FWHM_after = self.get_centroids(write_fits_file,
                                                                                      write_region_file,
                                                                                      False, nspots, fboxsize,
                                                                                      save_dir=dir_name)
                        moved = self.find_moved(centroids_before, centroids_after)
                        pos_number = self.set_pos_config(pos_conf, f"positioner_{pos_number}", pos_number, moved,
                                                         pos_id)
                        print("Moved other direction!")
                    except MoveNone:
                        print(f"Positioner {pos_id} did not move")
                    except MoveError as error:
                        pos_number = self.move_error_handler(error.moved, pos_conf, pos_number, pos_id)
                except MoveError as error:
                    pos_number = self.move_error_handler(error.moved, pos_conf, pos_number, pos_id)
                centroids_before = centroids_after
        pos_conf.write()
        shutil.move(pos_conf.filename, f"./data/{dir_name}")

    def get_pos_centroids(self, centroids):
        # image = camera.start_exposure()
        # nspots = 1 + len(self.fiducial_reference_centroids)
        # image = image[imx[0]:imx[1],imy[0]:imy[1]]
        # xCenSub, yCenSub, peaks, FWHMSub, _ = multicens.multiCens(image, n_centroids_to_keep=nspots, verbose=False, write_fits=False, size_fitbox=fboxsize)
        # del peaks
        # del FWHMSub
        # print(xCenSub)
        # print(yCenSub)
        # print(' get centroids >> ' + str(xCenSub[0]) + ' ' + str(yCenSub[0]))
        # centroids = zip(xCenSub[0:nspots], yCenSub[0:nspots])

        # now split into fid centroids and pos centroids
        #

        # now correct the positioner centroid(s)
        # correction is based on the fiducial_reference_centroids

        return centroids

    """Reads in data from config file about location of fiducial centroids if it exists"""

    def get_fiducial_reference(self):
        # fiducial centroids ae passed as a dictionary:
        # centroids = {'fid_1':(x,y),'fid_2':(x,y)}

        if self.fiducial_reference_centroids:
            self.use_fiducials = True
            return self.fiducial_reference_centroids

        # read fiducial_reference_centroids from file
        # file name is fiducial_reference.conf
        fid_file = 'fiducial_reference.conf'
        fid_file_exists = exists(fid_file)
        if not fid_file_exists:
            self.use_fiducials = False
            self.fiducial_reference_centroids = None
            return 'No fiducial reference file found'
        fid_data = ConfigObj(fid_file, unrepr=True)
        for fid in fid_data:
            self.fiducial_reference_centroids[fid] = fid_data[fid]
        self.use_fiducials = True
        return self.fiducial_reference_centroids

    """Given the list of centroids and the location of the fiducial centroids, returns the positioner centroids as a
    list and the fiducial centroids as a dictionary"""

    def identify_fiducials(self, centroids):
        # separates the centroids in pos_centroids and fid_centroids
        #
        # returned are pos_centroids (list) and fid_centroids (dict - {'fid_1':(x,y),'fid_2':(x,y) etc.})
        #
        pos_centroids = []
        fid_centroids = {}
        if not self.use_fiducials:
            pos_centroids = centroids
            return pos_centroids, fid_centroids

        if not self.fiducial_reference_centroids:
            return 'fiducial reference centroids required'

        for fid_id, fid_cen in self.fiducial_reference_centroids.items():
            for cen in centroids:
                if distance(cen, fid_cen) > self.fiducial_search_radius:
                    pos_centroids.append(cen)
                else:
                    fid_centroids[fid_id] = cen
        return pos_centroids, fid_centroids

    """Calculates the center of mass of the fiducial centroids and how much it moved, then adds that quantity to the
    coordinates of all the positioners"""

    def correct_centroids(self, pos_centroids, fid_centroids):
        # always returns a list
        # calculate correction
        d_x = []
        d_y = []
        for fid_id in list(fid_centroids.keys()):
            d_x.append(self.fiducial_reference_centroids[fid_id][0] - fid_centroids[fid_id][0])
            d_y.append(self.fiducial_reference_centroids[fid_id][1] - fid_centroids[fid_id][1])
        dx = np.mean(d_x)
        dy = np.mean(d_y)

        pos_centroids_corrected = [(x + dx, y + dy) for x, y in pos_centroids]
        return pos_centroids_corrected

    def get_speed_mode(self, motor):

        if motor in ['phi']:
            cruise_speed = self.speed_mode_cruise_phi
            creep_speed = self.speed_mode_creep_phi
        elif motor in ['theta']:
            cruise_speed = self.speed_mode_cruise_theta
            creep_speed = self.speed_mode_creep_theta
        else:
            return 'FAILED: invalid motor'
        return cruise_speed, creep_speed

    def set_gain(self, motor, cruise_gain={'cw': 1., 'ccw': 1.}, creep_gain={'cw': 1., 'ccw': 1.}):
        if motor in ['phi']:
            self.gain_phi_cruise = cruise_gain
            self.gain_phi_creep = creep_gain
        elif motor in ['theta']:
            self.gain_theta_cruise = cruise_gain
            self.gain_theta_creep = creep_gain
        else:
            return 'FAILED: invalid motor'
        return 'SUCCESS'

    def get_gain(self, motor):

        if motor in ['phi']:
            cruise_gain = self.gain_phi_cruise
            creep_gain = self.gain_phi_creep
        elif motor in ['theta']:
            cruise_gain = self.gain_theta_cruise
            creep_gain = self.gain_theta_creep
        else:
            return 'FAILED: invalid motor'
        return cruise_gain, creep_gain

    def get_speed_mode_cruise(self):
        return 'not implemented'

    def set_speed_mode_cruise(self, speed_mode_cruise_phi='default', speed_mode_cruise_theta='default'):

        self.speed_mode_cruise_phi = speed_mode_cruise_phi

        if 'hspeed' in self.speed_mode_cruise_phi:
            print(' Command 81 speed setting!!')
            return self.set_speed_mode_cruise_phi_henry(speed_mode_cruise_phi)

        self.speed_mode_cruise_theta = speed_mode_cruise_theta

        if self.verbose:
            print(
                ' set speed mode cruise phi, theta: ' + str(speed_mode_cruise_phi) + ', ' + str(speed_mode_cruise_phi))

        speed_setting_phi = get_speed_parameters_cruise(speed_mode_cruise_phi)
        speed_setting_theta = get_speed_parameters_cruise(speed_mode_cruise_theta)

        self.creep_threshold_phi = speed_setting_phi['spin_up_angle'] + speed_setting_phi['spin_down_angle']
        self.creep_threshold_theta = speed_setting_theta['spin_up_angle'] + speed_setting_theta['spin_down_angle']

        # first setting the ramps

        ramps = {'theta': {'cw': {}, 'ccw': {}}, 'phi': {'cw': {}, 'ccw': {}}}
        ramps['theta']['cw'] = {'up': speed_setting_theta['spinup'], 'down': speed_setting_theta['spindown']}
        ramps['theta']['ccw'] = {'up': speed_setting_theta['spinup'], 'down': speed_setting_theta['spindown']}
        ramps['phi']['cw'] = {'up': speed_setting_phi['spinup'], 'down': speed_setting_phi['spindown']}
        ramps['phi']['ccw'] = {'up': speed_setting_phi['spinup'], 'down': speed_setting_phi['spindown']}

        self.fipos.set_spin_ramps(None, ramps)
        time.sleep(0.1)

        # setting the cruise and creep steps

        # steps = {'theta':{'cw':{'cruise':33, 'creep':1}, 'ccw':{'cruise':33, 'creep':1}}, 'phi':{'cw':{'cruise':33, 'creep':1}, 'ccw':{'cruise':33, 'creep':1}}}
        steps = {'theta': {'cw': {}, 'ccw': {}}, 'phi': {'cw': {}, 'ccw': {}}}

        steps['theta']['cw'] = {'cruise': speed_setting_theta['cruise'], 'creep': 0}
        steps['theta']['ccw'] = {'cruise': speed_setting_theta['cruise'], 'creep': 0}
        steps['phi']['cw'] = {'cruise': speed_setting_phi['cruise'], 'creep': 0}
        steps['phi']['ccw'] = {'cruise': speed_setting_phi['cruise'], 'creep': 0}

        self.fipos.set_cruise_creep_speed(None, steps)
        time.sleep(0.1)
        return (speed_setting_phi, speed_setting_theta)

    def set_speed_mode_creep(self, speed_mode_creep_phi='default', speed_mode_creep_theta='default'):
        # verbose = self.verbose

        self.speed_mode_creep_phi = speed_mode_creep_phi
        self.speed_mode_creep_theta = speed_mode_creep_theta

        if self.verbose:
            print(' set speed mode creep phi, theta: ' + str(speed_mode_creep_phi) + ' , ' + str(speed_mode_creep_phi))

        speed_setting_phi = get_speed_parameters_creep(speed_mode_creep_phi)
        speed_setting_theta = get_speed_parameters_creep(speed_mode_creep_theta)

        steps = {'theta': {'cw': {}, 'ccw': {}}, 'phi': {'cw': {}, 'ccw': {}}}

        steps['theta']['cw'] = {'cruise': 0, 'creep': speed_setting_theta['step_size']}
        steps['theta']['ccw'] = {'cruise': 0, 'creep': speed_setting_theta['step_size']}
        steps['phi']['cw'] = {'cruise': 0, 'creep': speed_setting_phi['step_size']}
        steps['phi']['ccw'] = {'cruise': 0, 'creep': speed_setting_phi['step_size']}

        retval = self.fipos.set_cruise_creep_speed(None, steps)
        if retval not in 'SUCCESS':
            return retval

        periods = {'theta': speed_setting_theta['period'], 'phi': speed_setting_phi['period']}

        retval = self.fipos.set_creep_periods(None, periods)

        if retval not in 'SUCCESS':
            return retval
        time.sleep(0.1)
        return (speed_setting_phi, speed_setting_theta)

    def move_direct_cruise(self, direction, motor, angle_req, gain=1.):
        # cruise speed is 176 deg/sec
        # creep speed is 2.7 deg/sec
        # set the correct speed
        #
        # angle_req: this is the true angle we want to move
        # angle_cmd: this is the creep angle we send in command 29

        if direction not in ['cw', 'ccw']:
            return 'Invalid Direction'
        if motor not in ['theta', 'phi']:
            return 'Invalid Motor'

        speed_setting_phi = get_speed_parameters_cruise(self.speed_mode_cruise_phi)
        speed_setting_theta = get_speed_parameters_cruise(self.speed_mode_cruise_theta)

        # speed_setting_phi, speed_setting_theta = self.set_speed_mode_cruise(speed_mode_cruise, speed_mode_cruise)
        rpm_default = 9900

        speed_setting = speed_setting_phi if motor in ['phi'] else speed_setting_theta
        rpm_requested = speed_setting['rpm']
        spin_up_angle = speed_setting['spin_up_angle']
        spin_down_angle = speed_setting['spin_down_angle']

        cruise_scale = rpm_default / rpm_requested
        # angle_req = ((angle_cmd / cruise_scale) + spin_up_angle + spin_down_angle) / gain
        angle_cmd = (angle_req / gain - spin_up_angle - spin_down_angle) * cruise_scale

        if self.verbose:
            print(' move direct cruise, req. angle, cmd angle: ', angle_req, angle_cmd)
        self.fipos.move(self.dev_by_bus, direction, 'cruise', motor, angle_cmd)
        time.sleep(0.1)
        self.fipos.execute_movetable(self.dev_by_bus)
        sleeptime = max(1. + movetime(rpm_requested, angle_req), 1.)
        print('move direct cruise ', angle_req, angle_cmd, sleeptime)
        time.sleep(sleeptime)
        return 'SUCCESS'

    def move_direct_cruise2(self, direction_phi, direction_theta, angle_phi, angle_theta, gain_phi=1., gain_theta=1.):
        # cruise speed is 176 deg/sec
        # creep speed is 2.7 deg/sec
        # set the correct speed
        #
        # angle_req: this is the true angle we want to move
        # angle_cmd: this is the creep angle we send in command 29

        if direction_theta not in ['cw', 'ccw']:
            return 'Invalid Direction'
        if direction_phi not in ['cw', 'ccw']:
            return 'Invalid Direction'

        speed_setting_phi = get_speed_parameters_cruise(self.speed_mode_cruise_phi)
        speed_setting_theta = get_speed_parameters_cruise(self.speed_mode_cruise_theta)

        # speed_setting_phi, speed_setting_theta = self.set_speed_mode_cruise(speed_mode_cruise, speed_mode_cruise)
        rpm_default = 9900

        for motor in ['phi', 'theta']:
            speed_setting = speed_setting_phi if motor in ['phi'] else speed_setting_theta
            rpm_requested = speed_setting['rpm']
            spin_up_angle = speed_setting['spin_up_angle']
            spin_down_angle = speed_setting['spin_down_angle']

            cruise_scale = rpm_default / rpm_requested
            # angle_req = ((angle_cmd / cruise_scale) + spin_up_angle + spin_down_angle) / gain
            angle_cmd = (angle_req / gain - spin_up_angle - spin_down_angle) * cruise_scale

        if self.verbose:
            print(motor + ' move direct cruise, req. angle, cmd angle: ', angle_req, angle_cmd)
        self.fipos.move(self.dev_by_bus, direction, 'cruise', motor, angle_cmd)
        time.sleep(0.1)
        self.fipos.execute_movetable(self.dev_by_bus)
        sleeptime = max(1. + movetime(rpm_requested, angle_req), 1.)
        print('move direct cruise ', angle_req, angle_cmd, sleeptime)
        time.sleep(sleeptime)
        return 'SUCCESS'

    def move_direct_cruise_test(self, direction, motor, angle_req, gain=1.):
        # cruise speed is 176 deg/sec
        # creep speed is 2.7 deg/sec
        # set the correct speed
        #
        # angle_req: this is the true angle we want to move
        # angle_cmd: this is the creep angle we send in command 29

        if direction not in ['cw', 'ccw']:
            return 'Invalid Direction'
        if motor not in ['theta', 'phi']:
            return 'Invalid Motor'

        speed_setting_phi = get_speed_parameters_cruise(self.speed_mode_cruise_phi)
        speed_setting_theta = get_speed_parameters_cruise(self.speed_mode_cruise_theta)

        # speed_setting_phi, speed_setting_theta = self.set_speed_mode_cruise(speed_mode_cruise, speed_mode_cruise)
        rpm_default = 9900

        speed_setting = speed_setting_phi if motor in ['phi'] else speed_setting_theta
        rpm_requested = speed_setting['rpm']
        spin_up_angle = speed_setting['spin_up_angle']
        spin_down_angle = speed_setting['spin_down_angle']

        cruise_scale = rpm_default / rpm_requested
        # angle_req = ((angle_cmd / cruise_scale) + spin_up_angle + spin_down_angle) / gain
        angle_cmd = (angle_req / gain - spin_up_angle - spin_down_angle) * cruise_scale

        # if self.verbose:
        print(' move direct cruise, req. angle, cmd angle: ', angle_req, angle_cmd)
        self.fipos.move(self.dev_by_bus, direction, 'cruise', motor, angle_cmd)
        time.sleep(0.1)

        x = self.fipos.get_checksum()
        print('checksum ', x)
        self.fipos.clear_movetable()
        x = self.fipos.get_checksum()
        print('checksum ', x)
        self.fipos.execute_movetable(self.dev_by_bus)
        sleeptime = max(1. + movetime(rpm_requested, angle_req), 1.)
        print('move direct cruise ', angle_req, angle_cmd, sleeptime)
        time.sleep(sleeptime)
        return 'SUCCESS'

    def move_direct_creep(self, direction, motor, angle_req, gain=1.):
        # cruise speed is 176 deg/sec
        # creep speed is 2.7 deg/sec
        # set the correct speed
        #
        # angle_req: this is the true angle we want to move
        # angle_cmd: this is the creep angle we send in command 29

        if direction not in ['cw', 'ccw']:
            return 'Invalid Direction'
        if motor not in ['theta', 'phi']:
            return 'Invalid Motor'

        speed_setting_phi = get_speed_parameters_creep(self.speed_mode_creep_phi)
        speed_setting_theta = get_speed_parameters_creep(self.speed_mode_creep_theta)

        # (speed_setting_phi, speed_setting_theta) = self.set_speed_mode_creep(speed_mode_creep, speed_mode_creep)
        rpm_default = 150

        speed_setting = speed_setting_phi if motor in ['phi'] else speed_setting_theta
        rpm_requested = speed_setting['rpm']

        cruise_scale = rpm_default / rpm_requested
        angle_cmd = (angle_req * gain) * cruise_scale

        if self.verbose:
            print(' move direct creep, req. angle, cmd angle: ', angle_req, angle_cmd)

        self.fipos.move(self.dev_by_bus, direction, 'creep', motor, angle_cmd)
        time.sleep(0.1)
        self.fipos.execute_movetable(self.dev_by_bus)
        sleeptime = max(0.5 + movetime(rpm_requested, angle_req), 1.)
        print('move direct creep ', angle_req, angle_cmd, sleeptime)
        time.sleep(sleeptime)
        return 'SUCCESS'

    def measure_range_phi_old(self):
        # fipos = self.fipos
        # verbose = self.verbose
        # dev_by_bus = self.dev_by_bus
        # camera = self.camera
        # expect that the phi arm has been moved ccw to the hardstop
        #
        # move gently against the ccw hardstop. This should be done
        # as a creep but for now we are cruising
        #
        # if we pass a center as argument then overwrite the one stored in self.center

        if not self.center_phi:
            return 'need center'
        # gently move into the hardstop
        self.move_direct_cruise('ccw', 'phi', 10, 1.)
        #
        centroids = []
        # centroid against the ccw hardstop
        #
        centroids.append(self.get_centroids())
        #
        # move 180 deg ccw; this is less than full range
        self.move_direct_cruise('cw', 'phi', 180, 1.)
        last_centroid = self.get_centroids()
        centroids.append(last_centroid)
        #
        # -range_90 = angle_between(center,centroids[0],last_centroid)
        # -move_direct_cruise(fipos, dev_by_bus, 'cw', speed_mode_cruise, 'phi', 90, 1.)
        # -last_centroid = get_centroids(camera)
        # -centroids.append(last_centroid)
        #
        # iteratively move in small steps until the hardstop is reached
        # this should be CREEP but for now we cruise
        n_try = 10
        angle_to_hardstop = 180.
        while n_try > 0 and angle_to_hardstop > 0.5:
            self.move_direct_cruise('cw', 'phi', 5, 1.)
            centroid = self.get_centroids()
            print(self.center_phi, centroid, last_centroid)
            angle_to_hardstop = angle_between(self.center_phi, centroid, last_centroid)
            last_centroid = centroid
            centroids.append(last_centroid)
        full_range = get_angle('cw', self.center_phi, centroids[0], centroids[-1])
        print(' range: ' + str(round2(full_range)))
        self.range_angle_phi = full_range
        # self.range_limits_phi = (centroids[0], centroids[-1])

        self.range_calibration_available_phi = True

        return full_range, (centroids[0], centroids[-1])

    def measure_range_phi(self, speed='creep', angle_step=3):
        # fipos = self.fipos
        # verbose = self.verbose
        # dev_by_bus = self.dev_by_bus
        # camera = self.camera
        #
        # this should work for linear phi's not knowing where phi arm starts out from
        #
        #
        # move gently against the ccw hardstop. This should be done
        # as a creep but for now we are cruising
        #
        # if we pass a center as argument then overwrite the one stored in self.center
        #
        # start moving ccw

        centroids = []
        if not self.center_phi:
            return 'need center'
        # gently move into the ccw hardstop
        lin_distance = self.move_to_hardstop_simple('ccw')
        step, last_centroid = self.search_hardstop_phi('ccw', speed, angle_step)

        if step < 1:
            print(' ccw hardstop not found. trying again')
            # try again
            step, last_centroid = self.search_hardstop_phi('ccw', speed, angle_step)
        if step < 1:
            # give up
            print('ccw hardstop not found. giving up!!!')
        centroids.append(last_centroid)

        self.move_direct_cruise('cw', 'phi', 180, 1.)
        lin_distance = self.move_to_hardstop_simple('cw')

        step, last_centroid = self.search_hardstop_phi('cw', speed, angle_step)
        if step < 1:
            print(' cw hardstop not found. trying again')
            # try again
            step, last_centroid = self.search_hardstop_phi('cw', speed, angle_step)
            # give up
            print('cw hardstop not found. giving up!!!')
        centroids.append(last_centroid)

        #    centroids.append(last_centroid)
        full_range = calculate_angle(self.center_phi, centroids[0], centroids[1])
        print(' range: ' + str(round2(full_range)))
        self.range_angle_phi = full_range
        self.range_limits_phi = {'ccw': centroids[0], 'cw': centroids[-1]}
        self.range_calibration_available_phi = True

        return full_range

    def quick_calibrate_phi(self, direction='cw', return_to_hardstop=True):
        print(' quick calibration, direction = ' + direction)
        calibration_centroids = []

        cal_angle = 15.  # for angle in [5, 10, 15]:

        for i in range(8):
            print(i, direction, cal_angle)
            self.move_direct_cruise(direction, 'phi', cal_angle, 1.)
            calibration_centroids.append(self.get_centroids())

        center, radius = fitcircle(np.array(calibration_centroids))

        print('center: (' + str(round2(center[0])) + ', ' + str(round2(center[1])) + ')')
        print('radius: ' + str(round2(radius)))

        measured_angles = []
        for move, p1 in enumerate(calibration_centroids[:-1]):
            measured_angles.append(angle_between(center, p1, calibration_centroids[move + 1]))

        print('measured angles: ', measured_angles)
        self.center_phi = center
        self.radius_phi = radius
        if return_to_hardstop:
            self.move_direct_cruise(reverse_direction(direction), cal_angle * 8, 1.)
        gain = np.mean(measured_angles) / cal_angle
        print('gain: ' + str(gain))
        return center, radius, calibration_centroids, measured_angles, gain

    def search_hardstop_phi(self, direction, speed='creep', angle_step=3):
        # move gently by 3 deg creeps in the given direction and  look for hs
        # we do this by simply calculating the distance between points assuming
        # a rough scale.
        # We get the scale by first moving both directions and taking the larger of the two
        # values as the free direction. Then the scale is given scale
        #
        # scale at bench with STi: 131 microns/pixel
        # 1 deg is always about 52 microns
        #
        # So for a step of 3 degrees we expect 1.2 pixel

        microns_per_pixel = self.microns_per_pixel
        microns_per_degree = 52
        pixel_per_degree = microns_per_degree / microns_per_pixel

        if direction not in ['cw', 'ccw']:
            return 'ERROR: invalid direction'
        search_step = angle_step  # degrees

        has_moved_threshold = search_step * pixel_per_degree / 2.
        # default creep speed (150 RPM) is 2.7 sec per degree. It can take up to 75 seconds to
        # find the hardstop (63 3-deg moves)
        max_number_search_moves = 65
        #
        step = max_number_search_moves
        last_centroid = self.get_centroids()
        gain = self.gain_phi_creep[direction]
        while step > 0:
            print('step', step)
            if speed in ['cruise']:
                self.move_direct_cruise(direction, 'phi', search_step, gain)
            else:
                self.move_direct_creep(direction, 'phi', search_step, gain)
            # self.fipos.move_direct(None, direction, speed, 'phi', search_step)
            centroid = self.get_centroids()
            if distance(centroid, last_centroid) < has_moved_threshold:
                break
            step -= 1
            last_centroid = centroid
        # move another step into the hs
        if step > 0:
            if speed in ['cruise']:
                self.move_direct_cruise(direction, 'phi', search_step, gain)
            else:
                self.move_direct_creep(direction, 'phi', search_step, gain)
            last_centroid = self.get_centroids()
        return step, last_centroid

    def calibrate_phi(self, use_stored_if_available=True):
        write_conf_file = False
        positioner_id = list(self.dev_by_bus.values())[0][0]
        calfile = 'M0' + str(positioner_id) + '_cal.conf'
        cal_data = None
        if use_stored_if_available:
            calfile_exists = exists(calfile)
            print('calfile exists', calfile_exists)
            if calfile_exists:
                cal_data = ConfigObj(calfile, unrepr=True)
                cal_measured = 0
                if 'center_phi' in cal_data:
                    self.center_phi = cal_data['center_phi']
                    cal_measured += 1
                if 'radius_phi' in cal_data:
                    self.radius_phi = cal_data['radius_phi']
                    cal_measured += 1
                if cal_measured > 1:
                    self.arc_calibration_available_phi = True

                range_measured = 0
                if 'range_limits_phi' in cal_data:
                    self.range_limits_phi = (ast.literal_eval(cal_data['range_limits_phi'][0]),
                                             ast.literal_eval(cal_data['range_limits_phi'][1]))
                    range_measured += 1
                if 'range_angle_phi' in cal_data:
                    self.range_angle_phi = cal_data['range_angle_phi']
                    range_measured += 1
                if range_measured > 1:
                    self.range_calibration_available_phi = True

        if not self.arc_calibration_available_phi or not use_stored_if_available:
            center, radius, calibration_centroids, cali_angles = self.calibrate_arc_phi()
            print(' arc calibration ')
            print(center, radius, calibration_centroids, cali_angles)
            if not cal_data:
                cal_data = ConfigObj()
            cal_data['center_phi'] = center
            cal_data['radius_phi'] = radius
            self.fipos.move_direct(None, 'ccw', 'cruise', 'phi', 180)
            write_conf_file = True
        if not self.range_calibration_available_phi or not use_stored_if_available:
            range_angle, range_centroids = self.measure_range_phi()
            print(' range calibration ')
            print(range_angle, range_centroids)
            if not cal_data:
                cal_data = ConfigObj()
            cal_data['range_limits_phi'] = self.range_limits_phi
            cal_data['range_angle_phi'] = self.range_angle_phi
            write_conf_file = True

        if write_conf_file:
            cal_data.filename = calfile
            cal_data.write()

    def calibrate_arc_phi(self):
        # fipos = self.fipos
        # verbose = self.verbose
        # dev_by_bus = self.dev_by_bus
        # camera = self.camera
        # expect that the phi arm has been moved ccw to the hardstop
        #
        #
        print(' calibration')
        calibration_centroids = []
        # move off the hardstop by 5 deg
        self.move_direct_cruise('cw', 'phi', 5, 1.)
        #
        calibration_centroids.append(self.get_centroids())

        for i in range(8):
            if self.verbose:
                print(' calibration loop: ', i)
            self.move_direct_cruise('cw', 'phi', 20, 1.)
            calibration_centroids.append(self.get_centroids())

        center, radius = fitcircle(np.array(calibration_centroids))

        print('center: (' + str(round2(center[0])) + ', ' + str(round2(center[1])) + ')')
        print('radius: ' + str(round2(radius)))

        cali_angles = []
        for move, p1 in enumerate(calibration_centroids[:-1]):
            cali_angles.append(angle_between(center, p1, calibration_centroids[move + 1]))

        print('calibration angles: ', cali_angles)
        self.center_phi = center
        self.radius_phi = radius

        self.arc_calibration_available_phi = True

        return center, radius, calibration_centroids, cali_angles

    def remove_backlash_phi(self, direction):
        # direction is the direction we are going to move next

        retval = self.move_direct_creep(direction, 'phi', self.backlash_angle)

        return retval

    def move_to_hardstop_simple(self, direction, angle=30.):
        my_position = self.get_centroids()
        lin_dist = 1.
        n = 30
        while lin_dist > .99 and n > 0:
            self.move_direct_cruise(direction, 'phi', angle)
            cen = self.get_centroids()
            lin_dist = distance(my_position, cen)
            print('lin dist: ', lin_dist)
            my_position = cen
            # print(my_position, cen)
            n -= 1
        return lin_dist

    def move_to_hardstop_phi_old(self, direction='ccw'):

        # fipos = self.fipos
        # verbose = self.verbose
        # dev_by_bus = self.dev_by_bus
        # camera = self.camera

        # speed_mode_cruise = self.speed_mode_cruise_phi
        # speed_mode_creep = self.speed_mode_creep_phi

        overshoot = 1  # this is the nagle which we overshoot by on purpose when moving into the hardstop

        # move to hardstop; set default speed

        print(f' move to {direction} hardstop')

        range_phi = self.range_limits_phi

        # estimate position
        my_position = self.get_centroids()

        if direction in ['ccw']:
            hardstop = range_phi[0]
        if direction in ['cw']:
            hardstop = range_phi[1]

        angle_distance_to_hardstop = get_angle(direction, self.center_phi, my_position, hardstop)

        #         angle_distance_to_hardstop += overshoot

        #         if angle_distance_to_hardstop > 5:
        #             self.move_direct_cruise(direction, 'phi', angle_distance_to_hardstop - 5)
        #             angle_distance_to_hardstop = 5
        #         self.move_direct_creep(direction, 'phi', angle_distance_to_hardstop)

        #         self.remove_backlash_phi(reverse_direction(direction))
        print('distance to hardstop is:', angle_distance_to_hardstop)
        pars = get_speed_parameters_cruise(self.speed_mode_cruise_phi)
        ramp = pars['spin_up_angle'] + pars['spin_down_angle']
        # get the minimum angle as an integer and then add 1 deg to it, but set it so that the minimum target angle will be 5 if the ramp is smaller than 5 deg
        min_angle = max(int(ramp) + 1, 5)
        #         print('minimum angle due to ramp:', min_angle)
        num_moves = 0
        # take multiple moves if needed to get to the hardstop, stop after 5moves
        while angle_distance_to_hardstop > min_angle and num_moves < 5:
            self.move_direct_cruise(direction, 'phi', angle_distance_to_hardstop)
            new_position = self.get_centroids()
            new_angle = get_angle(direction, self.center_phi, new_position, hardstop)
            print('the angle moved is', angle_distance_to_hardstop - new_angle)
            angle_distance_to_hardstop = new_angle
            print('new dist to hardstop is', new_angle)
            if new_angle < 5:
                break
            num_moves += 1
            time.sleep(0.1)
        # creep once when you're within 5 deg
        #         self.move_direct_creep(direction, 'phi', angle_distance_to_hardstop)

        return 'SUCCESS'

    def cruise_step_sequence_phi(self, angle, nsteps=None, gain={'cw': 1., 'ccw': 1.}, backlash=False):
        # camera = self.camera
        # this is a sequence for a single angle. the range is divided in nsteps
        #
        # this requires calibration and range. Return with FAILED if neither is available
        if not self.range_calibration_available_phi and self.arc_calibration_available_phi:
            return 'FAILED: requires range and calibration measurements'

        full_range = self.range_angle_phi - self.backlash_angle  # account for backlash
        nsteps_max = np.int(full_range / angle)
        if nsteps:
            nsteps = min(nsteps_max, nsteps)
        else:
            nsteps = nsteps_max

        centroids = {'cw': [], 'ccw': []}
        angles = {'cw': [], 'ccw': []}
        # loop through the sequence (cw and ccw)

        for direction in ['cw', 'ccw']:
            # first move ccw to hardstop
            steps = nsteps
            self.move_to_hardstop_phi(reverse_direction(direction))
            self.search_hardstop_phi(reverse_direction(direction), 'cruise', 15)
            centroids[direction] = []
            last_centroid = self.get_centroids()
            centroids[direction].append(last_centroid)
            while steps > 0:
                print('direction', direction, 'step ', steps)
                self.move_direct_cruise(direction, 'phi', angle)
                centroid = self.get_centroids()
                angles[direction].append(get_angle(direction, self.center_phi, last_centroid, centroid))
                last_centroid = centroid
                centroids[direction].append(centroid)
                steps -= 1
        return centroids, angles

    def creep_step_sequence_phi(self, angle, nsteps=1):
        # camera = self.camera
        # this is a sequence for a single angle. the range is divided in nsteps
        #
        if not self.range_calibration_available_phi and self.arc_calibration_available_phi:
            return 'FAILED: requires range and calibration measurements'

        full_range = self.range_angle_phi - self.backlash_angle  # account for backlash
        nsteps_max = np.int(full_range / angle)
        if nsteps:
            nsteps = min(nsteps_max, nsteps)
        else:
            nsteps = nsteps_max

        centroids = {'cw': [], 'ccw': []}
        angles = {'cw': [], 'ccw': []}

        # loop through the sequence (cw and ccw)

        for direction in ['cw', 'ccw']:
            # first move ccw to hardstop
            steps = nsteps
            self.move_to_hardstop_phi(reverse_direction(direction))
            self.search_hardstop_phi(reverse_direction(direction), 'cruise', 15)
            centroids[direction] = []
            last_centroid = self.get_centroids()
            centroids[direction].append(last_centroid)
            while steps > 0:
                print('direction', direction, 'step ', steps)
                self.move_direct_creep(direction, 'phi', angle)
                centroid = self.get_centroids()
                angles[direction].append(get_angle(direction, self.center_phi, last_centroid, centroid))
                last_centroid = centroid
                centroids[direction].append(centroid)
                steps -= 1
        return centroids, angles

    def lifetime_sequence_phi(self, direction, angle, nsteps=1, gain=1., hardstop=False):
        # camera = self.camera
        # this is a sequence for a single angle with only one move
        #
        # this requires calibration and range. Return with FAILED if neither is available
        if not self.range_calibration_available_phi and self.arc_calibration_available_phi:
            return 'FAILED: requires range and calibration measurements'

        steps = nsteps

        centroids = np.zeros((2, 2))
        # move to hardstop if desired
        if hardstop == True:
            self.move_to_hardstop_phi(reverse_direction(direction))

        # self.search_hardstop_phi(reverse_direction(direction),'cruise', 15)
        # setup list to store start and end centroid, only one angle, so don't need to store that
        #         centroids = []

        last_centroid = self.get_centroids()
        # centroids.append(last_centroid)
        centroids[0] = last_centroid  # (x,y)

        # move the positioner and append the final point
        print(angle, gain)
        self.move_direct_cruise(direction, 'phi', angle, gain=gain)
        centroid = self.get_centroids()
        # centroids.append(centroid)
        centroids[1] = centroid  # (x,y)

        # get the angle and save it
        new_angle = get_angle(direction, self.center_phi, last_centroid, centroid)

        return centroids, new_angle

    def positioning_sequence_phi(self, angle):
        #
        # move is either a cruise move or a creep move depending on the angle to move
        # angle is the targeted angle
        # angle_to_target is the angular distance to target
        # angle_measured is the measured angle; angle_to_target = angle - angle_measured
        #
        # camera = self.camera
        # this is a sequence for a single angle. the range is divided in nsteps
        #
        # this requires calibration and range. Return with FAILED if neither is available
        #
        #
        if not self.range_calibration_available_phi and self.arc_calibration_available_phi:
            return 'FAILED: requires range and calibration measurements'

        full_range = self.range_angle_phi - self.backlash_angle  # account for backlash
        nsteps = np.int(full_range / angle)

        movelog = {}

        for direction in ['cw', 'ccw']:
            movelog[direction] = {}
            # first move ccw to hardstop
            self.move_to_hardstop_phi(reverse_direction(direction))
            movelog[direction]['centroids'] = []
            angle_to_target = angle
            while nsteps > 0:
                nmoves = 0
                movelog[direction][nsteps] = {'nmoves': 0, 'centroids': [], 'direction': [], 'speed': [],
                                              'angle_measured': [], 'angle_to_target': []}
                start_point = self.get_centroids()
                movelog[direction][nsteps]['centroids'].append(start_point)
                while abs(
                        angle_to_target) > self.targetdistance and nmoves <= self.number_of_correction_moves + 1:  # need to add '1' for blind move
                    if angle_to_target > 0:
                        move_direction = direction
                    else:
                        move_direction = reverse_direction(direction)
                    if angle_to_target > self.creep_threshold_phi:
                        self.move_direct_cruise(move_direction, self.speed_mode_cruise_phi, 'phi', angle_to_target)
                        speed = 'cruise'
                    else:
                        self.move_direct_creep(move_direction, self.speed_mode_creep_phi, 'phi', angle_to_target)
                        speed = 'creep'
                    movelog[direction][nsteps]['nmoves'] += 1
                    movelog[direction][nsteps]['direction'].append(move_direction)
                    movelog[direction][nsteps]['speed'].append(speed)
                    point = self.get_centroids()
                    movelog[direction][nsteps]['centroids'].append(start_point)
                    angle_measured = get_angle(move_direction, self.center_phi, start_point, point)
                    movelog[direction][nsteps]['angle_measured'].append(angle_measured)
                    angle_to_target = angle - angle_measured
                    movelog[direction][nsteps]['angle_to_target'].append(angle_to_target)
                    nmoves += 1
                nsteps -= 1
        return movelog


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-a", "--angle", type=int, help="move angle", required=True)
    parser.add_argument("-m", "--motor", type=str, help="motor (theta or phi)", required=True)
    parser.add_argument("-e", "--exposuretime", type=int, help="exposure time in ms", required=False)
    parser.add_argument("-s", "--speedmode", type=str, help="selected speed mode cruise", required=False)
    parser.add_argument("-sc", "--creepmode", type=str, help="selected speed mode creep", required=False)
    parser.add_argument("-d", "--cruisescale", type=str, help="dynamic scale cruise cw,ccw", required=False)
    parser.add_argument("-dc", "--creepscale", type=str, help="dynamic scale creep cw,ccw", required=False)
    parser.add_argument("-p", "--posid", type=int, help="move angle", required=True)
    parser.add_argument("-td", "--targetdistance", type=float, help="target distance angle", required=False)
    parser.add_argument("-c", "--calibration", action='store_true')  # default is false
    parser.add_argument("-t", "--test", type=str, help="test to run (cruise_test, creep_test, positioning_test)",
                        required=True)
    args = parser.parse_args()

    positioner_id = int(args.posid)

    target_angle = int(args.angle)
    if args.exposuretime is not None:
        exptime = int(args.exposuretime)
    else:
        exptime = 20

    # 1 deg = 52 microns
    # default is 0.5 deg = 26 microns
    if args.targetdistance is not None:
        target_distance_angle = float(args.targetdistance)
    else:
        target_distance_angle = .5

    motor = args.motor

    if args.speedmode is not None:
        cruise_mode = str(args.speedmode)
    else:
        cruise_mode = 'default'

    if args.creepmode is not None:
        creep_mode = str(args.creepmode)
    else:
        creep_mode = 'default'

    # fipos = fiposcontrol.FiposControl(['can0'])
    dev_by_bus = {'can0': [positioner_id]}
    mm = MoveMeasure(dev_by_bus, camera_name='Sti', verbose=False)

    mm.set_targetdistance(target_distance_angle)

    if motor in ['phi']:
        mm.set_speed_mode_cruise(speed_mode_cruise_phi=cruise_mode, speed_mode_cruise_theta='default')
        mm.set_speed_mode_creep(speed_mode_creep_phi=creep_mode, speed_mode_creep_theta='default')
    else:
        mm.set_speed_mode_cruise(speed_mode_cruise_phi='default', speed_mode_cruise_theta=cruise_mode)
        mm.set_speed_mode_creep(speed_mode_creep_phi='default', speed_mode_creep_theta=creep_mode)

    cruisescale = {'cw': 1., 'ccw': 1.}
    if args.cruisescale is not None:
        if ',' not in args.cruisescale:
            cruisescale = [float(args.cruisescale), float(args.cruisescale)]
        else:
            cruisescale = [float(item) for item in args.cruisescale.split(',')]
    creepscale = {'cw': 1., 'ccw': 1.}
    if args.creepscale is not None:
        if ',' not in args.creepscale:
            creepscale = [float(args.creepscale), float(args.creepscale)]
        else:
            creepscale = [float(item) for item in args.creepscale.split(',')]

    mm.set_gain(motor, cruise_gain=cruisescale, creep_gain=creepscale)

    do_calibration = args.calibration
    test_to_run = args.test

    if test_to_run not in ['cruise_test', 'creep_test', 'positioning_test']:
        print('Error: Invalid test')
        sys.exit()

    print('>> selected motor: ' + str(motor))
    print('>> selected speed mode: ' + str(cruise_mode))
    print('>> selected creep mode: ' + str(creep_mode))
    print('>> selected exposure time: ' + str(exptime))
    print('>> calibration?: ' + str(do_calibration))

    print('>> selected cruise scale cw, ccw: ' + str(cruisescale))
    print('>> selected creep scale cw, ccw: ' + str(creepscale))

    date_str, time_str = get_datetime_string()
    fname = 'linphi_new_'
    # log_file = open(fname + str(positioner_id) + '_' + str(target_angle) + '_' + speed_mode + '_' + date_str + '_' + time_str + '.log', 'w')
    pickle_file = open(fname + str(positioner_id) + '_' + str(target_angle) + '_' + date_str + '_' + time_str + '.pkl',
                       'wb')

    testlog = {'head': {}, 'calibration': {'phi': {}, 'theta': {}}, 'movelog': {}}

    if motor in ['phi']:
        mm.calibrate_phi(use_stored_if_available=True)

        testlog['calibration']['phi']['radius'] = mm.radius_phi
        testlog['calibration']['phi']['center'] = mm.center_phi
        testlog['calibration']['phi']['range_angle'] = mm.range_angle_phi
        testlog['calibration']['phi']['range_limits'] = mm.range_limits_phi

    if motor in ['theta']:
        print(' Not yet implemented')
        sys.exit()
        # if not mm.calibration_was_measured_theta:
        #    done = input(' Need to measure calibration in theta. Move Phi against ccw hardstop (i.e. move ccw); hit return when done. '
        # if not mm.range_was_measured_theta:
        #    done = input(' Need to measure range in theta. Move Phi against ccw hardstop (i.e. move ccw); hit return when done. '

    testlog['head']['motor'] = motor
    testlog['head']['test'] = test_to_run

    date_str, time_str = get_datetime_string()

    testlog['head']['time'] = time_str
    testlog['head']['date'] = date_str
    testlog['head']['posid'] = ''
    testlog['head']['targetdistance'] = ''

    testlog['head']['cruise_mode_phi'] = ''
    testlog['head']['cruise_mode_theta'] = ''
    testlog['head']['creep_mode_phi'] = ''
    testlog['head']['creep_mode_theta'] = ''

    testlog['head']['cruise_gain_phi'] = ''
    testlog['head']['cruise_gain_theta'] = ''
    testlog['head']['creep_gain_phi'] = ''
    testlog['head']['creep_gain_theta'] = ''

    if motor in ['phi']:
        if test_to_run in ['cruise_test']:
            testlog['movelog']['cruise_test'] = {}

            centroids = mm.cruise_step_sequence_phi(target_angle, nsteps=None)
            testlog['movelog']['cruise_test']['centroids'] = centroids

        if test_to_run in ['creep_test']:
            testlog['movelog']['creep_test'] = {}

            centroids = mm.creep_step_sequence_phi(target_angle, nsteps=None)
            testlog['movelog']['cruise_test']['centroids'] = centroids
    else:
        print('not implemented yet')
        sys.exit()

    pickle.dump(testlog, pickle_file)
    pickle_file.close()
    mm.close_camera()
    print('DONE!')
