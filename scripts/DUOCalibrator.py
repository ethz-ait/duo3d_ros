#!/usr/bin/python

#   Copyright (c) 2016 AIT, ETH Zurich. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name AIT nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# File: DUOCalibrator.py
# Created on: 09.03.16
# Author: Nicolas de Palezieux

from __future__ import print_function, division
import os
import shutil
from std_msgs.msg import String
import yaml
from cv_bridge import CvBridge
import numpy as np
import cv2
from ait_ros_messages.msg import VioSensorMsg

import sys
from PySide.QtCore import *
from PySide.QtGui import *
import rospy
import rospkg
import subprocess
import time


class DUOCalibrator(QMainWindow):
    images_updated = Signal()
    status_bar_update = Signal(str)

    def __init__(self):
        super(DUOCalibrator, self).__init__()

        self.serial_nr = None
        self.calib = None
        self.calib_path = None
        self.vio_sensor_cnt = 0
        self.recording = False
        self.corners = {'left': [], 'right': []}
        self.last_time = time.time()
        self.object_points = []

        self.left_image = None
        self.right_image = None

        self.left_image_label = None
        self.right_image_label = None

        self.chb_size_x_spin = None
        self.chb_size_y_spin = None

        self.res_height = 0
        self.res_width = 0
        self.chb_dimensions_spin = None
        self.start_calib_btn = None
        self.lens_type_le = None

        self.images_updated.connect(self.redraw_images)
        self.status_bar_update.connect(self.statusBar().showMessage)

        self.initUI()

        rospy.Subscriber("/vio_sensor/device_serial_nr", String, self.device_serial_nr_cb, queue_size=1)
        rospy.Subscriber("/vio_sensor", VioSensorMsg, self.vio_sensor_cb, queue_size=1)

    def initUI(self):

        vbox_top = QVBoxLayout()

        start_driver_btn = QPushButton('(Re)Start Camera Driver', self)
        start_driver_btn.clicked.connect(self.restart_duo_node)
        command = 'printenv ROS_MASTER_URI'
        process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
        if 'localhost' not in process.communicate()[0]:
            start_driver_btn.setEnabled(False)
            start_driver_btn.setToolTip('Cannot restart DUO node running on a different machine.')

        self.start_calib_btn = QPushButton('Start recording')
        self.start_calib_btn.clicked.connect(self.recording_button_clicked)

        hbox = QHBoxLayout()
        hbox.addWidget(start_driver_btn)
        hbox.addStretch(1)
        hbox.addWidget(self.start_calib_btn)
        vbox_top.addLayout(hbox)

        self.chb_size_x_spin = QSpinBox()
        self.chb_size_x_spin.setMinimum(3)
        self.chb_size_x_spin.setValue(5)
        self.chb_size_y_spin = QSpinBox()
        self.chb_size_y_spin.setMinimum(3)
        self.chb_size_y_spin.setValue(8)
        chb_size_label = QLabel('Checkerboard size:')
        chb_size_label.setToolTip('Number of inner corners of the checker borad.')
        chb_size_x = QLabel('x')
        self.chb_dimensions_spin = QSpinBox()
        self.chb_dimensions_spin.setMaximum(999)
        self.chb_dimensions_spin.setMinimum(1)
        self.chb_dimensions_spin.setValue(120)
        chb_dim_label = QLabel('Checkerboard dimensions:')
        chb_dim_unit_label = QLabel('mm')
        self.lens_type_le = QLineEdit('FS3028B13M8')
        lens_type_label = QLabel('Lens type:')

        hbox = QHBoxLayout()
        hbox.addWidget(chb_size_label)
        hbox.addWidget(self.chb_size_x_spin)
        hbox.addWidget(chb_size_x)
        hbox.addWidget(self.chb_size_y_spin)
        hbox.addStretch(1)
        hbox.addWidget(chb_dim_label)
        hbox.addWidget(self.chb_dimensions_spin)
        hbox.addWidget(chb_dim_unit_label)
        hbox.addStretch(1)
        hbox.addWidget(lens_type_label)
        hbox.addWidget(self.lens_type_le)

        vbox_top.addLayout(hbox)
        vbox_top.addStretch(1)

        self.left_image_label = QLabel(self)
        self.left_image_label.setText('L')

        self.right_image_label = QLabel(self)
        self.right_image_label.setText('R')

        hbox = QHBoxLayout()
        hbox.addStretch(1)
        hbox.addWidget(self.left_image_label)
        hbox.addWidget(self.right_image_label)
        hbox.addStretch(1)
        vbox_top.addLayout(hbox)

        central_widget = QWidget()

        central_widget.setLayout(vbox_top)
        self.setCentralWidget(central_widget)

        self.status_bar_update.emit('Waiting for Serial Nr from camera driver')

        self.setGeometry(300, 300, 800, 300)
        self.setWindowTitle('DUO Calibrator')
        self.show()

    def keyPressEvent(self, e):

        if e.key() == Qt.Key_Escape:
            self.close()

        super(DUOCalibrator, self).keyPressEvent(e)

    def open_calibration_file(self):
        try:
            package_path = rospkg.RosPack().get_path('duo3d_ros')
        except:
            package_path = os.path.dirname(os.path.realpath(__file__))
            rospy.logerr('Could not find ROS package duo3d_ros, storing calibration in {}'.format(package_path))

        lens = self.lens_type_le.text()
        self.calib_path = os.path.join(package_path, 'calib', self.serial_nr, lens, '{}x{}'.format(self.res_height, self.res_width), 'cameraParams.yaml')

        if os.path.isfile(self.calib_path):
            rospy.loginfo('Found calibration file')

            cnt = 1
            while os.path.isfile(os.path.join(package_path, 'calib', self.serial_nr, lens, '{}x{}'.format(self.res_height, self.res_width), 'cameraParams.yaml_bak{}'.format(cnt))):
                cnt += 1
            rospy.loginfo('Copying old calibration file to cameraParams.yaml_bak{}'.format(cnt))
            shutil.copy(self.calib_path, os.path.join(package_path, 'calib', self.serial_nr, lens, '{}x{}'.format(self.res_height, self.res_width), 'cameraParams.yaml_bak{}'.format(cnt)))

            with open(self.calib_path, 'r+') as calib_file:
                self.calib = yaml.load(calib_file)
        else:
            rospy.logwarn('Did not find existing find calibration. Creating file.')

            if not os.path.exists(os.path.dirname(self.calib_path)):
                rospy.loginfo('Directory does not exist yet. Creating directory.')
                os.makedirs(os.path.dirname(self.calib_path))

            package_path = os.path.dirname(os.path.realpath(__file__))
            template_path = os.path.join(os.path.dirname(package_path), 'calib', 'template_calib.yaml')
            with open(template_path, 'r') as template_file:
                template = yaml.load(template_file)
                with open(self.calib_path, 'w') as calib_file:
                    yaml.dump(template, calib_file)
                with open(self.calib_path, 'r+') as calib_file:
                    self.calib = yaml.load(calib_file)

    def extract_checkerboard_and_draw_corners(self, image, chbrd_size):
        image = CvBridge().imgmsg_to_cv2(image, 'mono8')
        image_color = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        ret, corners = cv2.findChessboardCorners(image_color, chbrd_size)

        if not ret:
            cv2.putText(image_color, 'Checkerboard not found', (0, self.res_height - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255))

        cv2.drawChessboardCorners(image_color, chbrd_size, corners, ret)

        return ret, corners, image_color

    def device_serial_nr_cb(self, data):
        snr = data.data

        if self.serial_nr is not None:
            if not snr == self.serial_nr:
                rospy.logwarn('Got new serial Nr but already have one. Restart to calibrate a new cameara.')
            return

        # if the serial nr is available, the width and height are too
        self.res_height = rospy.get_param('/duo_node/resolution_height')
        self.res_width = rospy.get_param('/duo_node/resolution_width')

        self.left_image_label.setFixedSize(QSize(self.res_width, self.res_height))
        self.right_image_label.setFixedSize(QSize(self.res_width, self.res_height))

        rospy.loginfo('Got new serial Nr: {}'.format(snr))
        self.status_bar_update.emit('Device serial Nr.: {}'.format(snr))
        self.serial_nr = snr

    def vio_sensor_cb(self, data):
        # wait until serial number was received
        if self.serial_nr is None:
            return

        self.vio_sensor_cnt += 1

        # open calibration
        # if self.calib is None and self.recording:
        #     self.open_calibration_file()

        chbrd_size = (self.chb_size_x_spin.value(), self.chb_size_y_spin.value())
        chbrd_size = (self.chb_size_y_spin.value(), self.chb_size_x_spin.value())

        if not self.vio_sensor_cnt % 20:
            # left image
            ret_left, corners_left, self.left_image = self.extract_checkerboard_and_draw_corners(data.left_image, chbrd_size)
            cv2.putText(self.left_image, 'L', (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255))

            # right image
            ret_right, corners_right, self.right_image = self.extract_checkerboard_and_draw_corners(data.right_image, chbrd_size)
            cv2.putText(self.right_image, 'R', (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255))

            if self.recording and ret_left and ret_right:
                elapsed_time = time.time() - self.last_time
                if elapsed_time > 1:
                    self.last_time = time.time()
                    self.corners['left'].append(corners_left)
                    self.corners['right'].append(corners_right)
                    objp = np.zeros((chbrd_size[0] * chbrd_size[1], 3), np.float32)
                    objp[:, :2] = np.mgrid[0:chbrd_size[0], 0:chbrd_size[1]].T.reshape(-1, 2) * self.chb_dimensions_spin.value() / 1000.0
                    # objp[:, :2] = np.mgrid[0:chbrd_size[0], 0:chbrd_size[1]].T.reshape(-1, 2)
                    self.object_points.append(objp)

                    self.status_bar_update.emit('{} frames captured'.format(len(self.corners['left'])))

            self.images_updated.emit()

    def redraw_images(self):
        qim = QImage(self.left_image.data, self.left_image.shape[1], self.left_image.shape[0], self.left_image.strides[0], QImage.Format_RGB888)
        pixmap = QPixmap()
        pixmap.convertFromImage(qim.rgbSwapped())
        self.left_image_label.setPixmap(pixmap)

        qim = QImage(self.right_image.data, self.right_image.shape[1], self.right_image.shape[0], self.right_image.strides[0], QImage.Format_RGB888)
        pixmap = QPixmap()
        pixmap.convertFromImage(qim.rgbSwapped())
        self.right_image_label.setPixmap(pixmap)

    def restart_duo_node(self):
        command = 'rosnode kill /duo_node'
        subprocess.Popen(command.split(), stdout=subprocess.PIPE)
        command = ' roslaunch duo3d_ros duo.launch'
        subprocess.Popen(command.split(), stdout=subprocess.PIPE)

    def recording_button_clicked(self):
        if self.recording:
            self.stop_recording()
        else:
            self.start_recording()

    def start_recording(self):

        self.chb_dimensions_spin.setEnabled(False)
        self.lens_type_le.setEnabled(False)
        self.chb_size_x_spin.setEnabled(False)
        self.chb_size_y_spin.setEnabled(False)
        self.lens_type_le.setToolTip('To change the lens type, restart the calibration tool.')
        self.start_calib_btn.setText('Stop recording')

        self.corners = {'left': [], 'right': []}
        self.status_bar_update.emit('{} frames captured'.format(len(self.corners['left'])))

        self.recording = True

    def stop_recording(self):
        self.chb_dimensions_spin.setEnabled(True)
        self.chb_size_x_spin.setEnabled(True)
        self.chb_size_y_spin.setEnabled(True)

        self.start_calib_btn.setText('Start recording')

        self.recording = False

        min_frames = 10
        if len(self.corners['left']) < min_frames:
            QMessageBox.error(self, "Not enough images",
                                        """You have not recorded enough images of the checkerboard. Please record at least {} images to calibrate""".format(min_frames),
                                        QMessageBox.Ok)
            return

        self.status_bar_update.emit('Calibrating left camera intrinsics')
        rmsL, mtxL, distL, rvecsL, tvecsL = cv2.calibrateCamera(self.object_points, self.corners['left'], (self.res_width, self.res_height), None, None, flags=cv2.CALIB_ZERO_TANGENT_DIST)
        print('Left camera RMS reprojection error {}'.format(rmsL))
        if rmsL > 1:
            rospy.logwarn('The reprojection error is very large. Take more images from different view points. Make sure That the checkerboard covers all parts of the image in some frames. Take images from close up, far away and shallow angles of attack.')

        self.status_bar_update.emit('Calibrating right camera intrinsics')
        rmsR, mtxR, distR, rvecsR, tvecsR = cv2.calibrateCamera(self.object_points, self.corners['right'], (self.res_width, self.res_height), None, None, flags=cv2.CALIB_ZERO_TANGENT_DIST)
        print('Right camera RMS reprojection error {}'.format(rmsR))
        if rmsR > 1:
            rospy.logwarn('The reprojection error is very large. Take more images from different view points. Make sure That the checkerboard covers all parts of the image in some frames. Take images from close up, far away and shallow angles of attack.')

        self.status_bar_update.emit('Calibrating extrinsics')
        rmsStereo, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F = cv2.stereoCalibrate(self.object_points,
                                                                                                            self.corners['left'],
                                                                                                            self.corners['right'],
                                                                                                            mtxL,
                                                                                                            distL,
                                                                                                            mtxR,
                                                                                                            distR,
                                                                                                            (self.res_width, self.res_height),
                                                                                                            flags=cv2.CALIB_FIX_INTRINSIC + cv2.CALIB_ZERO_TANGENT_DIST)
        print('Stereo RMS reprojection error {}'.format(rmsStereo))
        if rmsStereo > 1:
            rospy.logwarn('The reprojection error is very large. Take more images from different view points. Make sure That the checkerboard covers all parts of the image in some frames. Take images from close up, far away and shallow angles of attack.')
        thresh = 1  # threshold for RMS warning
        if rmsL > thresh or rmsR > thresh or rmsStereo > thresh:
            reply = QMessageBox.warning(self, "High reprojection error",
                                        """The reprojection error is very large.\nRMS reprojection error left: {:.2f}, right: {:.2f}, stereo: {:.2f}.\
                                        \nTake more images from different view points.\
                                        Make sure That the checkerboard covers all parts of the image in some frames.\
                                        \nTake images from close up, far away and shallow angles of attack.\
                                        \n\nWould you like to save this calibration anyway?""".format(rmsL, rmsR, rmsStereo),
                                        QMessageBox.Yes | QMessageBox.No,
                                        QMessageBox.No)

            if reply == QMessageBox.No:
                return

        self.open_calibration_file()
        self.calib['R_lr'] = R.T.tolist()
        self.calib['R_rl'] = R.tolist()
        self.calib['r_lr'] = (-R.T * T).tolist()

        self.calib['CameraParameters1']['DistortionModel'] = 'plumb_bob'  # opencv parameters are the same as matlab
        self.calib['CameraParameters1']['FocalLength'][0] = float(mtxL[0][0])
        self.calib['CameraParameters1']['FocalLength'][1] = float(mtxL[1][1])
        self.calib['CameraParameters1']['PrincipalPoint'][0] = float(mtxL[0][2])
        self.calib['CameraParameters1']['PrincipalPoint'][1] = float(mtxL[1][2])
        self.calib['CameraParameters1']['RadialDistortion'][0] = float(distL[0][0])
        self.calib['CameraParameters1']['RadialDistortion'][1] = float(distL[0][1])
        self.calib['CameraParameters1']['RadialDistortion'][2] = float(distL[0][4])

        self.calib['CameraParameters2']['DistortionModel'] = 'plumb_bob'
        self.calib['CameraParameters2']['FocalLength'][0] = float(mtxR[0][0])
        self.calib['CameraParameters2']['FocalLength'][1] = float(mtxR[1][1])
        self.calib['CameraParameters2']['PrincipalPoint'][0] = float(mtxR[0][2])
        self.calib['CameraParameters2']['PrincipalPoint'][1] = float(mtxR[1][2])
        self.calib['CameraParameters2']['RadialDistortion'][0] = float(distR[0][0])
        self.calib['CameraParameters2']['RadialDistortion'][1] = float(distR[0][1])
        self.calib['CameraParameters2']['RadialDistortion'][2] = float(distR[0][4])

        with open(self.calib_path, 'w') as outfile:
            outfile.write(yaml.dump(self.calib, default_flow_style=None))
            self.status_bar_update.emit('Wrote calibration to {}'.format(self.calib_path))
            rospy.loginfo('Wrote calibration to {}'.format(self.calib_path))


if __name__ == '__main__':
    rospy.init_node("DUOCalibrator")

    app = QApplication(sys.argv)
    ex = DUOCalibrator()
    ex.show()
    sys.exit(app.exec_())
