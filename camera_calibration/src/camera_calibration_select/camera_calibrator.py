# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import cv2
import message_filters
import numpy
import os
import rclpy
from rclpy.node import Node
import sensor_msgs.msg
import sensor_msgs.srv
import threading
import time
from camera_calibration_select.calibrator import MonoCalibrator, StereoCalibrator, Patterns
try:
    from queue import Queue
except ImportError:
    from Queue import Queue
from camera_calibration.calibrator import CAMERA_MODEL
from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSProfile


class BufferQueue(Queue):
    """Slight modification of the standard Queue that discards the oldest item
    when adding an item and the queue is full.
    """

    def put(self, item, *args, **kwargs):
        # The base implementation, for reference:
        # https://github.com/python/cpython/blob/2.7/Lib/Queue.py#L107
        # https://github.com/python/cpython/blob/3.8/Lib/queue.py#L121
        with self.mutex:
            if self.maxsize > 0 and self._qsize() == self.maxsize:
                self._get()
            self._put(item)
            self.unfinished_tasks += 1
            self.not_empty.notify()


class SpinThread(threading.Thread):
    """
    Thread that spins the ros node, while imshow runs in the main thread
    """

    def __init__(self, node):
        threading.Thread.__init__(self)
        self.node = node

    def run(self):
        rclpy.spin(self.node)


class ConsumerThread(threading.Thread):
    def __init__(self, queue, function):
        threading.Thread.__init__(self)
        self.queue = queue
        self.function = function

    def run(self):
        while rclpy.ok():
            m = self.queue.get()
            self.function(m)


class CalibrationNode(Node):
    def __init__(self, name, boards, service_check=True, synchronizer=message_filters.TimeSynchronizer, flags=0,
                 fisheye_flags=0, pattern=Patterns.Chessboard, camera_name='', checkerboard_flags=0,
                 max_chessboard_speed=-1, queue_size=1):
        super().__init__(name)

        self.set_camera_info_service = self.create_client(sensor_msgs.srv.SetCameraInfo,
                                                          "camera/set_camera_info")
        self.set_left_camera_info_service = self.create_client(sensor_msgs.srv.SetCameraInfo,
                                                               "left_camera/set_camera_info")
        self.set_right_camera_info_service = self.create_client(sensor_msgs.srv.SetCameraInfo,
                                                                "right_camera/set_camera_info")

        if service_check:
            # assume any non-default service names have been set.  Wait for the service to become ready
            for cli in [self.set_camera_info_service, self.set_left_camera_info_service, self.set_right_camera_info_service]:
                # remapped = rclpy.remap_name(svcname)
                # if remapped != svcname:
                # fullservicename = "%s/set_camera_info" % remapped
                print("Waiting for service", cli.srv_name, "...")
                # check all services so they are ready.
                try:
                    cli.wait_for_service(timeout_sec=5)
                    print("OK")
                except Exception as e:
                    print("Service not found: %s".format(e))
                    rclpy.shutdown()

        self._boards = boards
        self._calib_flags = flags
        self._fisheye_calib_flags = fisheye_flags
        self._checkerboard_flags = checkerboard_flags
        self._pattern = pattern
        self._camera_name = camera_name
        self._max_chessboard_speed = max_chessboard_speed
        lsub = message_filters.Subscriber(
            self, sensor_msgs.msg.Image, 'left', qos_profile=self.get_topic_qos("left"))
        rsub = message_filters.Subscriber(
            self, sensor_msgs.msg.Image, 'right', qos_profile=self.get_topic_qos("right"))
        ts = synchronizer([lsub, rsub], 4)
        ts.registerCallback(self.queue_stereo)

        msub = message_filters.Subscriber(
            self, sensor_msgs.msg.Image, 'image', qos_profile=self.get_topic_qos("image"))
        msub.registerCallback(self.queue_monocular)

        self.q_mono = BufferQueue(queue_size)
        self.q_stereo = BufferQueue(queue_size)

        self.c = None

        self._last_display = None

        mth = ConsumerThread(self.q_mono, self.handle_monocular)
        mth.daemon = True
        mth.start()

        sth = ConsumerThread(self.q_stereo, self.handle_stereo)
        sth.daemon = True
        sth.start()

    def redraw_stereo(self, *args):
        pass

    def redraw_monocular(self, *args):
        pass

    def queue_monocular(self, msg):
        self.q_mono.put(msg)

    def queue_stereo(self, lmsg, rmsg):
        self.q_stereo.put((lmsg, rmsg))

    def handle_monocular(self, msg):
        if self.c == None:
            if self._camera_name:
                self.c = MonoCalibrator(self._boards, self._calib_flags, self._fisheye_calib_flags, self._pattern, name=self._camera_name,
                                        checkerboard_flags=self._checkerboard_flags,
                                        max_chessboard_speed=self._max_chessboard_speed)
            else:
                self.c = MonoCalibrator(self._boards, self._calib_flags, self._fisheye_calib_flags, self._pattern,
                                        checkerboard_flags=self.checkerboard_flags,
                                        max_chessboard_speed=self._max_chessboard_speed)

        # This should just call the MonoCalibrator
        drawable = self.c.handle_msg(msg)
        self.displaywidth = drawable.scrib.shape[1]
        self.redraw_monocular(drawable)

    def handle_stereo(self, msg):
        if self.c == None:
            if self._camera_name:
                self.c = StereoCalibrator(self._boards, self._calib_flags, self._fisheye_calib_flags, self._pattern, name=self._camera_name,
                                          checkerboard_flags=self._checkerboard_flags,
                                          max_chessboard_speed=self._max_chessboard_speed)
            else:
                self.c = StereoCalibrator(self._boards, self._calib_flags, self._fisheye_calib_flags, self._pattern,
                                          checkerboard_flags=self._checkerboard_flags,
                                          max_chessboard_speed=self._max_chessboard_speed)

        drawable = self.c.handle_msg(msg)
        self.displaywidth = drawable.lscrib.shape[1] + drawable.rscrib.shape[1]
        self.redraw_stereo(drawable)

    def check_set_camera_info(self, response):
        if response.success:
            return True

        for i in range(10):
            print("!" * 80)
        print()
        print("Attempt to set camera info failed: " + response.result()
              if response.result() is not None else "Not available")
        print()
        for i in range(10):
            print("!" * 80)
        print()
        self.get_logger().error('Unable to set camera info for calibration. Failure message: %s' %
                                response.result() if response.result() is not None else "Not available")
        return False

    def do_upload(self):
        self.c.report()
        print(self.c.ost())
        info = self.c.as_message()

        req = sensor_msgs.srv.SetCameraInfo.Request()
        rv = True
        if self.c.is_mono:
            req.camera_info = info
            response = self.set_camera_info_service.call(req)
            rv = self.check_set_camera_info(response)
        else:
            req.camera_info = info[0]
            response = self.set_left_camera_info_service.call(req)
            rv = rv and self.check_set_camera_info(response)
            req.camera_info = info[1]
            response = self.set_right_camera_info_service.call(req)
            rv = rv and self.check_set_camera_info(response)
        return rv

    def get_topic_qos(self, topic_name: str) -> QoSProfile:
        """!
        Given a topic name, get the QoS profile with which it is being published.
        Replaces history and depth settings with default values since they cannot be retrieved
        @param topic_name (str) the topic name
        @return QosProfile the qos profile with which the topic is published. If no publishers exist
        for the given topic, it returns the sensor data QoS. returns None in case ROS1 is being used
        """
        topic_name = self.resolve_topic_name(topic_name)
        topic_info = self.get_publishers_info_by_topic(topic_name=topic_name)
        if len(topic_info):
            qos_profile = topic_info[0].qos_profile
            qos_profile.history = qos_profile_system_default.history
            qos_profile.depth = qos_profile_system_default.depth
            return qos_profile
        else:
            self.get_logger().warn(
                f"No publishers available for topic {topic_name}. Using system default QoS for subscriber.")
            return qos_profile_system_default


select_start = 200
calibrate_start = 300
save_start = 400
finish_start = 500

main_button_height = 100
main_button_width = 100


class OpenCVCalibrationNode(CalibrationNode):
    """ Calibration node with an OpenCV Gui """
    FONT_FACE = cv2.FONT_HERSHEY_SIMPLEX
    FONT_SCALE = 0.6
    FONT_THICKNESS = 2

    def __init__(self, *args, **kwargs):

        CalibrationNode.__init__(self, *args, **kwargs)

        self.queue_display = BufferQueue(maxsize=1)
        self.initWindow()

    def spin(self):
        sth = SpinThread(self)
        sth.start()

        while rclpy.ok():
            if self.queue_display.qsize() > 0:
                self.image = self.queue_display.get()
                cv2.imshow("display", self.image)
            else:
                time.sleep(0.1)
            k = cv2.waitKey(6) & 0xFF
            if k in [27, ord('q')]:
                return
            elif k == ord('s') and self.image is not None:
                self.screendump(self.image)

    def initWindow(self):
        cv2.namedWindow("display", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("display", self.on_mouse_with_selection)
        cv2.createTrackbar("Camera type: \n 0 : pinhole \n 1 : fisheye",
                           "display", 0, 1, self.on_model_change)
        cv2.createTrackbar("scale", "display", 0, 100, self.on_scale)

    @classmethod
    def putText(cls, img, text, org, color=(0, 0, 0)):
        cv2.putText(img, text, org, cls.FONT_FACE, cls.FONT_SCALE,
                    color, thickness=cls.FONT_THICKNESS)

    @classmethod
    def getTextSize(cls, text):
        return cv2.getTextSize(text, cls.FONT_FACE, cls.FONT_SCALE, cls.FONT_THICKNESS)[0]

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.displaywidth < x:
            if self.c.goodenough:
                if 180 <= y < 280:
                    print("**** Calibrating ****")
                    self.c.do_calibration()
                    self.buttons(self._last_display)
                    self.queue_display.put(self._last_display)
            if self.c.calibrated:
                if 280 <= y < 380:
                    self.c.do_save()
                elif 380 <= y < 480:
                    # Only shut down if we set camera info correctly, #3993
                    if self.do_upload():
                        rclpy.shutdown()

    def on_model_change(self, model_select_val):
        if self.c == None:
            print("Cannot change camera model until the first image has been received")
            return

        self.c.set_cammodel(
            CAMERA_MODEL.PINHOLE if model_select_val < 0.5 else CAMERA_MODEL.FISHEYE)

    def on_model_change(self, model_select_val):
        self.c.set_cammodel(
            CAMERA_MODEL.PINHOLE if model_select_val < 0.5 else CAMERA_MODEL.FISHEYE)

    def on_scale(self, scalevalue):
        if self.c and self.c.calibrated:
            self.c.set_alpha(scalevalue / 100.0)

    def button(self, dst, label, enable):
        dst.fill(255)
        size = (dst.shape[1], dst.shape[0])
        if enable:
            color = (155, 155, 80)
        else:
            color = (224, 224, 224)
        cv2.circle(dst, (size[0] // 2, size[1] // 2),
                   min(size) // 2, color, -1)
        (w, h) = self.getTextSize(label)
        self.putText(
            dst, label, ((size[0] - w) // 2, (size[1] + h) // 2), (255, 255, 255))

    def buttons(self, display):
        x = self.displaywidth
        self.button(display[180:280, x:x+100], "CALIBRATE", self.c.goodenough)
        self.button(display[280:380, x:x+100], "SAVE", self.c.calibrated)
        self.button(display[380:480, x:x+100], "COMMIT", self.c.calibrated)

    def y(self, i):
        """Set up right-size images"""
        return 30 + 40 * i

    def screendump(self, im):
        i = 0
        while os.access("/tmp/dump%d.png" % i, os.R_OK):
            i += 1
        cv2.imwrite("/tmp/dump%d.png" % i, im)
        print("Saved screen dump to /tmp/dump%d.png" % i)

    def redraw_monocular(self, drawable):
        height = drawable.scrib.shape[0]
        width = drawable.scrib.shape[1]
        max_height = max(finish_start + main_button_height, height)
        display = numpy.zeros(
            (max_height, width + 100, 3), dtype=numpy.uint8)
        image_top_edge = (display.shape[0] - height) // 2

        display[image_top_edge:image_top_edge +
                height, 0:width, :] = drawable.scrib
        display[:, width:width+100, :].fill(255)

        self.buttons_with_selection(display)
        if not self.c.calibrated:
            if drawable.params:
                # progression bars for each parameter being optimized
                for i, (label, lo, hi, progress) in enumerate(drawable.params):
                    (w, _) = self.getTextSize(label)
                    self.putText(display, label,
                                 (width + (100 - w) // 2, self.y(i)))
                    color = (0, 255, 0)
                    if progress < 1.0:
                        color = (0, int(progress*255.), 255)
                    cv2.line(display,
                             (int(width + lo * 100), self.y(i) + 20),
                             (int(width + hi * 100), self.y(i) + 20),
                             color, 4)

        else:
            self.putText(display, "lin.", (width, self.y(0)))
            linerror = drawable.linear_error
            if linerror is None or linerror < 0:
                msg = "?"
            else:
                msg = "%.2f" % linerror
                # print "linear", linerror
            self.putText(display, msg, (width, self.y(1)))

        self._last_display = display
        self.queue_display.put(display)

    def redraw_stereo(self, drawable):
        height = drawable.lscrib.shape[0]
        width = drawable.lscrib.shape[1]
        max_height = max(finish_start + main_button_height, height)
        display = numpy.zeros(
            (max_height, 2 * width + 100, 3), dtype=numpy.uint8)

        image_top_edge = (display.shape[0] - height) // 2
        display[image_top_edge:image_top_edge +
                height, 0:width, :] = drawable.lscrib
        display[image_top_edge:image_top_edge+height,
                width:2*width, :] = drawable.rscrib
        display[image_top_edge:image_top_edge +
                height, 2*width:2*width+100, :].fill(255)

        self.buttons_with_selection(display)

        if not self.c.calibrated:
            if drawable.params:
                for i, (label, lo, hi, progress) in enumerate(drawable.params):
                    (w, _) = self.getTextSize(label)
                    self.putText(display, label, (2 * width +
                                 (100 - w) // 2, self.y(i)))
                    color = (0, 255, 0)
                    if progress < 1.0:
                        color = (0, int(progress*255.), 255)
                    cv2.line(display,
                             (int(2 * width + lo * 100), self.y(i) + 20),
                             (int(2 * width + hi * 100), self.y(i) + 20),
                             color, 4)

        else:
            self.putText(display, "epi.", (2 * width, self.y(0)))
            if drawable.epierror == -1:
                msg = "?"
            else:
                msg = "%.2f" % drawable.epierror
            self.putText(display, msg, (2 * width, self.y(1)))
            # TODO dim is never set anywhere. Supposed to be observed chessboard size?
            if drawable.dim != -1:
                self.putText(display, "dim", (2 * width, self.y(2)))
                self.putText(display, "%.3f" %
                             drawable.dim, (2 * width, self.y(3)))

        self._last_display = display
        self.queue_display.put(display)

    _param_names = ["X", "Y", "Size", "Skew"]
    _param_ranges = [0.7, 0.7, 0.4, 0.5]

    def compute_goodenough(self, current_db):
        def lmin(seq1, seq2):
            """ Pairwise minimum of two sequences """
            return [min(a, b) for (a, b) in zip(seq1, seq2)]

        def lmax(seq1, seq2):
            """ Pairwise maximum of two sequences """
            return [max(a, b) for (a, b) in zip(seq1, seq2)]

        if current_db is None or len(current_db) == 0:
            raise ValueError("Cannot compute goodenough without any samples in the database")
        
        # Find range of checkerboard poses covered by samples in database
        all_params = [sample[0] for sample in current_db]
        min_params = all_params[0]
        max_params = all_params[0]
        for params in all_params[1:]:
            min_params = lmin(min_params, params)
            max_params = lmax(max_params, params)
        # Don't reward small size or skew
        min_params = [min_params[0], min_params[1], 0., 0.]

        # For each parameter, judge how much progress has been made toward adequate variation
        progress = [min((hi - lo) / r, 1.0) for (lo, hi, r)
                    in zip(min_params, max_params, self._param_ranges)]

        return list(zip(self._param_names, min_params, max_params, progress))

    def select_images_interactive(self):
        """
        Interactive GUI step during image selection showing live progress bars.
        Displays accept/ignore buttons with real-time parameter visualization.
        """
        if not self.c or not self.c.good_corners:
            print("No good corners detected. Run calibration collection first.")
            return 0

        self.c.allow_append = False  # Disable appending new images during selection

        images = [i for (p, i) in self.c.db]
        accepted_corners = []
        accepted_images = []
        accepted_db = []

        # State variables for GUI interaction
        state = {'current_idx': 0, 'action': None}
        button_height = 50
        button_width = 120
        button_y = 10
        accept_x = 10
        ignore_x = 140
        finish_x = 270

        def mouse_callback(event, x, y, flags, param):
            """Handle mouse clicks on buttons in image selection window"""
            if event == cv2.EVENT_LBUTTONDOWN:
                # Check ACCEPT button
                if accept_x <= x <= accept_x + button_width and button_y <= y <= button_y + button_height:
                    state['action'] = 'accept'
                # Check IGNORE button
                elif ignore_x <= x <= ignore_x + button_width and button_y <= y <= button_y + button_height:
                    state['action'] = 'ignore'
                # Check FINISH button
                elif finish_x <= x <= finish_x + button_width and button_y <= y <= button_y + button_height:
                    state['action'] = 'finish'

        window_name = "Image Selection - Review & Accept"
        cv2.namedWindow(window_name)
        cv2.setMouseCallback(window_name, mouse_callback)
        param_separation_y = 10

        try:
            while state['current_idx'] < len(self.c.good_corners):
                corners, ids, board = self.c.good_corners[state['current_idx']]

                # Get the corresponding image and db entry
                if state['current_idx'] < len(images):
                    img = images[state['current_idx']]
                    db_params, _ = self.c.db[state['current_idx']]
                else:
                    print(
                        f"Warning: Image {state['current_idx']} not found in database")
                    state['current_idx'] += 1
                    continue

                # Create display image with corners drawn
                if len(img.shape) == 2:
                    display_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                else:
                    display_img = img.copy()

                h, w = display_img.shape[:2]

                # Add button area background
                cv2.rectangle(display_img, (0, 0), (w, 70), (50, 50, 50), -1)

                # Draw buttons
                # ACCEPT button (green)
                cv2.rectangle(display_img, (accept_x, button_y),
                              (accept_x + button_width, button_y + button_height), (0, 200, 0), -1)
                cv2.rectangle(display_img, (accept_x, button_y),
                              (accept_x + button_width, button_y + button_height), (0, 255, 0), 2)
                cv2.putText(display_img, "ACCEPT", (accept_x + 15, button_y + 35),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                # IGNORE button (red)
                cv2.rectangle(display_img, (ignore_x, button_y),
                              (ignore_x + button_width, button_y + button_height), (0, 0, 200), -1)
                cv2.rectangle(display_img, (ignore_x, button_y),
                              (ignore_x + button_width, button_y + button_height), (0, 0, 255), 2)
                cv2.putText(display_img, "IGNORE", (ignore_x + 12, button_y + 35),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                # FINISH button (blue)
                cv2.rectangle(display_img, (finish_x, button_y),
                              (finish_x + button_width, button_y + button_height), (200, 0, 0), -1)
                cv2.rectangle(display_img, (finish_x, button_y),
                              (finish_x + button_width, button_y + button_height), (255, 0, 0), 2)
                cv2.putText(display_img, "FINISH", (finish_x + 15, button_y + 35),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                # parameter visualization
                try:
                    params = self.compute_goodenough(accepted_db)
                except Exception as e:
                    print(f"Error computing parameters for visualization: {e}")
                    params = []

                for i, (label, lo, hi, progress) in enumerate(params):
                    (text_width, _) = self.getTextSize(label)
                    self.putText(display_img, label,
                                 (finish_x + button_width + param_separation_y * (i+1) + 100 * i + (100 - text_width) // 2, button_y + button_height // 2 - 10))
                    color = (0, 255, 0)
                    if progress < 1.0:
                        color = (0, int(progress*255.), 255)
                    cv2.line(display_img,
                             (int(finish_x + button_width + (i+1) * param_separation_y + 100 * i + lo * 100),
                              button_y + button_height // 2 + 10),
                             (int(finish_x + button_width + (i+1) * param_separation_y + 100 * i + hi * 100),
                              button_y + button_height // 2 + 10),
                             color, 4)

                # Draw corners on image (below button area)
                display_with_corners = display_img.copy()
                if board.pattern == "charuco" and ids is not None:
                    cv2.aruco.drawDetectedCornersCharuco(
                        display_with_corners, corners, ids)
                else:
                    cv2.drawChessboardCorners(display_with_corners, (board.n_cols, board.n_rows),
                                              corners, True)

                # Add image counter text
                counter_text = f"Image {state['current_idx'] + 1}/{len(self.c.good_corners)} - Accepted: {len(accepted_corners)}"
                cv2.putText(display_with_corners, counter_text, (10, h - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                cv2.imshow(window_name, display_with_corners)

                # Reset action and wait for user click
                state['action'] = None
                while state['action'] is None:
                    key = cv2.waitKey(100) & 0xFF
                    if key == 27:  # ESC key to cancel
                        print("Selection cancelled by user")
                        cv2.destroyWindow(window_name)
                        self.c.allow_append = True  # Re-enable appending new images
                        return len(accepted_corners)

                # Process user action
                if state['action'] == 'accept':
                    accepted_corners.append((corners, ids, board))
                    accepted_images.append(img)
                    accepted_db.append(self.c.db[state['current_idx']])
                    print(f"  Image {state['current_idx'] + 1}: ACCEPTED")
                elif state['action'] == 'ignore':
                    print(f"  Image {state['current_idx'] + 1}: IGNORED")
                elif state['action'] == 'finish':
                    print("Selection finished by user")
                    break

                state['current_idx'] += 1

        finally:
            cv2.destroyWindow(window_name)
            self.c.allow_append = True  # Re-enable appending new images after selection

        # Update good_corners and db with accepted images only
        self.c.good_corners = accepted_corners
        self.c.db = accepted_db

        total_images = state['current_idx']
        print(
            f"\n*** Kept {len(accepted_corners)}/{total_images} images for calibration ***")

        return len(accepted_corners)

    def on_mouse_with_selection(self, event, x, y, flags, param):
        """Enhanced mouse handler with image selection option"""
        if event == cv2.EVENT_LBUTTONDOWN and self.displaywidth < x:
            # Option to start image selection before calibration
            if self.c.goodenough and select_start <= y < calibrate_start:
                print("**** Starting Image Selection ****")
                accepted = self.select_images_interactive()
                if accepted > 0:
                    # Recompute progress after filtering
                    params = self.c.compute_goodenough()
                    print(f"Progress after filtering: {params}")
                    # Re-render with updated parameters
                    self.buttons_with_selection(self._last_display)
                    self.queue_display.put(self._last_display)
                return

            if self.c.goodenough:
                if calibrate_start <= y < save_start:
                    print("**** Calibrating ****")
                    self.c.do_calibration()
                    self.buttons_with_selection(self._last_display)
                    self.queue_display.put(self._last_display)
            if self.c.calibrated:
                if save_start <= y < finish_start:
                    self.c.do_save()
                elif finish_start <= y < finish_start + main_button_height:
                    # Only shut down if we set camera info correctly, #3993
                    if self.do_upload():
                        rclpy.shutdown()

    def buttons_with_selection(self, display):
        """Enhanced button panel with image selection option"""
        x = self.displaywidth
        # if self.c.goodenough and not self.c.calibrated:
        # Add SELECT button
        self.button(display[select_start:select_start+main_button_height,
                    x:x+main_button_width], "SELECT", self.c.goodenough)
        self.button(display[calibrate_start:calibrate_start+main_button_height,
                    x:x+main_button_width], "CALIBRATE", self.c.goodenough)
        self.button(display[save_start:save_start+main_button_height,
                    x:x+main_button_width], "SAVE", self.c.calibrated)
        self.button(display[finish_start:finish_start+main_button_height,
                    x:x+main_button_width], "COMMIT", self.c.calibrated)
