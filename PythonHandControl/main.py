#!/usr/bin/env python
# -*- coding: utf-8 -*-
import csv
import copy
import argparse
import itertools
from collections import Counter
from collections import deque

import cv2 as cv
import numpy as np
import mediapipe as mp
import time
import math
import serial
import serial.tools.list_ports


from utils import CvFpsCalc
from model import KeyPointClassifier
from model import PointHistoryClassifier


# Teensy USB serial microcontroller program id data:
VENDOR_ID = 5824
PRODUCT_ID = 1155
SERIAL_NUMBER = 14814510
SERIAL_BAUDRATE = 115200


# Serial packet flag bits
ARM_RESET_POSITION        = 0x80  # Reset to starting position
ARM_MAX_HEIGHT_POSITION   = 0x40  # Move steper 1 full height
ARM_FULL_FORWARD_POSITION = 0x20  # Move stepper 1 full forward
ARM_OPEN_PWMSERVO         = 0x10  # Open the PWMServo completely
ARM_CLOSE_PWMSERVO        = 0x08  # Close the PWMServo
ARM_ONLY_FLAG_VALID       = 0x04  # Use only flag value of Serial Packet
ARM_NOCHECK_MOVE          = 0x02  # Move without hardware checks
ARM_MOVE_ABSOLUTE         = 0x01  # Move absolute (default relative)

ARM_HAND_X_DELTA        = 150
ARM_HAND_Y_DELTA        = 100

# Some constants
# Camera picture resolution
pic_frame_xrange = 1920
pic_frame_yrange = 1080
pic_frame_xcenter = (pic_frame_xrange / 2)
pic_frame_ycenter = (pic_frame_yrange / 2)


real_ball_diameter = 40.0 # in mm

# Steppers absolute position
stepper1_absolute_position = 0 # Right Stepper
stepper2_absolute_position = 0 # Base Stepper
stepper3_absolute_position = 0 # Left Stepper

# ARM state flags
previous_hand_signid = None


ARM_MOVE_HORIZONTAL = 0
ARM_MOVE_VERTICAL   = 1
ARM_MOVE_JAWS       = 2

arm_current_mode = ARM_MOVE_HORIZONTAL


serial_packet_number = 0


def getTeensyPort():
    return '/dev/cu.usbmodem148145101'


def serial_flush(sph):
    sph.flush()
    return



def increment_and_serial_packet_number():
    global serial_packet_number
    serial_packet_number += 1
    print("Sent Packet = ", serial_packet_number)
    return


def serial_send_packet(sph, flag, n0, n1, n2, n3, n4, n5):
    sph.write(bytearray('#', 'ascii'))
    sph.write(flag.to_bytes(2, byteorder='little', signed=True))
    sph.write(n0.to_bytes(4, byteorder='little', signed=True))
    sph.write(n1.to_bytes(4, byteorder='little', signed=True))
    sph.write(n2.to_bytes(4, byteorder='little', signed=True))
    sph.write(n3.to_bytes(4, byteorder='little', signed=True))
    sph.write(n4.to_bytes(4, byteorder='little', signed=True))
    sph.write(n5.to_bytes(4, byteorder='little', signed=True))
    sph.write(bytearray('$', 'ascii'))
    increment_and_serial_packet_number()
    #print("a = ", n0, ", b = ", n1, ", c = ", n2)
    #print("x1 = ", n3, ", y1 = ", n4, ", z1 = ", n5)
    return


def pwmservo_open_arms(sph):
    flag = ARM_OPEN_PWMSERVO | ARM_ONLY_FLAG_VALID
    serial_send_packet(sph, flag, 0, 0, 0, 0, 0, 0)
    return


def pwmservo_close_arms(sph):
    flag = ARM_CLOSE_PWMSERVO | ARM_ONLY_FLAG_VALID
    serial_send_packet(sph, flag, 0, 0, 0, 0, 0, 0)
    return


def arm_move_absolute(sph, p1, p2, p3, r, b, l):
    flag = ARM_MOVE_ABSOLUTE
    serial_send_packet(sph, flag, p1, p2, p3, r, b, l)
    return


# Update ARM absolute position
def update_absolute_move_position(abs_s1, abs_s2, abs_s3):
    global stepper1_absolute_position
    global stepper2_absolute_position
    global stepper3_absolute_position
    stepper1_absolute_position = abs_s1 # Right Stepper
    stepper2_absolute_position = abs_s2 # Base Stepper
    stepper3_absolute_position = abs_s3 # Left Stepper
    return


#
# Calculates the relative steps each stepper needs to make
#
def move_steppers_to_pic_center_pix(sph, dis, xpix, ypix):
    global stepper1_absolute_position
    global stepper2_absolute_position
    global stepper3_absolute_position
    
    stepper1_factor = 700000.0 / dis
    stepper2_factor = 700000.0 / dis
    stepper3_factor = 1000000.0 / dis
    xsteps = (stepper2_factor * xpix) / pic_frame_xrange
    ysteps = (-stepper3_factor * ypix) / pic_frame_yrange
    r = stepper1_absolute_position
    b = stepper2_absolute_position + int(xsteps)
    l = stepper3_absolute_position + int(ysteps)

    print("stepper2_absolute_position = ", stepper2_absolute_position)
    if (b < 0): 
        print("LEFT = ", b)
    else:
        print("RIGHT = ", b)

    arm_move_absolute(sph, stepper1_absolute_position, stepper2_absolute_position, stepper3_absolute_position, r, b, l)
    time.sleep(abs((max(xsteps, ysteps)))/1000.0)
    update_absolute_move_position(r, b, l)
    return


# Returns the distance in mm
# This cde uses focal length
def calculate_distance(pixel_diameter, focal_length_mm, real_diameter):
    """
    Calculate the distance to an object using the formula: distance = (real_diameter * focal_length_mm) / pixel_diameter
    :param pixel_diameter: Diameter of the object in pixels
    :param focal_length_mm: Focal length of the camera in millimeters
    :param real_diameter: Actual diameter of the object in millimeters
    :return: Distance to the object in millimeters
    """
    return round((real_diameter * focal_length_mm) / pixel_diameter, 6)


def filter_contours(contours, min_area, circularity_threshold):
    """
    Filter contours based on area and circularity
    :param contours: List of contours to filter
    :param min_area: Minimum area threshold
    :param circularity_threshold: Circularity threshold
    :return: Filtered contours
    """
    filtered_contours = []
    for contour in contours:
        # Calculate contour area
        area = cv2.contourArea(contour)
        if area > min_area:
            # Calculate circularity of the contour
            perimeter = cv2.arcLength(contour, True)
            circularity = 4 * np.pi * area / (perimeter ** 2)
            if circularity > circularity_threshold:
                filtered_contours.append(contour)
    return filtered_contours



def get_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--ball_tracking", type=bool, default=False)
    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--width", help='cap width', type=int, default=960)
    parser.add_argument("--height", help='cap height', type=int, default=540)

    parser.add_argument('--use_static_image_mode', action='store_true')
    parser.add_argument("--min_detection_confidence",
                        help='min_detection_confidence',
                        type=float,
                        default=0.9)
    parser.add_argument("--min_tracking_confidence",
                        help='min_tracking_confidence',
                        type=int,
                        default=0.5)

    args = parser.parse_args()

    return args


def arm_ball_tracking(sph, camera_frame, camera_matrix, dist_coeffs, focal_length, mfactorx, mfacitory):
    global real_ball_diameter
    global stepper1_absolute_position
    global stepper2_absolute_position
    global stepper3_absolute_position

    # We get inverted frame from the camera and it needs to be rotated
    # Rotate the image both vertically and horizontally
    frame = cv2.flip(camera_frame, -1)
    undistorted_frame = cv2.undistort(frame, camera_matrix, dist_coeffs)

    # Convert the frame to HSV color space
    img_hsv = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2HSV)

    # Threshold the image to isolate the ball
    # orange_lower = np.array([0, 100, 100])
    # orange_upper = np.array([30, 255, 255])
    green_lower = np.array((40, 40, 40))
    green_upper = np.array((90, 255, 255))
    mask_ball = cv2.inRange(img_hsv, green_lower, green_upper)

    # Apply morphological operations to remove noise
    kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))

    mask_ball = cv2.morphologyEx(mask_ball, cv2.MORPH_OPEN, kernel_open)
    mask_ball = cv2.morphologyEx(mask_ball, cv2.MORPH_CLOSE, kernel_close)

    # Find contours of the ball
    contours, _ = cv2.findContours(mask_ball, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter contours based on size and circularity
    min_area = 1000  # Adjusted minimum area threshold
    circularity_threshold = 0.5  # Adjusted circularity threshold
    filtered_contours = filter_contours(contours, min_area, circularity_threshold)
    center = None

    # check if there is atleast one contour
    if len(filtered_contours) > 0:
        # Find the contour with the largest area (the ball)
        ball_contour = max(filtered_contours, key=cv2.contourArea)
        epsilon = 0.001 * cv2.arcLength(ball_contour, True)
        ball_boundary = cv2.approxPolyDP(ball_contour, epsilon, True)

        if len(ball_boundary) > 2:
            # Calculate the center, radius and diameter of the ball
            (x, y), radius = cv2.minEnclosingCircle(ball_boundary)
            diameter = radius * 2

            pic_frame_xcenter = ((pic_frame_xrange/mfactorx) / 2)
            pic_frame_ycenter = ((pic_frame_yrange/mfactory) / 2)

            x_pix_delta = x - pic_frame_xcenter
            y_pix_delta = pic_frame_ycenter - y

            print("Ball x = ", x)
            print("x_pix_delta = ", x_pix_delta)

            M = cv2.moments(ball_contour)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # draw the circle and centroid on the frame,
            # then update the list of tracked points

            # Uncomment this code if you want a perfect circle around the ball
            # cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            # cv2.circle(frame, center, 5, (0, 0, 255), -1)

            # Draw the boundary around the ball as a polygon
            cv2.drawContours(frame, [ball_boundary], -1, (0, 255, 0), 2)
            # Calculate and print the distance to the ball
            distance = calculate_distance(diameter, focal_length_mm, real_ball_diameter)
            print("Distance to the ball: {:.6f} mm".format(distance))

            move_steppers_to_pic_center_pix(sph, distance, x_pix_delta, y_pix_delta)

            # Display the frame
            cv2.imshow("Live Feed", frame)
        else:
            cv2.imshow("Live Feed", frame)
    return



def arm_move_tracking(sph, hand_signid):
    global arm_current_mode
    global previous_hand_signid
    global stepper1_absolute_position
    global stepper2_absolute_position
    global stepper3_absolute_position

    if (hand_signid == 3) or (hand_signid == 0):  # OK or Open
        if arm_current_mode == ARM_MOVE_HORIZONTAL:
            print("HORIZONTAL")
            # Moe the arm  left
            r = stepper1_absolute_position
            b = stepper2_absolute_position + ARM_HAND_X_DELTA
            l = stepper3_absolute_position
            arm_move_absolute(sph, stepper1_absolute_position, stepper2_absolute_position, stepper3_absolute_position, r, b, l)
            time.sleep(abs(ARM_HAND_X_DELTA/1000.0))
            update_absolute_move_position(r, b, l)

        elif arm_current_mode == ARM_MOVE_VERTICAL:
            print("VERTICAL")
            # Moe the arm up
            r = stepper1_absolute_position
            b = stepper2_absolute_position
            l = stepper3_absolute_position + ARM_HAND_Y_DELTA
            arm_move_absolute(sph, stepper1_absolute_position, stepper2_absolute_position, stepper3_absolute_position, r, b, l)
            time.sleep(abs(ARM_HAND_Y_DELTA/1000.0))
            update_absolute_move_position(r, b, l)

        elif arm_current_mode == ARM_MOVE_JAWS:
            print("JAWS")
            # Open the jaws
            pwmservo_open_arms(sph)
            time.sleep(abs(0.1))

        else:
            print("Error: Wrong ARM mode")

    elif (hand_signid == 1):  # Close
        if arm_current_mode == ARM_MOVE_HORIZONTAL:
            print("HORIZONTAL")
            # Moe the arm  right
            r = stepper1_absolute_position
            b = stepper2_absolute_position - ARM_HAND_X_DELTA
            l = stepper3_absolute_position
            arm_move_absolute(sph, stepper1_absolute_position, stepper2_absolute_position, stepper3_absolute_position, r, b, l)
            time.sleep(abs(ARM_HAND_X_DELTA/1000.0))
            update_absolute_move_position(r, b, l)

        elif arm_current_mode == ARM_MOVE_VERTICAL:
            print("VERTICAL")
            # Moe the arm down
            r = stepper1_absolute_position
            b = stepper2_absolute_position
            l = stepper3_absolute_position - ARM_HAND_Y_DELTA
            arm_move_absolute(sph, stepper1_absolute_position, stepper2_absolute_position, stepper3_absolute_position, r, b, l)
            time.sleep(abs(ARM_HAND_Y_DELTA/1000.0))
            update_absolute_move_position(r, b, l)

        elif arm_current_mode == ARM_MOVE_JAWS:
            print("JAWS")
            # Close the jaws
            pwmservo_close_arms(sph)
            time.sleep(abs(0.1))

        else:
            print("Error: Wrong ARM mode")

    elif (hand_signid == 2):  # Pointer
        print("Pointer: Mode Change")
        if (previous_hand_signid != 2):
            if arm_current_mode == ARM_MOVE_HORIZONTAL:
                arm_current_mode = ARM_MOVE_VERTICAL
            elif arm_current_mode == ARM_MOVE_VERTICAL:
                arm_current_mode = ARM_MOVE_JAWS
            elif arm_current_mode == ARM_MOVE_JAWS:
                arm_current_mode = ARM_MOVE_HORIZONTAL
            else:
                print("Wrong ARM mode")
                arm_current_mode = ARM_MOVE_HORIZONTAL

    else:
        print("NO SIGNAL")

    previous_hand_signid = hand_signid
    return



def main():

    # Argument parsing #################################################################
    args = get_args()

    cap_device = args.device
    cap_width = args.width
    cap_height = args.height

    x_magnifiction_factor = pic_frame_xrange / cap_width
    y_magnifiction_factor = pic_frame_yrange / cap_height

    use_static_image_mode = args.use_static_image_mode
    min_detection_confidence = args.min_detection_confidence
    min_tracking_confidence = args.min_tracking_confidence

    ball_tracking = args.ball_tracking

    use_brect = True

    # Initialize the serial port if we have to track the ball ##########################
    serial_port_handle = None
    serial_port = getTeensyPort()
    serial_port_handle = serial.Serial(serial_port, SERIAL_BAUDRATE)
    serial_port_handle.flush()
    print("Teensy Serial Port = ", serial_port)

    # Camera preparation ###############################################################
    # Load camera calibration parameters
    calibration_file = "camera_calibration.npz"
    calibration_data = np.load(calibration_file)
    camera_matrix = calibration_data["camera_matrix"]
    dist_coeffs = calibration_data["dist_coeffs"]

    # Get the focal length in millimeters from the calibration data
    focal_length_mm = calibration_data["focal_length_mm"][0]
    print("Camera Focal Length = ", focal_length_mm)

    cap = cv.VideoCapture(cap_device)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, cap_width)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, cap_height)

    # Model load #############################################################
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(
        static_image_mode=use_static_image_mode,
        max_num_hands=2,
        min_detection_confidence=min_detection_confidence,
        min_tracking_confidence=min_tracking_confidence,
    )

    keypoint_classifier = KeyPointClassifier()

    point_history_classifier = PointHistoryClassifier()

    # Read labels ###########################################################
    with open('model/keypoint_classifier/keypoint_classifier_label.csv',
              encoding='utf-8-sig') as f:
        keypoint_classifier_labels = csv.reader(f)
        keypoint_classifier_labels = [
            row[0] for row in keypoint_classifier_labels
        ]
    with open(
            'model/point_history_classifier/point_history_classifier_label.csv',
            encoding='utf-8-sig') as f:
        point_history_classifier_labels = csv.reader(f)
        point_history_classifier_labels = [
            row[0] for row in point_history_classifier_labels
        ]

    print(keypoint_classifier_labels)


    # FPS Measurement ########################################################
    cvFpsCalc = CvFpsCalc(buffer_len=10)

    # Coordinate history #################################################################
    history_length = 16
    point_history = deque(maxlen=history_length)

    # Finger gesture history ################################################
    finger_gesture_history = deque(maxlen=history_length)

    #  ########################################################################
    mode = 0

    while True:
        fps = cvFpsCalc.get()

        # Process Key (ESC: end) #################################################
        key = cv.waitKey(10)
        if key == 27:  # ESC
            break
        number, mode = select_mode(key, mode)

        # Camera capture #####################################################
        ret, image = cap.read()
        if not ret:
            break

        if (ball_tracking): # Ball tracking 
            arm_ball_tracking(serial_port_handle, image, camera_matrix, dist_coeffs, focal_length_mm, x_magnifiction_factor, y_magnifiction_factor)
            continue

        image = cv.flip(image, 1)  # Mirror display
        debug_image = copy.deepcopy(image)

        # Detection implementation #############################################################
        image = cv.cvtColor(image, cv.COLOR_BGR2RGB)

        image.flags.writeable = False
        results = hands.process(image)
        image.flags.writeable = True

        #  ####################################################################
        if results.multi_hand_landmarks is not None:
            for hand_landmarks, handedness in zip(results.multi_hand_landmarks,
                                                  results.multi_handedness):
                # Bounding box calculation
                brect = calc_bounding_rect(debug_image, hand_landmarks)
                # Landmark calculation
                landmark_list = calc_landmark_list(debug_image, hand_landmarks)

                # Conversion to relative coordinates / normalized coordinates
                pre_processed_landmark_list = pre_process_landmark(
                    landmark_list)
                pre_processed_point_history_list = pre_process_point_history(
                    debug_image, point_history)
                # Write to the dataset file
                logging_csv(number, mode, pre_processed_landmark_list,
                            pre_processed_point_history_list)

                # Hand sign classification
                hand_sign_id = keypoint_classifier(pre_processed_landmark_list)
                if hand_sign_id == "Not Applicable":  # Point gesture
                    point_history.append(landmark_list[8])
                else:
                    point_history.append([0, 0])

                # Finger gesture classification
                finger_gesture_id = 0
                point_history_len = len(pre_processed_point_history_list)
                if point_history_len == (history_length * 2):
                    finger_gesture_id = point_history_classifier(
                        pre_processed_point_history_list)

                # Calculates the gesture IDs in the latest detection
                finger_gesture_history.append(finger_gesture_id)
                most_common_fg_id = Counter(
                    finger_gesture_history).most_common()

                # Drawing part
                debug_image = draw_bounding_rect(use_brect, debug_image, brect)
                debug_image = draw_landmarks(debug_image, landmark_list)
                debug_image = draw_info_text(
                    debug_image,
                    brect,
                    handedness,
                    keypoint_classifier_labels[hand_sign_id],
                    point_history_classifier_labels[most_common_fg_id[0][0]],
                )
                if (not ball_tracking): # ARM movement tracking 
                    arm_move_tracking(serial_port_handle, hand_sign_id)

        else:
            point_history.append([0, 0])

        debug_image = draw_point_history(debug_image, point_history)
        debug_image = draw_info(debug_image, fps, mode, number)

        # Screen reflection #############################################################
        cv.imshow('Hand Gesture Recognition', debug_image)

    cap.release()
    cv.destroyAllWindows()


def select_mode(key, mode):
    number = -1
    if 48 <= key <= 57:  # 0 ~ 9
        number = key - 48
    if key == 110:  # n
        mode = 0
    if key == 107:  # k
        mode = 1
    if key == 104:  # h
        mode = 2
    return number, mode


def calc_bounding_rect(image, landmarks):
    image_width, image_height = image.shape[1], image.shape[0]

    landmark_array = np.empty((0, 2), int)

    for _, landmark in enumerate(landmarks.landmark):
        landmark_x = min(int(landmark.x * image_width), image_width - 1)
        landmark_y = min(int(landmark.y * image_height), image_height - 1)

        landmark_point = [np.array((landmark_x, landmark_y))]

        landmark_array = np.append(landmark_array, landmark_point, axis=0)

    x, y, w, h = cv.boundingRect(landmark_array)

    return [x, y, x + w, y + h]


def calc_landmark_list(image, landmarks):
    image_width, image_height = image.shape[1], image.shape[0]

    landmark_point = []

    # Keypoint
    for _, landmark in enumerate(landmarks.landmark):
        landmark_x = min(int(landmark.x * image_width), image_width - 1)
        landmark_y = min(int(landmark.y * image_height), image_height - 1)
        # landmark_z = landmark.z

        landmark_point.append([landmark_x, landmark_y])

    return landmark_point


def pre_process_landmark(landmark_list):
    temp_landmark_list = copy.deepcopy(landmark_list)

    # Convert to relative coordinates
    base_x, base_y = 0, 0
    for index, landmark_point in enumerate(temp_landmark_list):
        if index == 0:
            base_x, base_y = landmark_point[0], landmark_point[1]

        temp_landmark_list[index][0] = temp_landmark_list[index][0] - base_x
        temp_landmark_list[index][1] = temp_landmark_list[index][1] - base_y

    # Convert to a one-dimensional list
    temp_landmark_list = list(
        itertools.chain.from_iterable(temp_landmark_list))

    # Normalization
    max_value = max(list(map(abs, temp_landmark_list)))

    def normalize_(n):
        return n / max_value

    temp_landmark_list = list(map(normalize_, temp_landmark_list))

    return temp_landmark_list


def pre_process_point_history(image, point_history):
    image_width, image_height = image.shape[1], image.shape[0]

    temp_point_history = copy.deepcopy(point_history)

    # Convert to relative coordinates
    base_x, base_y = 0, 0
    for index, point in enumerate(temp_point_history):
        if index == 0:
            base_x, base_y = point[0], point[1]

        temp_point_history[index][0] = (temp_point_history[index][0] -
                                        base_x) / image_width
        temp_point_history[index][1] = (temp_point_history[index][1] -
                                        base_y) / image_height

    # Convert to a one-dimensional list
    temp_point_history = list(
        itertools.chain.from_iterable(temp_point_history))

    return temp_point_history


def logging_csv(number, mode, landmark_list, point_history_list):
    if mode == 0:
        pass
    if mode == 1 and (0 <= number <= 9):
        csv_path = 'model/keypoint_classifier/keypoint.csv'
        with open(csv_path, 'a', newline="") as f:
            writer = csv.writer(f)
            writer.writerow([number, *landmark_list])
    if mode == 2 and (0 <= number <= 9):
        csv_path = 'model/point_history_classifier/point_history.csv'
        with open(csv_path, 'a', newline="") as f:
            writer = csv.writer(f)
            writer.writerow([number, *point_history_list])
    return


def draw_landmarks(image, landmark_point):
    if len(landmark_point) > 0:
        # Thumb
        cv.line(image, tuple(landmark_point[2]), tuple(landmark_point[3]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[2]), tuple(landmark_point[3]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[3]), tuple(landmark_point[4]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[3]), tuple(landmark_point[4]),
                (255, 255, 255), 2)

        # Index finger
        cv.line(image, tuple(landmark_point[5]), tuple(landmark_point[6]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[5]), tuple(landmark_point[6]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[6]), tuple(landmark_point[7]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[6]), tuple(landmark_point[7]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[7]), tuple(landmark_point[8]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[7]), tuple(landmark_point[8]),
                (255, 255, 255), 2)

        # Middle finger
        cv.line(image, tuple(landmark_point[9]), tuple(landmark_point[10]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[9]), tuple(landmark_point[10]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[10]), tuple(landmark_point[11]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[10]), tuple(landmark_point[11]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[11]), tuple(landmark_point[12]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[11]), tuple(landmark_point[12]),
                (255, 255, 255), 2)

        # Ring finger
        cv.line(image, tuple(landmark_point[13]), tuple(landmark_point[14]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[13]), tuple(landmark_point[14]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[14]), tuple(landmark_point[15]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[14]), tuple(landmark_point[15]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[15]), tuple(landmark_point[16]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[15]), tuple(landmark_point[16]),
                (255, 255, 255), 2)

        # Little finger
        cv.line(image, tuple(landmark_point[17]), tuple(landmark_point[18]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[17]), tuple(landmark_point[18]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[18]), tuple(landmark_point[19]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[18]), tuple(landmark_point[19]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[19]), tuple(landmark_point[20]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[19]), tuple(landmark_point[20]),
                (255, 255, 255), 2)

        # Palm
        cv.line(image, tuple(landmark_point[0]), tuple(landmark_point[1]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[0]), tuple(landmark_point[1]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[1]), tuple(landmark_point[2]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[1]), tuple(landmark_point[2]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[2]), tuple(landmark_point[5]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[2]), tuple(landmark_point[5]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[5]), tuple(landmark_point[9]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[5]), tuple(landmark_point[9]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[9]), tuple(landmark_point[13]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[9]), tuple(landmark_point[13]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[13]), tuple(landmark_point[17]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[13]), tuple(landmark_point[17]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[17]), tuple(landmark_point[0]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[17]), tuple(landmark_point[0]),
                (255, 255, 255), 2)

    # Key Points
    for index, landmark in enumerate(landmark_point):
        if index == 0:  # 手首1
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 1:  # 手首2
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 2:  # 親指：付け根
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 3:  # 親指：第1関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 4:  # 親指：指先
            cv.circle(image, (landmark[0], landmark[1]), 8, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)
        if index == 5:  # 人差指：付け根
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 6:  # 人差指：第2関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 7:  # 人差指：第1関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 8:  # 人差指：指先
            cv.circle(image, (landmark[0], landmark[1]), 8, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)
        if index == 9:  # 中指：付け根
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 10:  # 中指：第2関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 11:  # 中指：第1関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 12:  # 中指：指先
            cv.circle(image, (landmark[0], landmark[1]), 8, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)
        if index == 13:  # 薬指：付け根
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 14:  # 薬指：第2関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 15:  # 薬指：第1関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 16:  # 薬指：指先
            cv.circle(image, (landmark[0], landmark[1]), 8, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)
        if index == 17:  # 小指：付け根
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 18:  # 小指：第2関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 19:  # 小指：第1関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 20:  # 小指：指先
            cv.circle(image, (landmark[0], landmark[1]), 8, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)

    return image


def draw_bounding_rect(use_brect, image, brect):
    if use_brect:
        # Outer rectangle
        cv.rectangle(image, (brect[0], brect[1]), (brect[2], brect[3]),
                     (0, 0, 0), 1)

    return image


def draw_info_text(image, brect, handedness, hand_sign_text,
                   finger_gesture_text):
    cv.rectangle(image, (brect[0], brect[1]), (brect[2], brect[1] - 22),
                 (0, 0, 0), -1)

    info_text = handedness.classification[0].label[0:]
    if hand_sign_text != "":
        info_text = info_text + ':' + hand_sign_text
    cv.putText(image, info_text, (brect[0] + 5, brect[1] - 4),
               cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv.LINE_AA)

    if finger_gesture_text != "":
        cv.putText(image, "Finger Gesture:" + finger_gesture_text, (10, 60),
                   cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 4, cv.LINE_AA)
        cv.putText(image, "Finger Gesture:" + finger_gesture_text, (10, 60),
                   cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2,
                   cv.LINE_AA)

    return image


def draw_point_history(image, point_history):
    for index, point in enumerate(point_history):
        if point[0] != 0 and point[1] != 0:
            cv.circle(image, (point[0], point[1]), 1 + int(index / 2),
                      (152, 251, 152), 2)

    return image


def draw_info(image, fps, mode, number):
    cv.putText(image, "FPS:" + str(fps), (10, 30), cv.FONT_HERSHEY_SIMPLEX,
               1.0, (0, 0, 0), 4, cv.LINE_AA)
    cv.putText(image, "FPS:" + str(fps), (10, 30), cv.FONT_HERSHEY_SIMPLEX,
               1.0, (255, 255, 255), 2, cv.LINE_AA)

    mode_string = ['Logging Key Point', 'Logging Point History']
    if 1 <= mode <= 2:
        cv.putText(image, "MODE:" + mode_string[mode - 1], (10, 90),
                   cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1,
                   cv.LINE_AA)
        if 0 <= number <= 9:
            cv.putText(image, "NUM:" + str(number), (10, 110),
                       cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1,
                       cv.LINE_AA)
    return image


if __name__ == '__main__':
    main()
