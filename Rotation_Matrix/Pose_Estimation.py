import cv2
import numpy as np
import math
from operator import itemgetter
from PIL import Image
import PIL.ImageOps

from Rotation_Matrix.timer import Timer
from calc_pose import calculate_translation
"""
author: parker
this is a proof-of-concept script that identifies a potential "landing platform" and  uses perspective transforms to calculate
the pose (position/orientation). This is the rotation matrix and the Translation matrix. Currently I am just printing this data
to a terminal and displaying a few edited images using cv2.show(). returns nothing
"""


def callback(x):
    #the cv2.createTrackbar() requires callback param
    pass


#TODO
def calibrate_gimbal():
    """
    come up with a way to run a calibration sequence at startup. For now just set Yaw and Pitch to global variables
    :return: returns nothing
    """
    pass


#TODO
def get_current_gimbal():
    """
    :return: should just return the current pitch&yaw of the gimbal.
    """
    pass


#TODO
def update_gimbal_pid(servoPitch_pos, servoYaw_pos, bounded_rect_coords):
    """
    :param servoPitch_pos: servos current Pitch. Use global variables to update position
    :param servoYaw_pos: servos current Yaw. Use global variables to update position
    :param bounded_rect_coords: use bounded_rect_coords instead of sorted_beacons so that we can update gimbal with less than 4 beacons.
    :return: returns nothing
    """
    pass


def filter_rectangles(sort_rect):
    #return 4 largest rectangles
    sorted_area = sorted(sort_rect, key=itemgetter(4))
    sorted_area.reverse()
    if len(sorted_area) > 4:
        for i in sorted_area:
            sorted_area = sorted_area[:-1]
            if len(sorted_area) == 4:
                break
    return_rectangles = sorted_area
    return return_rectangles


def convert_to_cartesian(beacons):
    i = 0
    # print 'default to cartesian in'
    # print beacons
    for beacon in beacons:
        #convert x
        if beacon[0] > WIDTH / 2:
            beacon[0] = beacon[0] - WIDTH / 2
        elif beacon[0] < WIDTH / 2:
            beacon[0] = WIDTH / 2 - beacon[0]
            beacon[0] *= -1
        else:
            beacon[0] = 0
        # print 'beacon val'
        # print beacon[0]
        beacons[i][0] = beacon[0]
        # print 'array val'
        # print beacons[i][0]
        #convert y
        if beacon[1] > HEIGHT / 2:
            beacon[1] = beacon[1] - HEIGHT / 2
            beacon[1] *= -1
        elif beacon[1] < HEIGHT / 2:
            beacon[1] = HEIGHT / 2 - beacon[1]
        else:
            beacon[1] = 0
        beacons[i][1] = beacon[1]
        i += 1
    # print 'default to cartesian out'
    # print beacons
    return beacons


def cartesian_to_default(sorted_beacons):
    for beacon in sorted_beacons:
        #convert x
        if beacon[0] > 0:
            beacon[0] += WIDTH / 2
        elif beacon[0] < 0:
            beacon[0] *= -1
            beacon[0] = WIDTH / 2 - beacon[0]
        else:
            beacon[0] = WIDTH / 2
        #convert y
        if beacon[1] > 0:
            beacon[1] = HEIGHT / 2 - beacon[1]
        elif beacon[1] < 0:
            beacon[1] *= -1
            beacon[1] = HEIGHT / 2 + beacon[1]
        else:
            beacon[1] = HEIGHT
    # print 'cartesian_to_default out'
    # print sorted_beacons
    printable_coords = sorted_beacons
    return printable_coords


def get_bounded_rect_coords(cartesian_beacons):
    for coords in cartesian_beacons:
        xcoord = coords[0] + coords[2] / 2
        ycoord = coords[1] + coords[3] / 2
        coords[0] = xcoord
        coords[1] = ycoord
    bounded_rect_cords = cartesian_beacons
    return bounded_rect_cords


def sort_beacons(cartesian_coords):
    sorted_by_y = sorted(cartesian_coords, key=itemgetter(1))
    sorted_by_y.reverse()  #smallest to largest
    try:
        #now we are now smallest to largest y, swap slots 0-1 and 2-3 if necessary to get x in order
        if (sorted_by_y[0][0] < sorted_by_y[1][0]):  #if first x is smaller than second x
            sorted_by_y[0], sorted_by_y[1] = sorted_by_y[1], sorted_by_y[0]
        if (sorted_by_y[2][0] > sorted_by_y[3][0]):  #if first x is smaller than second x
            sorted_by_y[2], sorted_by_y[3] = sorted_by_y[3], sorted_by_y[2]
    except IndexError:
        sorted_by_y = [0, 0, 0, 0, 0]
    return sorted_by_y


def distance_between(coord1, coord2):
    try:
        difference = math.sqrt(
            (math.fabs((coord1[0] - coord2[0]) * (coord1[0] - coord2[0]))) +
            math.fabs((coord1[1] - coord2[1]) * (coord1[1] - coord2[1]))
        )
    except TypeError or ValueError:
        difference = 0
    return difference


def slice_coords(beacon):
    return beacon[0:2]


#ratio references measured in cm
CAM_HEIGHT = 74
CAM_DIST = 160
CAM_ANGLE = 90 - (math.atan(CAM_DIST / CAM_HEIGHT) * 100)
PIX_RS = 403
PIX_LS = 400
PIX_TS = 200
PIX_BS = 342
APPROACH_DIST_CM = 200
#RS = 43.8
#LS = 42.5

ilowH = 0
ihighH = 185
ilowS = 0
ihighS = 125
ilowV = 216
ihighV = 255

WIDTH = 640
HEIGHT = 480


def create_window():
    cv2.namedWindow('image')

    cv2.createTrackbar('lowH', 'image', ilowH, 255, callback)
    cv2.createTrackbar('highH', 'image', ihighH, 255, callback)
    cv2.createTrackbar('lowS', 'image', ilowS, 255, callback)
    cv2.createTrackbar('highS', 'image', ihighS, 255, callback)
    cv2.createTrackbar('lowV', 'image', ilowV, 255, callback)
    cv2.createTrackbar('highV', 'image', ihighV, 255, callback)


def start_capture():
    cap = cv2.VideoCapture(0)
    cap.set(3, WIDTH)  # width
    cap.set(4, HEIGHT)  # height
    # cap.set(CV_CAP_PROP_EXPOSURE, 0.0)
    return cap


def process_frame(frame, (low_h, high_h), (low_s, high_s), (low_v, high_v)):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_hsv = np.array([low_h, low_s, low_v])
    higher_hsv = np.array([high_h, high_s, high_v])
    mask = cv2.inRange(frame, lower_hsv, higher_hsv)
    _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    return contours, mask


def main():
    cap = start_capture()
    create_window()

    while True:
        with Timer() as t:
            # grab the frame
            ret, frame = cap.read()
            frame = cv2.UMat(frame)  # enable OpenCL processing
            original = frame

            # get trackbar positions
            h = cv2.getTrackbarPos('lowH', 'image'), cv2.getTrackbarPos('highH', 'image')
            s = cv2.getTrackbarPos('lowS', 'image'), cv2.getTrackbarPos('highS', 'image')
            v = cv2.getTrackbarPos('lowV', 'image'), cv2.getTrackbarPos('highV', 'image')

            # filter out the hsv values for the "beacons" we want and
            blank_slate = cv2.inRange(frame, 0, 0)
            contours, mask = process_frame(frame, h, s, v)
            num_contour = 0
            sort_rect = []
            for c in contours:
                if cv2.contourArea(c) < 40:
                    continue
                rect = cv2.boundingRect(c)
                x, y, w, h = rect
                cv2.rectangle(original, (x, y), (x + w, y + h), (0, 255, 0), 2)
                num_contour += 1
                data = [x, y, w, h, w * h]
                sort_rect.append(data)

            # filters all detected rectangles for only the 4 largest
            beacons = filter_rectangles(sort_rect)
            # get bounded rect coords before we convert to cartesian (makes math easier).
            bounded_rect_cords = get_bounded_rect_coords(beacons)
            # now convert to cartesian where (0,0) is at the center of the cameras field of view. OpenCV defaults to 0,0 in upper left corner
            cartesian_coords = convert_to_cartesian(bounded_rect_cords)
            # now that we have exact coordinates we can identify which is which by ordering them (upper right, upper left, lower left, lower right)
            sorted_beacons = sort_beacons(cartesian_coords)
            # print sorted_beacons

            # if by this point we have not identified all the beacons (in this case its 4) then we can't proceed with the transform
            if len(sorted_beacons) != 4:
                print 'no image lock'
            else:
                # to perform the cv2.PnP transform OpenCV needs the default coordinate settings so we revert the sorted beacons back to default coordinate system
                target = cartesian_to_default(sorted_beacons)

                # Camera settings these basically assume a perfect projection plane and will need to be calibrated. just fillers for now
                focal_length = 510
                camera_matrix = np.array(
                    [(focal_length, 0, WIDTH / 2), (0, focal_length, HEIGHT / 2), (0, 0, 1)], dtype="double"
                )

                # print camera_matrix

                # 2D points (use target)
                target_points = np.array(
                    [
                        (target[0][0], target[0][1]), (target[1][0], target[1][1]), (target[2][0], target[2][1]),
                        (target[3][0], target[3][1])
                    ],
                    dtype="double"
                )

                # small balls
                # # 3D points (arbitrary reference frame. Measured dimensions in cm with upperright as origin
                model_points = np.array(
                    [
                        (0, 0, 0),  # upper right
                        (-24.5, 0, 0),  # upper left
                        (-24.5, -12.0, 43.8),  # lower left
                        (0, -12.0, 43.8)  # lower right
                    ],
                    dtype="double"
                )

                # big balls
                # # 3D points (arbitrary reference frame. Measured dimensions in cm with upperright as origin
                # model_points = np.array([
                #     (0, 0, 0),  # upper right
                #     (-59.7, 0, 0),  # upper left
                #     (-59.7, -39.4, 86.3),  # lower left
                #     (0, -39.4, 87) # lower right
                # ], dtype="double")

                distortion_coeffs = np.zeros(
                    (5, 1), dtype="double"
                )  # assume zero distortion...this is practically the case
                # distortion_coeffs =  np.array([#got these from running calibration.py
                #     (.13006041, .16734663,.00304074,-.01473097,-1.71723664)
                # ], dtype="double")

                # MAGIC HAPPENS HERE. fortunately cv3 has a perspective transform. which allows us to extract the rotation_vector and translation_vector
                # note that the translation vector is from the center of the cameras frame to the camera at the same depth as the 0,0,0 beacon.

                (success, rotation_vector,
                 translation_vector) = cv2.solvePnP(model_points, target_points, camera_matrix, distortion_coeffs)

                # adds the bgr axes to the original image because fancy
                (zAxis, jacobian) = cv2.projectPoints(
                    np.array([(0.0, 0.0, 30.0)]), rotation_vector, translation_vector, camera_matrix, distortion_coeffs
                )
                (yAxis, jacobian) = cv2.projectPoints(
                    np.array([(0.0, 30.0, 0.0)]), rotation_vector, translation_vector, camera_matrix, distortion_coeffs
                )
                (xAxis, jacobian) = cv2.projectPoints(
                    np.array([(30.0, 0.0, 0.0)]), rotation_vector, translation_vector, camera_matrix, distortion_coeffs
                )
                p1z = (int(target_points[0][0]), int(target_points[0][1]))
                p2z = (int(zAxis[0][0][0]), int(zAxis[0][0][1]))
                cv2.line(original, p1z, p2z, (255, 0, 0), 2)  # (blue)
                p1y = (int(target_points[0][0]), int(target_points[0][1]))
                p2y = (int(yAxis[0][0][0]), int(yAxis[0][0][1]))
                cv2.line(original, p1y, p2y, (0, 255, 0), 2)  # (green)
                p1x = (int(target_points[0][0]), int(target_points[0][1]))
                p2x = (int(xAxis[0][0][0]), int(xAxis[0][0][1]))
                cv2.line(original, p1x, p2x, (0, 0, 255), 2)  # (red)

                # this just traces a rectangle around the target so we know if sort_beacons worked right.
                center = [0, 0]
                cv2.line(blank_slate, (target[0][0], target[0][1]), (target[1][0], target[1][1]), (255, 255, 255), 2)
                cv2.line(blank_slate, (target[1][0], target[1][1]), (target[2][0], target[2][1]), (255, 255, 255), 2)
                cv2.line(blank_slate, (target[2][0], target[2][1]), (target[3][0], target[3][1]), (255, 255, 255), 2)
                cv2.line(blank_slate, (target[3][0], target[3][1]), (target[0][0], target[0][1]), (255, 255, 255), 2)

                # adjust rotation_matrix
                camera_angle = .486
                rotation_vector[0] -= np.pi * -1  # flip x to correct orientation
                # rotation_vector[2] *= -1 #flip our z axis so that positive is facing the camera

                x_theta = -camera_angle  # add gimbals reported camera angle to x
                z_theta = 0  # rotation_vector[1]
                y_theta = 0  # rotation_vector[2]

                a = translation_vector[0]
                b = translation_vector[1]
                c = translation_vector[2]

                # print "no rotation" , (a, b, c)
                a1 = a * math.cos(z_theta) - b * math.sin(z_theta)
                b1 = a * math.sin(z_theta) + b * math.cos(z_theta)
                c1 = c
                # print "x axis rotation", (a1, b1, c1)

                c2 = c1 * math.cos(y_theta) - a1 * math.sin(y_theta)
                a2 = c1 * math.sin(y_theta) + a1 * math.cos(y_theta)
                b2 = b1
                # print "y axis rotation", (a2, b2, c2)

                b3 = b2 * math.cos(x_theta) - c2 * math.sin(x_theta)
                c3 = b2 * math.sin(x_theta) + c2 * math.cos(x_theta)
                a3 = a2

                a3 = (math.atan(rotation_vector[2]) * c3) - translation_vector[0]

                # print "z axis rotation", (a3, b3, c3)

                translation = calculate_translation(rotation_vector[2], translation_vector[2], APPROACH_DIST_CM)
                print "translation = ", translation
                # print "(a3, b3, c3) = ", (a3, b3, c3)
                print "rotation_vector", format(np.rad2deg(rotation_vector))  # radians (x,y,z)
                print "translation_vector", format(
                    translation_vector
                )  # calibrated to be cm. this was done in the camera calibration matrix

        print "Main loop took %sms" % t.interval
        # Just adds extra displays to make real-time tuning and debugging easier
        lineThickness = 1
        cv2.line(original, (0, HEIGHT / 2), (WIDTH, HEIGHT / 2), (100, 100, 0), lineThickness)
        cv2.line(original, (WIDTH / 2, 0), (WIDTH / 2, HEIGHT), (100, 100, 0), lineThickness)
        frame = cv2.bitwise_and(frame, frame, mask=mask)
        # cv2.imshow('blank_slate', blank_slate)
        cv2.imshow('mask', mask)
        cv2.imshow('image', original)
        k = cv2.waitKey(1) & 0xFF  # set frame refresh here
        if k == 113 or k == 27:
            break

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
