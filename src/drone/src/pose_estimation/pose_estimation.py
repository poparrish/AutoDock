import argparse

import cv2
import numpy as np
from buildTarget import *
"""
author: parker, nathan, wankun
this is a proof-of-concept script that identifies a potential "landing platform" and  uses perspective transforms to calculate
the pose (position/orientation). These are the euler angles and the Translation matrix. 
"""

def callback(x):
    # the cv2.createTrackbar() requires callback param
    pass

def filter_rectangles(sort_rect, num_rect):
    # return 4 largest rectangles
    sorted_area = sorted(sort_rect, key=itemgetter(4))
    sorted_area.reverse()
    if len(sorted_area) > num_rect:
        for i in sorted_area:
            sorted_area = sorted_area[:-1]
            if len(sorted_area) == num_rect:
                break
    return_rectangles = sorted_area
    return return_rectangles

def get_bounded_rect_coords(cartesian_beacons):
    for coords in cartesian_beacons:
        xcoord = coords[0] + coords[2] / 2
        ycoord = coords[1] + coords[3] / 2
        coords[0] = xcoord
        coords[1] = ycoord
    bounded_rect_cords = cartesian_beacons
    return bounded_rect_cords

def slice_coords(beacon):
    return beacon[0:2]

# ratio references measured in cm
CAM_HEIGHT = 74
CAM_DIST = 160
CAM_ANGLE = 90 - (math.atan(CAM_DIST / CAM_HEIGHT) * 100)
PIX_RS = 403
PIX_LS = 400
PIX_TS = 200
PIX_BS = 342
# RS = 43.8
# LS = 42.5

print 'camangle'
print CAM_ANGLE

WIDTH = 640
HEIGHT = 480
APPROACH_DIST_CM = 250

def start_capture(src, resize):
    cap = cv2.VideoCapture(src)
    cv2.namedWindow('image')

    if resize:
        cap.set(3, WIDTH)  # width
        cap.set(4, HEIGHT)  # height

    return cap

def yawpitchrolldecomposition(R):
    sin_x = math.sqrt(R[2,0] * R[2,0] +  R[2,1] * R[2,1])
    validity = sin_x < 1e-6
    if not validity:
        z1    = math.atan2(R[2,0], R[2,1])     # around z1-axis
        x      = math.atan2(sin_x,  R[2,2])     # around x-axis
        z2    = math.atan2(R[0,2], -R[1,2])    # around z2-axis
    else: # gimbal lock
        z1    = 0                                         # around z1-axis
        x      = math.atan2(sin_x,  R[2,2])     # around x-axis
        z2    = 0                                         # around z2-axis

    return np.array([[z1], [x], [z2]])


def estimate_pose(frame, calibration):
    original = frame

    # filter out the hsv values for the "beacons" we want and create bounding rectangles
    lower_hsv = np.array([calibration[k][0] for k in 'hsv'])
    higher_hsv = np.array([calibration[k][1] for k in 'hsv'])
    mask = cv2.inRange(frame, lower_hsv, higher_hsv)
    _, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    num_contour = 0
    sort_rect = []
    for c in contours:
        if cv2.contourArea(c) < 1:
            continue
        rect = cv2.boundingRect(c)
        x, y, w, h = rect
        cv2.rectangle(original, (x, y), (x + w, y + h), (0, 255, 0), 2)
        num_contour += 1
        data = [x, y, w, h, w * h]
        sort_rect.append(data)

    # filters all detected rectangles for only the num_rect largest
    num_rect = 7
    beacons = filter_rectangles(sort_rect, num_rect)
    bounded_rect_cords = get_bounded_rect_coords(beacons)
    sliced = []
    for _ in bounded_rect_cords:#we just want x & y get rid of all the other data like area, length,width etc
        sliced.append(_[:2])

    if len(sliced) != 7:
        return None
    else:
        target = getTarget(bounded_rect_cords)
        print 'TARGET: ',target
        # to perform the cv2.PnP transform OpenCV needs the default coordinate settings so we revert the sorted beacons back to default coordinate system

        # Camera settings these basically assume a perfect projection plane and will need to be calibrated. just fillers for now 11/5/18
        focal_length = 510
        camera_matrix = np.array(
            [(focal_length, 0, WIDTH / 2), (0, focal_length, HEIGHT / 2), (0, 0, 1)], dtype="double"
        )

        # 2D points (use target)
        target_points = np.array(
            [
                (target[0][0], target[0][1]), (target[1][0], target[1][1]),
                (target[2][0], target[2][1]), (target[3][0], target[3][1]),
                (target[4][0], target[4][1]), (target[5][0], target[5][1]),
                (target[6][0], target[6][1])
            ],
            dtype="double"
        )
        print 'made it past target points'
        # 3D points (arbitrary reference frame. Measured dimensions in cm with upper middle as origin
        model_points = np.array(
            [
                (-45.7, 0, 86.3), (-48.2, 0, 41.3), (-45.7, 0, 2.5), (0, 0, 0), (45.7, 0, 2.5), (48.2, 0, 41.3),
                (45.7, 0, 86.3)
            ],
            dtype="double"
        )

        distortion_coeffs = np.zeros((5, 1), dtype="double")  # assume zero distortion...this is practically the case
        # distortion_coeffs =  np.array([#got these from running calibration.py
        #     (.13006041, .16734663,.00304074,-.01473097,-1.71723664)
        # ], dtype="double")

        # MAGIC HAPPENS HERE. fortunately cv3 has a perspective transform. which allows us to extract the rotation_vector and translation_vector
        # note that the translation vector is from the center of the cameras frame to the camera at the same depth as the 0,0,0 beacon.
        print 'made it to pose'
        success, rotation_vector, translation_vector = cv2.solvePnP(
            objectPoints=model_points,
            imagePoints=target_points,
            cameraMatrix=camera_matrix,
            distCoeffs=distortion_coeffs
        )
        print 'made it through pose'

        #now take dot product to get drone position relative to the platform
        rodrigues = cv2.Rodrigues(rotation_vector)[0]#get euler angles from rotation matrix
        drone_pos = np.dot(-rodrigues.T, translation_vector)
        euler = yawpitchrolldecomposition(rodrigues)
        #print "drone_pos", drone_pos
        debug = {
            'mask': mask,
            'target': target,
            'target_points': target_points,
            'distortion_coeffs': distortion_coeffs,
            'camera_matrix': camera_matrix
        }
        return drone_pos, euler, target


def update_calibration(calibration):
    # create trackbars for color change
    low_h = cv2.getTrackbarPos('lowH', 'image')
    high_h = cv2.getTrackbarPos('highH', 'image')
    low_s = cv2.getTrackbarPos('lowS', 'image')
    high_s = cv2.getTrackbarPos('highS', 'image')
    low_v = cv2.getTrackbarPos('lowV', 'image')
    high_v = cv2.getTrackbarPos('highV', 'image')
    return {
        'h': (low_h, high_h),
        's': (low_s, high_s),
        'v': (low_v, high_v),
    }


def draw_debug(frame, pose):
    if pose is not None:
        (translation_vector, rotation_vector, debug) = pose
        target = debug['target']
        target_points = debug['target_points']
        distortion_coeffs = debug['distortion_coeffs']
        camera_matrix = debug['camera_matrix']
        mask = debug['mask']

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
        cv2.line(frame, p1z, p2z, (255, 0, 0), 2)  # (blue)
        p1y = (int(target_points[0][0]), int(target_points[0][1]))
        p2y = (int(yAxis[0][0][0]), int(yAxis[0][0][1]))
        cv2.line(frame, p1y, p2y, (0, 255, 0), 2)  # (green)
        p1x = (int(target_points[0][0]), int(target_points[0][1]))
        p2x = (int(xAxis[0][0][0]), int(xAxis[0][0][1]))
        cv2.line(frame, p1x, p2x, (0, 0, 255), 2)  # (red)
        print target
        # Just adds extra displays to make real-time tuning and debugging easier
        lineThickness = 1
        cv2.line(frame, (0, HEIGHT / 2), (WIDTH, HEIGHT / 2), (100, 100, 0), lineThickness)
        cv2.line(frame, (WIDTH / 2, 0), (WIDTH / 2, HEIGHT), (100, 100, 0), lineThickness)
        frame = cv2.bitwise_and(frame, frame, mask=mask)
        cv2.imshow('mask', mask)

    cv2.imshow('frame', frame)
    k = cv2.waitKey(1) & 0xFF  # set frame refresh here
    if k == 113 or k == 27:
        cv2.destroyAllWindows()
        exit(0)


def main(src, resize, calibration):
    print 'Starting capture on device "%s"...' % src
    cap = start_capture(src, resize)


    cv2.createTrackbar('lowH', 'image', calibration['h'][0], 255, callback)
    cv2.createTrackbar('highH', 'image', calibration['h'][1], 255, callback)
    cv2.createTrackbar('lowS', 'image', calibration['s'][0], 255, callback)
    cv2.createTrackbar('highS', 'image', calibration['s'][1], 255, callback)
    cv2.createTrackbar('lowV', 'image', calibration['v'][0], 255, callback)
    cv2.createTrackbar('highV', 'image', calibration['v'][1], 255, callback)

    ret, frame = cap.read()

    if not ret:
        print 'no video'

    try:
        calibration = update_calibration(calibration)
        pose = estimate_pose(frame, calibration)
        return pose
        #draw_debug(frame, pose)
        if pose is None:
            return None
            print 'no image lock'
    except Exception as e:
        print 'failed to reconstruct target %s' % e


    # while True:
    #     ret, frame = cap.read()
    #
    #     if not ret:
    #         print 'no video'
    #         continue
    #
    #     try:
    #         calibration = update_calibration(calibration)
    #         pose = estimate_pose(frame, calibration)
    #         return pose
    #         #draw_debug(frame, pose)
    #         if pose is None:
    #             return None
    #             print 'no image lock'
    #     except Exception as e:
    #
    #         print 'failed to reconstruct target %s' % e



# Default camera calibration
default_calibration = {
    'h': (0, 185),
    's': (0, 125),
    'v': (216, 255),
}

# Used for testing purposes (running w/o ROS)
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--src", help="video source device", type=str, default='0')
    parser.add_argument("-r", "--resize", help="resize video source", type=bool, default=False)

    args = parser.parse_args()

    main(args.src, args.resize, default_calibration)