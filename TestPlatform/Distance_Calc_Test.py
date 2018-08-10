import cv2
import numpy as np
import math
from operator import itemgetter
from PIL import Image
import PIL.ImageOps


def callback(x):
    pass

def filter_rectangles(sort_rect):
    #return 3 largest rectangles
    sorted_area = sorted(sort_rect, key = itemgetter(4))
    sorted_area.reverse()
    return_rectangles = []
    if len(sorted_area) > 4:
        for i in sorted_area:
            sorted_area = sorted_area[:-1]
            if len(sorted_area) == 4:
                return_rectangles = sorted_area
                break
    return_rectangles = sorted_area
    return return_rectangles

def convert_to_cartesian(beacons):
    for beacon in beacons:
        #convert x
        if beacon[0] > WIDTH/2:
            beacon[0] = beacon[0]-WIDTH/2
        elif beacon[0] < WIDTH/2:
            beacon[0] = WIDTH/2-beacon[0]
            beacon[0] *= -1
        else:
            beacon[0] = 0
        #convert y
        if beacon[1] > HEIGHT/2:
            beacon[1] = beacon[1]-HEIGHT/2
            beacon[1] *= -1
        elif beacon[1] < HEIGHT/2:
            beacon[1] = HEIGHT/2-beacon[1]
        else:
            beacon[1] = 0
    cartesian_coords = beacons
    return cartesian_coords

def get_bounded_rect_coords(cartesian_beacons):
    for coords in cartesian_beacons:
        xcoord = coords[0] + coords[2]/2
        ycoord = coords[1] + coords[3]/2
        coords[0] = xcoord
        coords[1] = ycoord
    bounded_rect_cords = cartesian_beacons
    return bounded_rect_cords

def sort_beacons(cartesian_coords):
    sorted_by_y = sorted(cartesian_coords, key=itemgetter(1))
    sorted_by_y.reverse()
    try:
        #now we are now smallest to largest y, swap slots 0-1 and 2-3 if necessary to get x in order
        if(sorted_by_y[0][0] > sorted_by_y[0][1]):#if first x is smaller than second x
            sorted_by_y[0], sorted_by_y[1] = sorted_by_y[1], sorted_by_y[0]
        if(sorted_by_y[0][2] > sorted_by_y[0][3]):#if first x is smaller than second x
            sorted_by_y[2], sorted_by_y[3] = sorted_by_y[3], sorted_by_y[2]
    except IndexError:
        sorted_by_y = [0,0,0,0]
    return sorted_by_y

def distance_between(coord1, coord2):
    try:
        difference = math.sqrt((math.fabs((coord1[0]-coord2[0])*(coord1[0]-coord2[0]))) + math.fabs((coord1[1]-coord2[1])*(coord1[1]-coord2[1])))
    except TypeError or ValueError:
        difference = 0
    return difference

#ratio references measured in cm
CAM_HEIGHT = 60
CAM_DIST = 115
CAM_ANGLE = 90 - (math.atan(CAM_DIST/CAM_HEIGHT) * 100)
PIX_RS = 181
PIX_LS = 184
PIX_TS = 109
PIX_BS = 138

print 'camangle'
print CAM_ANGLE
cap = cv2.VideoCapture(1)
cv2.namedWindow('image')
WIDTH = 640
HEIGHT = 480
cap.set(3, WIDTH)#width
cap.set(4, HEIGHT)#height
ilowH = 52
ihighH = 141
ilowS = 26
ihighS = 93
ilowV = 152
ihighV = 255
# create trackbars for color change
cv2.createTrackbar('lowH','image',ilowH,179,callback)
cv2.createTrackbar('highH','image',ihighH,179,callback)
cv2.createTrackbar('lowS','image',ilowS,255,callback)
cv2.createTrackbar('highS','image',ihighS,255,callback)
cv2.createTrackbar('lowV','image',ilowV,255,callback)
cv2.createTrackbar('highV','image',ihighV,255,callback)



while(True):
    # grab the frame
    ret, frame = cap.read()


    original = frame

    # get trackbar positions
    ilowH = cv2.getTrackbarPos('lowH', 'image')
    ihighH = cv2.getTrackbarPos('highH', 'image')
    ilowS = cv2.getTrackbarPos('lowS', 'image')
    ihighS = cv2.getTrackbarPos('highS', 'image')
    ilowV = cv2.getTrackbarPos('lowV', 'image')
    ihighV = cv2.getTrackbarPos('highV', 'image')

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_hsv = np.array([ilowH, ilowS, ilowV])
    higher_hsv = np.array([ihighH, ihighS, ihighV])
    mask = cv2.inRange(frame, lower_hsv, higher_hsv)
    _, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    num_contour = 0
    sort_rect = []
    for c in contours:
        if cv2.contourArea(c) < 40:
            continue
        rect = cv2.boundingRect(c)
        x, y, w, h = rect
        cv2.rectangle(original, (x, y), (x + w, y + h), (0, 255, 0), 2)
        num_contour += 1
        xcoord = x
        ycoord = y
        data = [x,y,w,h,w*h]
        sort_rect.append(data)


    #filters all detected rectangles for only the 4 largest
    beacons = filter_rectangles(sort_rect)
    #get bounded rect coords before we convert to cartesian (makes math easier)
    bounded_rect_cords = get_bounded_rect_coords(beacons)
    #now convert to cartesian where (0,0) is at the center of the cameras field of view
    cartesian_coords = convert_to_cartesian(bounded_rect_cords)
    #now that we have exact coordinates we can identify which is which by ordering them (upper right, upper left, lower left, lower right)
    sorted_beacons = sort_beacons(cartesian_coords)
    #STEPS
    #1 make a method that calculates distance between two bounded_rect_coords inputs

    #1.1 make a method that orders the points
    #1.2 determine focal length
    #2 solve location on the y-axis
    #3 solve location on the x & yaw axis (birds eye view think theta_dot and trans)

    try:
        RS = distance_between(sorted_beacons[0], sorted_beacons[3])
        LS = distance_between(sorted_beacons[1], sorted_beacons[2])
        TS = distance_between(sorted_beacons[0], sorted_beacons[1])
        BS = distance_between(sorted_beacons[2], sorted_beacons[3])
    except IndexError or ValueError or TypeError:
        RS = 0
        LS = 0
        TS = 0
        BS = 0


    print num_contour
    print sorted_beacons
    print RS, LS, TS, BS,





    # add changes to the images being outputted
    lineThickness = 1
    cv2.line(original, (0, HEIGHT/2),(WIDTH,HEIGHT/2), (100,100,0), lineThickness)
    cv2.line(original, (WIDTH/2, 0), (WIDTH/2, HEIGHT), (100,100,0), lineThickness)
    frame = cv2.bitwise_and(frame, frame, mask=mask)
    cv2.imshow('mask', mask)
    cv2.imshow('image', original)
    k = cv2.waitKey(1) & 0xFF  # large wait time to remove freezing
    if k == 113 or k == 27:
        break

cv2.destroyAllWindows()