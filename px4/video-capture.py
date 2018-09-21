import time

import cv2

cap = cv2.VideoCapture("udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink fps-update-interval=1000 sync=false")

# gst-launch-1.0  -v udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink fps-update-interval=1000 sync=false

# cap = cv2.VideoCapture('udp://@172.19.0.2:5600')

print(cap.isOpened())
print(cv2.getBuildInformation())

while True:

    ret, frame = cap.read()
    print ret, frame
    time.sleep(0.5)
