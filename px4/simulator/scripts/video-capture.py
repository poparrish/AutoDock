#!/usr/bin/env python2
import cv2

# capture simulator GStream video
gst = "udpsrc port=5600 caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264\" ! rtph264depay ! avdec_h264 ! videoconvert ! appsink"


def start():
    cap = cv2.VideoCapture(gst)
    print(cap.isOpened())
    print(cv2.getBuildInformation())

    while True:
        ret, frame = cap.read()

        if not ret:
            print "No image"
            continue

        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    print "Starting video capture, this may take a moment..."
    start()
