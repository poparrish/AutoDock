import cv2
from benchmarker import Benchmarker

from Rotation_Matrix.Pose_Estimation import process_frame

cv2.ocl.setUseOpenCL(True)
print "Supports OpenCL: %s\nEnabled: %s\n" % (cv2.ocl.haveOpenCL(), cv2.ocl.useOpenCL())

with Benchmarker(loop=100, extra=2) as bench:
    h = 0, 185
    s = 0, 125
    v = 216, 255

    img = cv2.imread("../Rotation_Matrix/test_images/perspective_transform_3.png", cv2.IMREAD_COLOR)

    @bench("gpu")
    def _(bm):
        frame = cv2.UMat(img)
        for _ in bm:
            process_frame(frame, h, s, v)

    @bench("cpu")
    def _(bm):
        frame = img.copy()
        for _ in bm:
            process_frame(frame, h, s, v)
