import numpy as np
import cv2

mtx = np.load('mtx.npy')
dist = np.load('dist.npy')


cam = cv2.VideoCapture(0) # Change this value to have same value as other file

# get best camera parameters for undistortion
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
    mtx, dist, (640, 480), 1, (640, 480))

# verify the undistorted video stream
while(1):
    ret_val, img = cam.read()

    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]

    cv2.imshow('uvc_stream', dst)
    if cv2.waitKey(1) == ord('q'):
        break  # q to stop
