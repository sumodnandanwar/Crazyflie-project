import numpy as np
import cv2

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*8, 3), np.float32)
objp[:, :2] = np.mgrid[0:8, 0:6].T.reshape(-1, 2)

# arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.

# record video
cam = cv2.VideoCapture(0) #change this value using trial and error to get correct camera, use same value in other file
while(1):
    ret_val, img = cam.read()

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (8, 6), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        imgpoints.append(corners)

        print("Detection cnt: %s" % len(objpoints))

        # corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        # imgpoints.append(corners2)

        cv2.drawChessboardCorners(img, (8, 6), corners, ret)

    cv2.imshow('uvc_stream', img)
    if cv2.waitKey(1) == ord('q'):
        break  # q to stop

print('Processing images...')

cv2.destroyAllWindows()

# run calculatation
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None)

# store your camera parameters, here we used numpy to store as an example.
np.save('mtx.npy', mtx)
np.save('dist.npy', dist)
