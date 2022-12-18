import cv2 as cv
import numpy as np
import sys
import math

def read_camera_parameters(filepath = 'camera_parameters/intrinsic.dat'):

    inf = open(filepath, 'r')

    cmtx = []
    dist = []

    #ignore first line
    line = inf.readline()
    for _ in range(3):
        line = inf.readline().split()
        line = [float(en) for en in line]
        cmtx.append(line)

    #ignore line that says "distortion"
    line = inf.readline()
    line = inf.readline().split()
    line = [float(en) for en in line]
    dist.append(line)

    #cmtx = camera matrix, dist = distortion parameters
    return np.array(cmtx), np.array(dist)

def get_qr_coords(cmtx, dist, points):

    #Selected coordinate points for each corner of QR code.
    qr_edges = np.array([[0,0,0],
                         [0,1,0],
                         [1,1,0],
                         [1,0,0]], dtype = 'float32').reshape((4,1,3))

    #determine the orientation of QR code coordinate system with respect to camera coorindate system.
    ret, rvec, tvec = cv.solvePnP(qr_edges, points, cmtx, dist)

    #Define unit xyz axes. These are then projected to camera view using the rotation matrix and translation vector.
    unitv_points = np.array([[0,0,0], [1,0,0], [0,1,0], [0,0,1]], dtype = 'float32').reshape((4,1,3))
    if ret:
        points, jac = cv.projectPoints(unitv_points, rvec, tvec, cmtx, dist)
        return points, rvec, tvec

    #return empty arrays if rotation and translation values not found
    else: return [], [], []


def get_orientation(cmtx, dist):

    ret, img = cap.read()
    if ret == False: return None

    ret_qr, points = qr.detect(img)
    cv.imshow('frame', img)

    if ret_qr:
        axis_points, rvec, tvec = get_qr_coords(cmtx, dist, points)

        #check axes points are projected to camera view.
        if len(axis_points) > 0:
            axis_points = axis_points.reshape((4,2))

            x1 = axis_points[1][0]
            y1 = axis_points[1][1]
            x0 = axis_points[0][0]
            y0 = axis_points[0][1]

            angle = math.atan2(x0-x1,y0-y1)
            angle_dg = -(math.floor((-180-math.degrees(angle))*100)/100)
            return angle_dg



if __name__ == '__main__':

    #read camera intrinsic parameters.
    cmtx, dist = read_camera_parameters()

    input_source = 'media/test.mp4'
    if len(sys.argv) > 1:
        input_source = int(sys.argv[1])

    cap = cv.VideoCapture(input_source)
    qr = cv.QRCodeDetector()
    angle_dg = 0

    while True:
        angle_dg = get_orientation(cmtx, dist)
        print('Angle: ', angle_dg)
        
        k = cv.waitKey(20)
        if k == 27: break #27 is ESC key.

    cap.release()
    cv.destroyAllWindows()
