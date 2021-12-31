"""
Abedin Sherifi
"""

"""
Imports
"""
import numpy as np
import cv2
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

"""
Global Variables
"""
kpt1 = []
desc1 = []
camera_pos = [0, 0, 0]
camera_xyz = []
point_cloud_xyz = []
dim = 10

"""
Running SLAM on a single frame. This includes data matching of keypoints as well as reconstruction in 3D.
"""
def Run_Slam(currFrame):
        pnt1, pnt2 = Data_Matching(currFrame)
        if pnt1 and pnt2:
            Reconstruction(pnt1, pnt2)

"""
Finding keypoints/descriptors and matching points. The ORB STAR detector is initiated. ORB is a fusion of FAST kpts and BRIEF descr.
The ORB STAR detector first uses FAST kpt detector then applies Harris Corner Detection.
"""
def Data_Matching(currFrame):
        global kpt1
        global desc1
        global height
        global width

        height, width, ch = currFrame.shape
        orb = cv2.ORB_create()
        kpt2, desc2 = orb.detectAndCompute(currFrame, None)

        pnt1 = []
        pnt2 = []

        if kpt1:
            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            matches = bf.match(desc1, desc2)
            matches = sorted(matches, key=lambda x: x.distance)

            for mat in matches[:100]:

                img1_idx = mat.queryIdx
                img2_idx = mat.trainIdx

                (x1, y1) = kpt1[img1_idx].pt
                (x2, y2) = kpt2[img2_idx].pt

                pnt1.append([x1, y1])
                pnt2.append([x2, y2])

        kpt1 = kpt2
        desc1 = desc2

        return pnt1, pnt2

"""
The point cloud is generated. Points are added to the point cloud individually with no EKF in the loop. Camera pos is calculated through the fundamental matrix and added to the point cloud.
"""
def Reconstruction(pnt1, pnt2):
        global camera_pos
        global camera_xyz
        global point_cloud_xyz
        cx = width / 2
        cy = height / 2

        # Get image cx, cy and focal legth to get intrinsic camera matrix
        # My cell phone camera has focal length of 339
        I = np.array([[339, 0, cx],
                      [0, 339, cy],
                      [0, 0, 1]])
        
        # Find the fundamental matrix using the feature points and the 8-point algorithm through cv2 findFundamentalMat. 
        F, mask = cv2.findFundamentalMat(np.float32(pnt2), np.float32(pnt1), cv2.FM_8POINT)

        # The recoverPose takes in the fundamental and intrinsic matrixes and the 
        # feature points to returns the points that pass the cheirality check and
        # rotation and translation matrix. The chaerality check implies that the triangulated 3D pnts have pos depth.
        points, R, T, mask = cv2.recoverPose(F, np.float32(pnt2), np.float32(pnt1), I)

        # R is equal to the inverse of the recoverPose rotation matirx
        R = np.asmatrix(R).I

        # Appending camera position to camera array
        camera_xyz.append([camera_pos[0] + T[0], camera_pos[1] + T[1], camera_pos[2] + T[2]])

        # Extrinsic matrix
        E = np.hstack((R, T))

        # Going from 2D to 3D and saving to the point cloud array
        for k in range(len(pnt2)):
            pts2d = np.asmatrix([pnt2[k][0], pnt2[k][1], 1]).T
            C = np.asmatrix(I) * np.asmatrix(E)
            pts3d = np.asmatrix(C).I * pts2d
            point_cloud_xyz.append([pts3d[0][0] * dim + camera_pos[0],
                                pts3d[1][0] * dim + camera_pos[1],
                                pts3d[2][0] * dim + camera_pos[2]])

        camera_pos = [camera_pos[0] + T[0], camera_pos[1] + T[1], camera_pos[2] + T[2]]
        print("Current Camera Location Per Frame: %s, %s, %s" % (camera_pos[0], 
                                                        camera_pos[1], camera_pos[2]))

"""
This function prints out the 3-D map, with the point clouds and the camera position.
The dark green is the current camera and will get lighter for the previous positions.
"""
def Map_Generator():
        global point_cloud_xyz
        global camera_xyz

        point_cloud_xyz = np.array(point_cloud_xyz)
        camera_xyz = np.array(camera_xyz)
        fig = plt.figure()
        ax = Axes3D(fig)
        ax.scatter(point_cloud_xyz[:, [0]], point_cloud_xyz[:, [1]], point_cloud_xyz[:, [2]], s = 40, c='r')
        ax.scatter(camera_xyz[:, [0]], camera_xyz[:, [1]], camera_xyz[:, [2]], s = 100, c='g')
        plt.show()
