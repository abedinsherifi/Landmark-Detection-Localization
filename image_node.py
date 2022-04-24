#! /usr/bin/python

'''
Abedin Sherifi
RBE594
'''

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import math
import numpy as np
import pandas as pd
import time
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d as plt3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
from std_msgs.msg import Float64MultiArray

bridge = CvBridge()
image_number = 0

x_poss = []
y_poss = []
z_poss = []

kpt1 = []
desc1 = []
camera_pos = [0, 0, 0]
camera_xyz = []
point_cloud_xyz = []
landmarks_xyz = []
landmarks_xx = []
depth_factor = 10
F = []

p1 = []
p2 = []
p3 = []
p4 = []
p5 = []
p6 = []
p7 = []
p8 = []

def image_callback(msg):

    global image_number
    print("Received an image!")
    try:
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError:
        print('Error')
    else:
        #cv2.imwrite('camera_image.jpeg', cv2_img)
        #pnt1, pnt2 = Data_Matching(cv2_img)
        #print(pnt1)
        #height, width, layers = cv2_img.shape
        #print(f'height:{height},width:{width}')
        #(height, width) = (480, 640)
        
        Run_Slam(cv2_img)
        
        #orb = cv2.ORB_create()
        #kpt, desc = orb.detectAndCompute(cv2_img, None)
        #img2 = cv2.drawKeypoints(cv2_img, kpt, None, color=(12, 232, 12), flags=0)
        #path_kpts = '/home/dino/framess'
        
        #cv2.imwrite(os.path.join(path_kpts,'frame%06i.png' % image_number), img2)
        
        #path = '/home/dino/frames'
        #cv2.imwrite(os.path.join(path,'frame%06i.png' % image_number), cv2_img)
        #image_number = image_number + 1

def pose_callback(data):
    pos_cur = data
    global x_pos, y_pos, z_pos
    global x_poss, y_poss, z_poss
    x_pos = data.position.x
    y_pos = data.position.y
    z_pos = data.position.z
    x_poss.append(data.position.x)
    y_poss.append(data.position.y)
    z_poss.append(data.position.z)
    print(f'Firefly Position X:{data.position.x} Y:{data.position.y} Z:{data.position.z}')

def listen_to_groundtruth_pos():
    msg = PoseStamped()
    msg_pose = Pose()
    rospy.Subscriber('/firefly/ground_truth/pose', Pose, pose_callback)
    
    try:
        pub = rospy.Publisher('landmarks', Float64MultiArray, queue_size=10)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
        	if len(p1) and len(p2) and len(p3) and len(p4) and len(p5) and len(p6) and len(p7) and len(p8):
        	        obs_points = Float64MultiArray()
        	        obs_points.data = [p1, p2, p3, p4, p5, p6, p7, p8]
        	        rospy.loginfo(obs_points)
        	        pub.publish(obs_points)
        	        rate.sleep()
    except rospy.ROSInterruptException:
            pass
    rospy.spin()

def Run_Slam(currFrame):
        pnt1, pnt2 = Data_Matching(currFrame)
        if pnt1 and pnt2:
            Reconstruction(pnt1, pnt2)

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
        #image_number = 0

        if kpt1 != [] and kpt2 != []:
            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            matches = bf.match(desc1, desc2)
            matches = sorted(matches, key=lambda x: x.distance)
            for mat in matches[:20]:

                img1_idx = mat.queryIdx
                img2_idx = mat.trainIdx

                (x1, y1) = kpt1[img1_idx].pt
                (x2, y2) = kpt2[img2_idx].pt
                pnt1.append([x1, y1])
                pnt2.append([x2, y2])
                
            #img_matches = cv2.drawMatches(currFrame,kpt1,currFrame-1,kpt2,matches[:],None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
            #plt.imshow(img_matches),plt.show()
            #path_kpts = '/home/dino/frames'
            #cv2.imwrite(os.path.join(path_kpts,'frame%06i.png' % image_number), img_matches)
            #image_number = image_number + 1
        
        kpt1 = kpt2
        desc1 = desc2
        return pnt1, pnt2
	

def Reconstruction(pnt1, pnt2):
        global camera_pos
        global camera_xyz
        global point_cloud_xyz
        global F
        global landmarks_xyz
        global landmarks_xx
        
        cx = width / 2
        cy = height / 2

        I = np.array([[493, 0, cx],
                      [0, 493, cy],
                      [0, 0, 1]])
        
        F, mask = cv2.findFundamentalMat(np.float32(pnt2), np.float32(pnt1), cv2.FM_RANSAC)
        
        points, R, T, mask = cv2.recoverPose(F, np.float32(pnt2), np.float32(pnt1), I)
        
        R = np.asmatrix(R).I
        
        camera_xyz.append([x_pos, y_pos, z_pos])
        
        E = np.hstack((R, T))

        for k in range(len(pnt2)):
            pts2d = np.asmatrix([pnt2[k][0], pnt2[k][1], 1]).T
            P = np.asmatrix(I) * np.asmatrix(E)
            pts3d = np.asmatrix(P).I * pts2d
            point_cloud_xyz.append([pts2d[0] + x_pos,
                                pts2d[1] + y_pos,
                                abs(pts3d[2][0] * 2) + z_pos])
        
        camera_pos = [x_pos, y_pos, z_pos]
        landmarks_xyz = np.array(point_cloud_xyz)

        #landmarks_x = landmarks_xyz[:,0,0]
        #landmarks_y = landmarks_xyz[:,1,0]
        #landmarks_z = landmarks_xyz[:,2,0]
        
        landmarks_x = landmarks_xyz[0][0][0][0]
        landmarks_xx = landmarks_x.tolist()
        landmarks_y = landmarks_xyz[0][1][0][0]
        landmarks_z = landmarks_xyz[0][2][0][0]
        print(f'Landmark (x,y,z):({landmarks_x},{landmarks_y},{landmarks_z})')
        
        vehicle_pos = np.array([x_pos, y_pos, z_pos])
        euclidean_dist(vehicle_pos, landmarks_xyz)
        
def plot_traj(x_poss, y_poss, z_poss):
	fig_traj_only = plt.figure(2)
	ax_traj = Axes3D(fig_traj_only)
	ax_traj.plot(x_poss, y_poss, z_poss, color='b', linewidth=2)
	ax_traj.set_xlabel('Pos X')
	ax_traj.set_ylabel('Pos Y')
	ax_traj.set_zlabel('Pos Z')
	fig_traj_only.suptitle('Drone Path Trajectory', fontsize=16)
	trajectory_fig = plt.gcf()
	plt.show()
	plt.draw()
	trajectory_fig.savefig('trajectory.png', dpi=100)

def plot_landmarks(landmarks_xyz, x_poss, y_poss, z_poss, p1, p2, p3, p4, p5, p6, p7, p8):
	fig_landmarks_only = plt.figure(3)
	ax_landmark = Axes3D(fig_landmarks_only)
	ax_landmark.scatter(landmarks_xyz[:, [0]], landmarks_xyz[:, [1]], landmarks_xyz[:, [2]], s = 5, c='g')
	ax_landmark.scatter(x_poss, y_poss, z_poss, s = 100, c='c')
	ax_landmark.plot(x_poss, y_poss, z_poss, color='b', linewidth=2)
	plot_obstacles(p1,p2,p3,p4,p5,p6,p7,p8,ax_landmark)
	ax_landmark.set_xlabel('Pos X')
	ax_landmark.set_ylabel('Pos Y')
	ax_landmark.set_zlabel('Pos Z')
	fig_landmarks_only.suptitle('Landmarks Localization', fontsize=16)
	landmarks_fig = plt.gcf()
	plt.show()
	plt.draw()
	landmarks_fig.savefig('landmarks.png', dpi=100)
	
def plot_obstacles(p1,p2,p3,p4,p5,p6,p7,p8, fig):

        edges = [
            [p1, p2, p5, p3],
            [p1, p2, p6, p4],
            [p4, p6, p8, p7],
            [p7, p8, p5, p3],
            [p1, p3, p7, p4],
            [p2, p5, p8, p6]
        ]

        # Plot cube
        faces = Poly3DCollection(edges, linewidths=1, edgecolors='k')
        faces.set_facecolor((1, 0.5, 0, 0.3))

        fig.add_collection3d(faces)

def Map_Generator():
        plot_traj(x_poss, y_poss, z_poss)
        plot_landmarks(landmarks_xyz, x_poss, y_poss, z_poss, p1, p2, p3, p4, p5, p6, p7, p8)
        
def euclidean_dist(vehicle_pos, landmark_pos):

	global p1, p2, p3, p4, p5, p6, p7, p8
	dist3d = np.linalg.norm(vehicle_pos-landmark_pos[0])
	#print(f'Euclidean Distance:{dist3d}')
	if dist3d <= 100:
		#print(f'Eucliean Distance:{dist3d}\nLandmark Pose:{landmark_pos}')
		p8 = [landmark_pos[0][0][0][0] + 50, landmark_pos[0][1][0][0] + 50, landmark_pos[0][2][0][0] + 50]
		p7 = [landmark_pos[0][0][0][0] - 50, landmark_pos[0][1][0][0] + 50, landmark_pos[0][2][0][0] + 50]
		p4 = [landmark_pos[0][0][0][0] - 50, landmark_pos[0][1][0][0] - 50, landmark_pos[0][2][0][0] + 50]
		p6 = [landmark_pos[0][0][0][0] + 50, landmark_pos[0][1][0][0] - 50, landmark_pos[0][2][0][0] + 50]
		p5 = [landmark_pos[0][0][0][0] + 50, landmark_pos[0][1][0][0] + 50, landmark_pos[0][2][0][0] - 50]
		p3 = [landmark_pos[0][0][0][0] - 50, landmark_pos[0][1][0][0] + 50, landmark_pos[0][2][0][0] - 50]
		p1 = [landmark_pos[0][0][0][0] - 50, landmark_pos[0][1][0][0] - 50, landmark_pos[0][2][0][0] - 50]
		p2 = [landmark_pos[0][0][0][0] + 50, landmark_pos[0][1][0][0] - 50, landmark_pos[0][2][0][0] - 50]
		#return print(f'Eucliean Distance:{dist3d}\nLandmark Pose:{landmark_pos}')
		
		return print(f'p1:{p1}\np2:{p2}\np3:{p3}\np4:{p4}\np5:{p5}\np6:{p6}\np7:{p7}\np8:{p8}')
		
		
def main():
	rospy.init_node('image_node')
	image_topic = "/firefly/vi_sensor/camera_depth/camera/image_raw"
	rospy.Subscriber(image_topic, Image, image_callback)
	listen_to_groundtruth_pos()
	Map_Generator()
	rospy.spin()

if __name__ == '__main__':
    main()
