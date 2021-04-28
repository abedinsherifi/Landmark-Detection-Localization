"""
Abedin Sherifi
RBE595
HW4 Coding Assignment
10/31/2020
"""


"""
Imports
"""
import SLAM
import cv2

cap = cv2.VideoCapture("Boston.mp4")
count = 0

def Features(frame):

    # Initiating ORB
    orb = cv2.ORB_create()

    # Finding the kpts and descs
    kpt, desc = orb.detectAndCompute(frame, None)

    # Drawing kpts
    img2 = cv2.drawKeypoints(frame, kpt, None, color=(255, 0, 0), flags=0)

    #Display results on screen
    cv2.imshow('Mono SLAM', img2)

# Read until video is completed
while(cap.isOpened()):
  ret, frame = cap.read()
  if ret == True:

    count = count + 1
    frame = cv2.resize(frame, (800, 600))

    Features(frame)

    if count >= 10:
      SLAM.Run_Slam(frame)
      count = 0

    if cv2.waitKey(1) & 0xFF == ord('q'):
      break

  else:
    break

# Point cloud map generated for all the previous frames
print("Building Point Cloud Map")
SLAM.Map_Generator()

# Closes all the frames
cv2.destroyAllWindows()




