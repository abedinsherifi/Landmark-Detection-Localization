![Your Repository's Stats](https://github-readme-stats.vercel.app/api?username=abedinsherifi&show_icons=true)
![Your Repository's Stats](https://github-readme-stats.vercel.app/api/top-langs/?username=abedinsherifi&theme=blue-green)
![](https://komarev.com/ghpvc/?username=abedinsherifi)

<p align="center">
  <a href="https://github.com/prespafree1/Mono-SLAM">
    <img alt="GitHub stars" src="https://img.shields.io/github/stars/prespafree1/Landmark-Detection-Localization.svg">
  </a>
  <a href="https://github.com/prespafree1/Mono-SLAM">
    <img alt="GitHub forks" src="https://img.shields.io/github/forks/prespafree1/Landmark-Detection-Localization.svg">
  </a>
    <a href="https://github.com/prespafree1/Landmark-Detection-Localization/graphs/contributors" alt="Contributors">
        <img src="https://img.shields.io/github/contributors/prespafree1/Landmark-Detection-Localization" /></a>
</p>

# Landmark Detection and Localization
## Introduction
The landmark detection and localization algorithm accepts a frame, converts the frame to open-cv image format. Then, it tries to extract the features using the ORB feature extractor. Top matches are sent to the reconstruction function where we compute the intrinsic matrix, find the fundamental matrix,
recover the relative camera rotation R matrix and translation T vector, compute the extrinsic matrix, and we finally get the 3d points by multiplying the inverse of the projection matrix with the 2d points. <br>

The figure below captures the algorithm pseudo-code: <br>
![](images/Landmark_Detection_Pseudo_Code.png) <br>

Camera calibration is necessary before determining position of camera relative to a landmark feature. In camera calibration, we try to find the intrinsic and extrinsic matrices. The intrinsic matrix is specific to the camera and is made of the focal length and optical centers. <br>

Landmark detection and localization was run in ROS in real time. One thing to note here is that we initially wanted to also do simultaneous camera localization but due to a lot of error accumulation and tracking failures due to sudden motion, we instead just substituted the camera location with the (x,y,z) positions from the Ground Truth Pose topic. <br>

Figures shown below cover few details. In cyan we are depicting the vehicle path. The green dots are the localized landmark features. The orange cube represents the obstacle we generate with the landmark feature point in the center of the cube. We calculate the Euclidean distance from vehicle’s cur-
rent position to the landmark feature points. If this distance is less than or equal to 100 m, then we generate the obstacle cube with the z component 50 m above the landmark feature and 50 m below the landmark feature. We then publish the 8 points of the obstacle cube so that the collision checker can subscribe and use them. <br>

![](images/landmarks.png) <br>

![](images/landmarks1.png) <br>


https://abedinsherifi.github.io/Landmark-Detection-Localization/
