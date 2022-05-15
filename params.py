import numpy as np
import open3d as o3d
from cv2 import cv2 as cv
import matplotlib.image as mpimg

### Camera parameters / projection parameters
# If you want to use simualtion data to do the overlay define sim_data as "yes", otherwise if you
# use camera data define as "no". 
# --> If you use simulation data, adapt the projection matrix called mp. 
# There is no need to adapt the following parametes: width_pixels, height_pixels, focal_length, width_sensorsize, height_sensorsize
# 
# --> If you use data from a real camera or real images, you need to adapt the following parametes: 
# width_pixels, height_pixels, focal_length, width_sensorsize, height_sensorsize
sim_data = "yes"

# Name of the topic the overlay node gets its pcd files from
sub_topic_pcd = 'velodyne_points'

# Name of the topic the overlay node gets its image files from
sub_topic_image = 'front'

# Name of the topic under which the overlay image is published --> this topic must be received by RVIZ
pub_topic_overlay = 'front_PCD_overlay'

# Initial Calibration
# Translation according to camera and LIDAR xml file from simulation (new Luminar LiDAR setup!!!, not velodyne)
# Enter the negative vector from LiDAR to camera
if sim_data == "yes":
    # translation = (-0.0743899, 0, -0.0633099)   # Luminar dataset
    translation = (-0.1069789, 0, -0.0470642)   # Dataset 1x1x1 meter dice in FOV   
    rotation = (0,0,0) # The order the rotation is done is z,y',x''
elif sim_data == "no": # If the real camera and LiDAR data is used the initial translations are different
    translation = (-0.71224, 0, 0.06637) # according to TUM wiki
    rotation = (0,0,0) # The order the rotation is done is z,y',x''

# Default user input transformation / rotation --> don't change
usr_translation = (0,0,0) 
usr_rotation = (0,0,0)

### Front camera FOV / Simaltaion FOV
horizontal_FOV_cam = 25 # Total FOV in degrees
vertical_FOV_cam = 19 # Total FOV in degrees

# Front camera / lense parameters according to factsheets 29.01.2021 

# Following parameters need to be defined in case of camera usage
# --> Allied Vision Mako G-319
# --> 12mm UC Series Fixed Focal Lense
# Sensorsize = 1/1.8" --> 7.2*5.4mm
width_sensorsize = 0.0072 # 7,2mm
height_sensorsize = 0.0054 # 5,4mm
focal_length = 0.012 # 12mm

# Following parameters need to be defined in case of simulator data or camera usage
width_pixels = 2064 # horizontal number of pixels from camera image / simulation image
height_pixels = 1544 # vertical number of pixels from camera image / simulation image

# If simulation data is used, enter the projection matrix in the if loop. 
# If camera data is used, enter the camera parameters on top (focal length, sensor size).
# The camera matrix is then calculated automatically in the else loop and converted from mm to pixel units in the code itself.
if sim_data == "yes":
    # Projection matrices (simulation images are not taken by a camera):
    # Projection matrix is valid for the camera xml file 201207_mapping and later datasets
    mp = np.array([[4.496418,0,0,0],
                    [0,6.010755,0,0],
                    [0,0,-1.00001,-1],
                    [0,0,-0.0200001,0,]]).T

    # Projection matrix from the camera xml file ims_one_enemy_200927 and 20201109_dataset 
    # mp = np.array([[3.519355,0,0,0],
    #                 [0,4.70463,0,0],
    #                 [0,0,-1.00001,-1],
    #                 [0,0,-0.0200001,0]]).T

else:
    # Camera matrix (not needed for FTM simulation data)
    mp =np.array ([[focal_length, 0, width_pixels/2],
                [0, focal_length, height_pixels/2],
                [0,0,1]])

# Timestep the overlay is published in!
# --> must be longer than the processing time of the computer
overlay_publish_timestep = 8 # Seconds

# Quality of overlay image
dpi = 250 # Pixels per inch

# Size of the LiDAR points on the overlay
point_size=0.1

# Not displaying pcd points which have a higher distance than pcd_dis_max
# The smaller the maximum distance, the more sensitive is the colorindication of the pcd points to differences in distance. 
pcd_dis_max = 200 # Meters

# Not displaying pcd points which are closer than pcd_dist_min
pcd_dis_min = 1 # Meters

# Slider parameters
# Max / min slider value
max_trans = 0.5 # meters
min_trans = -0.5 # meters
max_deg = 10 # degrees
min_deg = -10 # degrees

resolution_trans = 0.001 # Defines the steprange of the translation sliders in meters
resolution_rot = 0.01 # Defines the steprange of the rotation slider in degrees

width_slider = 800 # Defines the width of the GUI window in pixels