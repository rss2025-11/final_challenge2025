import sys
import math
import cv2 as cv
import numpy as np


def create_homography_matrix():
    # PTS_IMAGE_PLANE = [[544, 257], [155, 207], [304, 196], [545, 193], [237, 177], [269, 196], [398, 168]]  
    PTS_IMAGE_PLANE = [[408, 333], [287, 215], [214, 219], [237, 190], [376, 192], [462, 231], [469, 203]]  
    # PTS_GROUND_PLANE = [[26.25, -14.5], [44.75, 21], [61, 2.25], [96.75, 17.75], [58.5, -38.25] ,[120.75,12.25], [123, -31.75]]  # dummy points
    PTS_GROUND_PLANE = [[37.5, -3.5], [107, 15], [100, 35.5], [174, 26.75], [164.5, -22.5] ,[87, -29.5], [134.5, -51.5]]  # dummy points
    
    METERS_PER_CM = 0.01

    METERS_PER_INCH = 0.0254

    if not len(PTS_GROUND_PLANE) == len(PTS_IMAGE_PLANE):
        print(
            "ERROR: PTS_GROUND_PLANE and PTS_IMAGE_PLANE should be of same length"
        )

    # Initialize data into a homography matrix

    np_pts_ground = np.array(PTS_GROUND_PLANE)
    np_pts_ground = np_pts_ground * METERS_PER_CM
    np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

    np_pts_image = np.array(PTS_IMAGE_PLANE)
    np_pts_image = np_pts_image * 1.0
    np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])
    homography, err = cv.findHomography(np_pts_image, np_pts_ground)
    print("Homography Transformer Initialized")
    return homography
