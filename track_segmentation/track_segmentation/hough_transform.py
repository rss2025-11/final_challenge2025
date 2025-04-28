"""
@file hough_lines.py
@brief This program demonstrates line finding with the Hough transform
"""
import sys
import math
import cv2 as cv
import numpy as np

def skeletonize(image):
    # Ensure the image is grayscale
    if len(image.shape) == 3:
        image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    
    # Ensure image is binary (black and white)
    _, binary = cv.threshold(image, 127, 255, cv.THRESH_BINARY)
    print(binary)
    # Skeletonize using OpenCV's thinning function
    skeleton = np.zeros_like(binary)
    skeleton = cv.ximgproc.thinning(binary, skeleton, thinningType = cv.ximgproc.THINNING_GUOHALL)

    # # Display the skeletonized image
    # cv.imshow("skeletonized image", skeleton)
    # cv.waitKey(0)  # Wait indefinitely for a key press
    # cv.destroyAllWindows()  # Close all OpenCV windows
    
    return skeleton  # Return the skeletonized image

def edges_clean(image):
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    # Define white color range in HSV
    lower_white = np.array([0, 0, 150], dtype=np.uint8)
    upper_white = np.array([180, 25, 255], dtype=np.uint8)
    # Create mask for white color
    mask_initial = cv.inRange(hsv, lower_white, upper_white)
    _, mask = cv.threshold(mask_initial, 127, 255, cv.THRESH_BINARY)

    # Apply erosion to thin thick lines slightly
    kernel = cv.getStructuringElement(cv.MORPH_RECT, (1, 1))
    thinned_mask = skeletonize(mask)#cv.erode(mask, kernel, iterations=1)

    # Crop the lower part of the image
    cropped_dst = thinned_mask[len(thinned_mask[0])//5:, :]
    # cropped_dst = mask[len(mask[0])//5:, :]

    # Convert to BGR for visualization
    linesP = cv.HoughLinesP(cropped_dst, 1, np.pi / 180, 50, None, 100, 50) # TODO: Tune these numbers if needed

    # # For visualization only:
    cdstP_vis = cv.cvtColor(cropped_dst, cv.COLOR_GRAY2BGR)
    # if linesP is not None:
    #     for i in range(0, len(linesP)):
    #         l = linesP[i][0]
    #         cv.line(cdstP_vis, (l[0], l[1]), (l[2], l[3]), (0,0,255), 1, cv.LINE_AA)

    # cv.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP_vis)
    # cv.waitKey()
    # return 0
    return cdstP_vis, linesP



def compute_lane_edges(lines, image_width, image):
    # lines shape: (N, 1, 4), where 4 = (x1, y1, x2, y2)
    lines = np.squeeze(lines)  # (N, 4)

    # Compute horizontal spread (|x2 - x1|) for all lines
    x1 = lines[:, 0]
    x2 = lines[:, 2]
    y1 = lines[:, 1]
    y2 = lines[:, 3]

    score = np.abs(image_width/2 - (-x1 * np.divide((y2 - y1), (x2 - x1)) + y1))
    max_index = np.argmin(score)
    closest_line = lines[max_index]

    # # Get the two lines
    # closest_lines = vertical_lines[closest_indices]
    if closest_line is not None:
        # for i in range(0, len(closest_line)):
        #     l = closest_line
        cv.line(image, (closest_line[0], closest_line[1]), (closest_line[2], closest_line[3]), (0,0,255), 2, cv.LINE_AA)
    cv.imshow("Detected Lines (in red) - Probabilistic Line Transform", image)
    cv.waitKey()
    return 0
    return closest_lines  # shape (2, 4)

filename = "/home/racecar/racecar_ws/src/final_challenge2025/racetrack_images/lane_3/image16.png"
src = cv.imread(filename, cv.IMREAD_COLOR) # (Color, since later you cvtColor it to HSV)
cdstP, linesP = edges_clean(src)
compute_lane_edges(linesP, src.shape[1], cdstP)
# edges_clean(src)
