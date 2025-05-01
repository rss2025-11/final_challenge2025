"""
@file hough_lines.py
@brief This program demonstrates line finding with the Hough transform
"""
import sys
import math
import cv2 as cv
import numpy as np

lookahead_dist = 2.0 # meters
lookahead_pt_history = np.tile([lookahead_dist, 0.0], (20, 1))

def compute_weighted_lookahead(lookahead_pt_history, new_lookahead_point):

    # TODO: if the most recently computed point is very far from the rest, don't use the raw calculation

    lookahead_pt_history = np.roll(lookahead_pt_history, 1, axis=0)
    lookahead_pt_history[0] = new_lookahead_point

    weights = np.ones(20) * (0.4 / 17)  # Remaining 17 entries share 0.4
    weights[0] = 0.3  # Newest
    weights[1] = 0.2  # 2nd newest
    weights[2] = 0.1  # 3rd newest

    weighted_point = np.sum(lookahead_pt_history * weights[:, np.newaxis], axis=0)
    return weighted_point


def skeletonize(image):
    # Ensure the image is grayscale
    if len(image.shape) == 3:
        image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    
    # Ensure image is binary (black and white)
    _, binary = cv.threshold(image, 127, 255, cv.THRESH_BINARY)
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
    hsv[:np.shape(hsv)[0]//3,:] = 255
    mask_initial = cv.inRange(hsv, lower_white, upper_white)
    _, mask = cv.threshold(mask_initial, 127, 255, cv.THRESH_BINARY)

    # Apply erosion to thin thick lines slightly
    kernel = cv.getStructuringElement(cv.MORPH_RECT, (1, 1))
    thinned_mask = skeletonize(mask)#cv.erode(mask, kernel, iterations=1)

    # print("height, width of original image:", np.shape(thinned_mask))
    # Crop the lower part of the image
    # cropped_dst = thinned_mask[np.shape(thinned_mask)[0]//3:, :]
    # cropped_dst = mask[len(mask[0])//5:, :]

    # Convert to BGR for visualization # was last 50
    linesP = cv.HoughLinesP(thinned_mask, 1, np.pi / 180, 50, None, 100, 20) # TODO: Tune these numbers if needed

    # # For visualization only:
    cdstP_vis = cv.cvtColor(thinned_mask, cv.COLOR_GRAY2BGR)
    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            cv.line(cdstP_vis, (l[0], l[1]), (l[2], l[3]), (255,0,0), 1, cv.LINE_AA)

    # cv.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP_vis)
    # cv.waitKey()
    # return 0
    return cdstP_vis, linesP



def compute_lane_edges(lines, image_width, image_height, image):
    # lines shape: (N, 1, 4), where 4 = (x1, y1, x2, y2)
    lines = np.squeeze(lines)  # (N, 4)

    # Compute horizontal spread (|x2 - x1|) for all lines
    x1 = lines[:, 0]
    x2 = lines[:, 2]
    y1 = lines[:, 1]
    y2 = lines[:, 3]

    # TODO: how should we handle case where y2-y1 = 0?
    # Should replace that intercept with inf

    # if np.any(abs(x1-x2) < 0.001):
    #     intercept_to_robot = np.avg(x1)
    # else:
    dx = x1 - x2
    dy = y1 - y2
    # Create mask for non-horizontal lines (dy >= 5)
    non_horizontal_mask = (abs(dy) >= 5)
    # Filter out horizontal lines
    lines = lines[non_horizontal_mask]
    dx = dx[non_horizontal_mask]
    dy = dy[non_horizontal_mask]
    x1 = x1[non_horizontal_mask]
    y1 = y1[non_horizontal_mask]
    y2 = y2[non_horizontal_mask]

    non_vertical_mask = (abs(dx) >= 5)

    lines = lines[non_vertical_mask]
    dx = dx[non_vertical_mask]
    dy = dy[non_vertical_mask]
    x1 = x1[non_vertical_mask]
    y1 = y1[non_vertical_mask]
    y2 = y2[non_vertical_mask]

    slope = np.divide(dy, dx)

    vertical_enough_mask = (abs(slope) > 0.2)

    lines = lines[vertical_enough_mask]
    dx = dx[vertical_enough_mask]
    dy = dy[vertical_enough_mask]
    x1 = x1[vertical_enough_mask]
    y1 = y1[vertical_enough_mask]
    y2 = y2[vertical_enough_mask]
    slope = slope[vertical_enough_mask]


    # intercept_to_robot describes the x intercept at the bottom of the image
    # i.e. where vertically does the line reach the camera?
    intercept_to_robot = (np.divide(image_height - y1, slope) + x1)
    score = np.abs(image_width/2 - intercept_to_robot)
    max_index = np.argmin(score)

    if (intercept_to_robot[max_index] < image_width/2):
        is_left = 1
    else:
        is_left = 0

    closest_line = lines[max_index]

    # # Get the two lines
    # closest_lines = vertical_lines[closest_indices]
    if closest_line is not None:
        # for i in range(0, len(closest_line)):
        #     l = closest_line
        cv.line(image, (closest_line[0], closest_line[1]), (closest_line[2], closest_line[3]), (255,0,255), 2, cv.LINE_AA)
    # cv.imshow("Detected Lines (in red) - Probabilistic Line Transform", image)
    # cv.waitKey()
    # return 0
    closest_line = np.array(([closest_line[0], closest_line[1]], [closest_line[2], closest_line[3]]))


    return closest_line, is_left, image  # shape (2, 4)

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

def transform_homography(homography, point):
    u = point[0]
    v = point[1]
    homogeneous_point = np.array([[u], [v], [1]])
    xy = np.dot(homography, homogeneous_point)
    scaling_factor = 1.0 / xy[2, 0]
    homogeneous_xy = xy * scaling_factor
    x = homogeneous_xy[0, 0]
    y = homogeneous_xy[1, 0]

    # remember that x and y are reversed to be in the car frame
    return x, y

def determine_midline(is_left, closest_line):
    # convert using homography matrix
    # determine if the closest line was to the left or right
    # create a parallel line
    if is_left:
        offset = -0.4
    else:
        offset = 0.4
    point1x = closest_line[0][0]
    point2x = closest_line[1][0]
    point1y = closest_line[0][1]
    point2y = closest_line[1][1]
    point1y = point1y + offset
    point2y = point2y + offset
    
    new_line = np.array(([point1x, point1y], [point2x, point2y]))
    return new_line

def determine_lookahead_point(p1x, p1y, p2x, p2y, is_left):
    # assume the points have been transformed into robot frame using homography matrix already
    slope = (p2y - p1y) / (p2x - p1x)
    y_intercept = slope * (lookahead_dist - p1x) + p1y
    if is_left:
        offset = -0.25
    else:
        offset = 0.25
    lookahead_point = [lookahead_dist, y_intercept + offset]
    return lookahead_point

def viz_lookeahead_pt(point, im):
    pixel_x, pixel_y = transform_homography(inv_homography, point)
    # Draw a green circle at the transformed pixel coordinates
    cv.circle(im, (int(round(pixel_x)), int(round(pixel_y))), radius=5, color=(0, 255, 0), thickness=-1)
    cv.imshow("Lookeahead point", im)
    cv.waitKey()

def viz_midline(line, im):
    pixel_1x, pixel_1y = transform_homography(inv_homography, line[0])
    pixel_2x, pixel_2y = transform_homography(inv_homography, line[1])
    cv.line(im, (round(pixel_1x), round(pixel_1y)), (round(pixel_2x), round(pixel_2y)), (0,255,0), 2, cv.LINE_AA)
    cv.imshow("Midline", im)
    cv.waitKey()


video_path = "../output.mp4"
output_path = "test_lines.mp4"

cap = cv.VideoCapture(video_path)

# Get video properties
fps = cap.get(cv.CAP_PROP_FPS)
width  = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))

# Define the codec and create VideoWriter object
fourcc = cv.VideoWriter_fourcc(*'mp4v')  # or use 'XVID' or 'avc1'
out = cv.VideoWriter(output_path, fourcc, fps, (width, height))

homography = create_homography_matrix()
inv_homography = np.linalg.inv(homography)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    cdstP, linesP = edges_clean(frame)
    display_frame = frame.copy()

    if linesP is not None:
        try:
            closest_line, is_left, image = compute_lane_edges(linesP, width, height, display_frame)
            point1x, point1y = transform_homography(homography, closest_line[0])
            point2x, point2y = transform_homography(homography, closest_line[1])
            lane_line = np.array(([point1x, point1y], [point2x, point2y]))
            lookahead_pt = determine_lookahead_point(point1x, point1y, point2x, point2y, is_left)
            print(lookahead_pt)
            weighted_lookahead = compute_weighted_lookahead(lookahead_pt_history, lookahead_pt)
            pixel_1x, pixel_1y = transform_homography(inv_homography, weighted_lookahead)
            cv.circle(image, (int(round(pixel_1x)), int(round(pixel_1y))), radius=5, color=(0, 255, 0), thickness=-1)
        except Exception as e:
            print("Error processing frame:", e)

    out.write(display_frame)
    cv.imshow("Midline", display_frame)

    if cv.waitKey(1) == 27:  # ESC to break
        break

cap.release()
out.release()
cv.destroyAllWindows()



# filename = "/home/racecar/racecar_ws/src/final_challenge2025/racetrack_images/lane_3/image19.png"
# homography = create_homography_matrix()
# inv_homography = np.linalg.inv(homography)
# src = cv.imread(filename, cv.IMREAD_COLOR) # (Color, since later you cvtColor it to HSV)
# cdstP, linesP = edges_clean(src)
# closest_line, is_left, image = compute_lane_edges(linesP, src.shape[1], src.shape[0], cdstP)
# point1x, point1y = transform_homography(homography, closest_line[0])
# point2x, point2y = transform_homography(homography, closest_line[1])
# lane_line = np.array(([point1x, point1y], [point2x, point2y]))
# midline = determine_midline(is_left, lane_line)
# viz_midline(midline, image)