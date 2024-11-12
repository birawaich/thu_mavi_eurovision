import cv2
import numpy as np

#distance if you have the frames
def disparity(frame_left, frame_right):
    # StereoBM matcher (for computing the depth map)
    stereo_left = cv2.StereoBM_create(numDisparities=16, blockSize=15)
    stereo_right = cv2.ximgproc.createRightMatcher(stereo_left)

    # Convert frames to grayscale for depth map calculation
    gray_left = cv2.cvtColor(frame_left, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(frame_right, cv2.COLOR_BGR2GRAY)

    # Compute the depth map
    disparity_left = frame_left.compute(gray_left, gray_right).astype(np.float32) / 16.0
    disparity_left[disparity_left == 0] = 0.1  # Avoid division by zero
    disparity_right = stereo_right.compute(gray_right, gray_left).astype(np.float32) / 16.0
    disparity_right[disparity_left == 0] = 0.1

    # Apply WLS filter to improve disparity
    wls_filter = cv2.ximgproc.createDisparityWLSFilter(stereo_left)
    wls_filter.setLambda(1000)  #variable, can be adjusted
    wls_filter.setSigmaColor(1.5)   #variable, can be adjusted
    filtered_disp = wls_filter.filter(disparity_left, gray_left, disparity_map_right=disparity_right)
    filtered_disp[filtered_disp <= 0] = 0.1

    return filtered_disp

def depth_pixels(x,y,filtered_disp):
    distance = 10 * 50 / filtered_disp[x,y]
    return (distance)  # distance in cm

