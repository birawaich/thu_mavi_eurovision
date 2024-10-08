# program to capture single image from webcam in python 

# importing OpenCV library 
from cv2 import VideoCapture, imshow, imwrite, waitKey, destroyWindow, destroyAllWindows

# initialize the camera 
# If you have multiple camera connected with 
# current device, assign a value in cam_port 
# variable according to that 
# for i in range(20):
#     print(i)
#     cam_port = i
#     cam = VideoCapture(cam_port)

cam = [None, None]
result = [None, None]
image = [None, None]

cam_port = [2,6]
cam[0] = VideoCapture(cam_port[0])
cam[0].set(3,160)
cam[0].set(4,120)
result[0], image[0] = cam[0].read()
cam[1] = VideoCapture(cam_port[1])
cam[1].set(3,160)
cam[1].set(4,120)
result[1], image[1] = cam[1].read()
if not cam[0].isOpened() or not cam[1].isOpened():
    print("error")
    exit()

# reading the input using the camera
# # If image will detected without any error, 
# # show result 
if result[0] and result[1]: 

	# showing result, it take frame name and image 
	# output 
    imshow("cam 0", image[0])
    imshow("cam 1", image[1])

	# saving image in local storage 
    imwrite("cam0.png", image[0])
    imwrite("cam1.png", image[1])

	# If keyboard interrupt occurs, destroy image 
	# window 
    waitKey(0)
    cam[0].release()
    cam[1].release()
    destroyAllWindows()

# If captured image is corrupted, moving to else part 
else:
    print(f"No image detected. Please! try again. cam0 {result[0]}, cam1 {result[1]}") 
