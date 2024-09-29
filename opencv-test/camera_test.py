# program to capture single image from webcam in python 

# importing OpenCV library 
from cv2 import VideoCapture, imshow, imwrite, waitKey, destroyWindow

# initialize the camera 
# If you have multiple camera connected with 
# current device, assign a value in cam_port 
# variable according to that 
# for i in range(20):
#     print(i)
#     cam_port = i
#     cam = VideoCapture(cam_port)
cam = [None, None]
cam_port = [2,6]
cam[0] = VideoCapture(cam_port[0]) 
cam[1] = VideoCapture(cam_port[1]) 

# # reading the input using the camera 
result[0], image[0] = cam[0].read() 


# # If image will detected without any error, 
# # show result 
if result: 

	# showing result, it take frame name and image 
	# output 
	imshow("cam 0", image[0])
    imshow("cam 1", image[0])

	# saving image in local storage 
	imwrite("cam0.png", image[0])
    imwrite("cam1.png", image[1])

	# If keyboard interrupt occurs, destroy image 
	# window 
	waitKey(0)
	destroyWindow("cam 0")
    destroyWindow("cam 1")

# If captured image is corrupted, moving to else part 
else: 
	print("No image detected. Please! try again") 
