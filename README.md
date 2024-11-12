Repository of the group **EuroVision** of the Tsinghua Course _Machine Vision_ 2024.

# Setup guide

## Windows
```
git clone git@github.com:birawaich/thu_mavi_eurovision.git
cd thu_mavi_eurovision
python -m venv .venv
.venv\Scripts\activate
set PYTHONPATH = %PYTHONPATH%;%cd%
pip install -r requirements.txt
python project\main.py
```

## Linux
```
git clone git@github.com:birawaich/thu_mavi_eurovision.git
cd thu_mavi_eurovision
virtualenv --python=/usr/bin/python3.10 .venv
echo 'export PYTHONPATH="$PWD:$PYTHONPATH"' >> .venv/bin/activate
source .venv/bin/activate
pip install -r requirements.txt
```

# Running Application

Entry Point: `main.py`

Quick description: Navigate towards the object designated with `keyword` in `main.py`.
Set to **bottle** by default.
If the object is not seen, it explores (no smart exploring i.e. no SLAM).

Runs 3 Threads:

- Capturing Images wiht both cameras and sending it to a pipe
- Reading form that pipe, detect objects with YOLO, and if keyword is on the frame, estimate distance to option --> send result to a pipe
- Navigate the robot towards the object by reading from the two pipes

## Quick Description Source Files

- `camera_capture.py` ... function and class (`FrameContainer`) to capture the images with the camera
- `frame_evaluation.py` ... functions to evaluate the captured frames
- `frame_container.py` ... class to store images, detected objects on them, depth map, ... with a timestamp
- `navigation.py` ... functions to run robot control loop
- `camera_#_extrinsics.yaml` ... camera calibration parameters

## Camera Calibration

Files used for camera calibration can be found in `camera-calib/`. It is taken from [this blog post](https://github.com/TemugeB/python_stereo_camera_calibrate).

## Other Folders

- `camera_mount/` ... contains the CAD files for the camera mount
- `camera_parameters/` ... the extracted camera parameters and an (unused) class on how to get them
- `*test` ... was some test code that was used along the way
- `*.ipynb` ... as above, some Jupyther Notebooks

# Legacy

Ideally, this code drives a Turtlebot. However, due to an odysee of errors, this never worked reliably. See below for some information.

## Turtlebot setup

See [TurtleBot Info](./turtlebot_info.md).

Also see [some notes](./progress.md).


## Yolo V10 setup
First time running

```
pip install ultralytics
yolo export model=yolov8n.pt imgsz=640 format=onnx opset=12
pip install -r requirements.txt
```

And then
```
python yolo-test/main.py --model yolov8n.onnx --img image.jpg
```

# Resources

[OpenCV Documentation](https://docs.opencv.org/4.10.0/index.html)  
[OpenCV Python Package](https://pypi.org/project/opencv-python/)

