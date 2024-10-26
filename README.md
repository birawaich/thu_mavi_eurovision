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

## Turtlebot setup

See [TurtleBot Info](./turtlebot_info.md)


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

