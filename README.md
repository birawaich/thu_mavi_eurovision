Repository of the group **EuroVision** of the Tsinghua Course _Machine Vision_ 2024.

# Setup guide

## Windows
```
git clone git@gitlab.ewi.tudelft.nl:ti3115tu/2022-2023/Group-16.git
cd Group-16
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