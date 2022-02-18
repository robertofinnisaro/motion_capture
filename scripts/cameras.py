#!/usr/bin/env python3
import os
from re import sub
from threading import Thread
import subprocess
import sys

Camera4 = subprocess.Popen([sys.executable, "~/catkin_ws/src/motion_capture/scripts/camera_4.py"])
Camera5 = subprocess.Popen([sys.executable, "~/catkin_ws/src/motion_capture/scripts/camera_5.py"])
