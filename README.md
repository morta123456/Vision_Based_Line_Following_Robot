## 🤖 Vision-Based Line Following Robot

[![Python](https://img.shields.io/badge/Python-3.8%2B-blue)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](./LICENSE)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.8.1-red)](https://opencv.org/)
[![NumPy](https://img.shields.io/badge/NumPy-1.24%2B-lightgrey)](https://numpy.org/)
[![Camera Based](https://img.shields.io/badge/Sensor-Camera-blueviolet)]()
[![Autonomous](https://img.shields.io/badge/Mode-Autonomous-success)]()

## 🧭 Project Overview

This project implements a **camera-only line-following robot** that uses **OpenCV** for real-time image analysis.
The robot captures video frames, detects the path using **color thresholds and contours**, and adjusts its motion accordingly through a **motor control system**.

By leveraging only visual data, this project demonstrates how computer vision can replace traditional sensors like infrared or ultrasonic modules in simple autonomous navigation tasks.

## 🎯 Project Goals

- 📷 Use a camera as the sole sensor for path detection and tracking.

- 🧠 Implement color and contour-based line detection using OpenCV.

- ⚙️ Control the robot’s motors dynamically based on the detected path position.

- 🧩 Develop modular and reusable Python scripts for image processing and control logic.

- 🧪 Test and validate the line-following performance under different lighting and path conditions.

## 🧩 Key Components

| Module | Description |
|---------|----------------------|
| **main_follow_line.py** | Main script that runs the line-following behavior |
| **linefollower.py** | Core module for image acquisition and line detection |
| **colorranges.py** | Defines color thresholds for detecting the line |
| **motor_controller.py** | Controls the robot’s wheel motors based on steering signals |
| **robotcontroller.py** | Integrates camera input and motor commands for overall control |
| **mathutils.py** | Helper functions for angle, geometry, and motion calculations |
| **wheel_test.py** | Basic wheel/motor testing script |
| **test_main.py** | Script for running unit tests |
| **Test.ipynb** | Notebook for vision tuning, debugging, and analysis |

