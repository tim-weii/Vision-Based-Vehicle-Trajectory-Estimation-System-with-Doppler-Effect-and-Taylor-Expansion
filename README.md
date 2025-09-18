# 🚗 Vision-Based Vehicle Trajectory Estimation System

[![Python](https://img.shields.io/badge/Python-3.9+-blue?logo=python)](https://www.python.org/)  
[![YOLOv3](https://img.shields.io/badge/YOLOv3-Object%20Detection-green)](https://pjreddie.com/darknet/yolo/)  
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

---

## 📖 Overview

This project implements a **vision-based vehicle trajectory estimation system** that combines **computer vision, GPS, and Doppler-based relative velocity estimation** for real-time risk assessment.  

Vehicles are detected from single or multiple cameras, and **license plates are used as stable reference points** (center + orientation). After target tracking builds continuous trajectories, **Taylor expansion** (local polynomial approximation) is applied for short-term trajectory prediction. GPS and Doppler (or visual proxy) provide relative velocity estimation, enabling calculation of **dynamic safe distances** and **collision risk alerts** in real time.

---

## 🧱 System Architecture

```text
Input
 ├─ Cameras (single/multi-view)
 ├─ GPS (ego-velocity, heading, timestamp)        │
        ▼
Perception
 ├─ Vehicle Detection (YOLOv3)
 ├─ License Plate Detection & Preprocessing
 └─ Plate Center & Orientation Extraction
        │
        ▼
Tracking
 └─ Multi-Object Tracking (Kalman filter for ID consistency)
        │
        ▼
Trajectory & Prediction
 ├─ Trajectory Buffer (per track_id)
 └─ Short-term Prediction (Taylor Expansion, 0.5–1.5s horizon)
        │
        ▼
Sensor Fusion
 ├─ Ego velocity & heading from GPS
 ├─ Relative radial velocity from Doppler
 └─ Range & range-rate estimation
        │
        ▼
Risk Assessment
 ├─ TTC (Time-To-Collision)
 ├─ Dynamic Safe Distance (physics-based or zone-based)
 └─ Risk Alert Decision
        │
        ▼
Visualization & Output
 ├─ Overlays (bbox, ID, trajectory, risk)
 └─ Alerts (color, audio, logs)
```
