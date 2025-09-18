#  Vision-Based Vehicle Trajectory Estimation System

[![Python](https://img.shields.io/badge/Python-3.9+-blue?logo=python)](https://www.python.org/)  
[![YOLOv3](https://img.shields.io/badge/YOLOv3-Object%20Detection-green)](https://pjreddie.com/darknet/yolo/)  
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

---

##  Overview

This project implements a **vision-based vehicle trajectory estimation system** that combines **computer vision, GPS, and Doppler-based relative velocity estimation** for real-time risk assessment.  

Vehicles are detected from single or multiple cameras, and **license plates are used as stable reference points** (center + orientation). After target tracking builds continuous trajectories, **Taylor expansion** (local polynomial approximation) is applied for short-term trajectory prediction. GPS and Doppler (or visual proxy) provide relative velocity estimation, enabling calculation of **dynamic safe distances** and **collision risk alerts** in real time.

---

##  Repository Layout

```text
.
├─ src/
│  ├─ detectors/                         # Detection and license plate processing
│  │  ├─ yolo_v3.py                      # Vehicle detection with YOLOv3 (load_model, infer)
│  │  ├─ license_plate.py                # License plate detection with heavy preprocessing
│  │  └─ plate_locator.py                # ROI refinement, plate angle estimation, center extraction
│  ├─ tracking/                          # Multi-object tracking
│  │  └─ kalman.py                       # Kalman filter (ID consistency, not final prediction)
│  ├─ trajectory/                        # Trajectory & short-term prediction
│  │  ├─ buffer.py                       # Sliding window buffer {t, x, y, θ} per track_id
│  │  └─ taylor_predictor.py             # Taylor expansion (0.5–1.5s short-term prediction)
│  ├─ fusion/                            # Sensor fusion
│  │  ├─ gps.py                          # GPS parsing (ego velocity, heading, timestamp)
│  │  └─ doppler.py                      # Radial relative velocity estimation
│  ├─ risk/                              # Risk assessment
│  │  ├─ ttc.py                          # Time-To-Collision calculation
│  │  └─ safe_distance.py                # Safe distance (physics-based / zone-based)
│  ├─ viz/                               # Visualization
│  │  ├─ overlay.py                      # Overlay boxes, IDs, trajectories, alerts
│  │  └─ palette.py                      # Color palette and style configs
│  └─ utils/                             # Shared utilities
│     ├─ geometry.py                     # Projection, distance, angle conversions
│     ├─ smoothing.py                    # EMA, median filter, hysteresis smoothing
│     └─ metrics.py                      # Evaluation metrics (ADE, FDE, IDF1, etc.)
├─ configs/
│  ├─ model.yaml                         # Model paths, input sizes, detector thresholds
│  └─ thresholds.yaml                    # Risk thresholds (TTC, safe distance, zone boundaries)
└─ README.md
```

## System Architecture

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
