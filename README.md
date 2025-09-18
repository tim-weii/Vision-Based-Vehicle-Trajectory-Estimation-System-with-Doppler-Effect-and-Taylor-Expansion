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
 ├─ GPS (ego-velocity, heading, timestamp) 
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


## 🔄 Workflow (Step-by-Step)

The system processes video streams and sensor inputs step by step:

1. **Vehicle Detection**  
   - Detect vehicles from raw video frames using YOLOv3.  
   - Extract license plate regions with preprocessing (contrast, denoising, edge ops).  
   - Plate centers serve as **stable reference points**.  
   - *Example:*  
     ![Vehicle detection and plate extraction](<img width="204" height="155" alt="image" src="https://github.com/user-attachments/assets/1d25c0da-4d3f-437e-ba57-0d6853113141" />
)

---

2. **Tracking (ID Consistency)**  
   - Maintain consistent IDs across frames using a Kalman filter (state: `[x, y, vx, vy]`).  
   - Prevents ID switching when occlusion or detection jitter occurs.
   
---

3. **Trajectory Buffer & Taylor Expansion**  
   - Store trajectory history in a sliding buffer `{t, x, y, θ}`.  
   - Apply **Taylor expansion** for short-term trajectory prediction (0.5–1.5s).  

   **Formula:**  
   $$
   x(t) \approx x_0 + v_x \Delta t + \tfrac{1}{2} a_x \Delta t^2 \\
   y(t) \approx y_0 + v_y \Delta t + \tfrac{1}{2} a_y \Delta t^2
   $$

---

4. **Sensor Fusion (GPS + Doppler)**  
   - GPS provides ego-velocity $v_{ego}$ and heading $\psi$.  
   - Doppler (or visual proxy) provides **radial relative velocity** $v_{rel}$.  
   - Fusion yields **range and range-rate** estimation.  

   **Formula:**  
   $$
   v_{rel} = (v_{target} - v_{ego}) \cdot \hat{r}
   $$

---

5. **Risk Assessment**  
   - Two strategies:  
     - **Physics-based safe distance:**  
       $$
       d_{safe} = v_{ego} \cdot t_{react} + \frac{v_{ego}^2}{2a_{brake}}
       $$  
     - **Time-to-Collision (TTC):**  
       $$
       TTC = \frac{distance}{\max(\epsilon, v_{closing})}
       $$  

   - Risk alert is triggered if:  
     - $distance < d_{safe}$, OR  
     - $TTC < \tau$ (threshold).  

---

6. **Visualization & Alerts**  
   - Overlay predicted trajectories, risk zones, and alert signals.  
   - High-risk vehicles are marked in **red** with optional audio/haptic alerts.  
   - *Example:*  
     ![Final overlay visualization](<img width="694" height="401" alt="image" src="https://github.com/user-attachments/assets/8658f11e-3da1-4606-9cfd-c3e5afeef054" />

---

##  System Diagram

```mermaid
flowchart LR
    A[Camera Frames] --> B[YOLOv3 Detection]
    B --> C[License Plate Extraction]
    C --> D[Tracking (Kalman)]
    D --> E[Trajectory Buffer]
    E --> F[Taylor Expansion Prediction]
    F --> G[Fusion (GPS + Doppler)]
    G --> H[Risk Assessment]
    H --> I[Visualization & Alerts]

```
