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


##  Workflow (Step-by-Step)

The system processes video streams and sensor inputs step by step:

---

1. **Vehicle Detection**  
   - Detect vehicles from raw video frames using **YOLOv3**.  
   - Extract license plate regions with **preprocessing** (contrast enhancement, denoising, edge operations).  
   - Use license plate **centers** as stable reference points for tracking.  

   *Example:*  
   <p align="center">
     <img src="https://github.com/user-attachments/assets/1d25c0da-4d3f-437e-ba57-0d6853113141" 
          alt="Vehicle detection and plate extraction" width="260" height="200">
   </p>

---

2. **Tracking (ID Consistency)**  
   - Maintain consistent IDs across frames using a **Kalman filter** (state vector: `[x, y, vx, vy]`).  
   - Ensures temporal continuity and prevents ID switching during occlusion or detection noise.  

   *Visualization:*  
   <p align="center">
     <img src="https://www.researchgate.net/publication/354627620/figure/fig4/AS:1080241044889618@1634560943438/Working-principle-of-Kalman-filter.jpg" 
          alt="Working principle of Kalman filter" width="600" height="453">
   </p>  

   *Source:* [ResearchGate – Working principle of Kalman filter](https://www.researchgate.net/figure/Working-principle-of-Kalman-filter_fig4_354627620)

---

3. **Trajectory Buffer & Taylor Expansion**  
   - Store trajectory history in a **sliding buffer** `{t, x, y, θ}` per track_id.  
   - Apply **Taylor expansion (local polynomial approximation)** for **short-term trajectory prediction** (0.5–1.5s horizon).  

   **Formula:**  
   $$
   x(t) \approx x_0 + v_x \Delta t + \tfrac{1}{2} a_x \Delta t^2
   $$

   $$
   y(t) \approx y_0 + v_y \Delta t + \tfrac{1}{2} a_y \Delta t^2
   $$

---

4. **Sensor Fusion (GPS + Doppler)**  
   - **GPS** provides ego-velocity $v_{ego}$ and heading $\psi$.  
   - **Doppler (or visual proxy)** provides **radial relative velocity** $v_{rel}$.  
   - Fusion yields **range and range-rate** estimation.  

   **Formula:**  
   $$
   v_{rel} = (v_{target} - v_{ego}) \cdot \hat{r}
   $$

---

5. **Risk Assessment**

The system evaluates collision risk with **two complementary strategies**:

---

###  Physics-based Safe Distance

$$
d_{safe} = v_{ego} \cdot t_{react} + \frac{v_{ego}^2}{2a_{brake}}
$$
[
- **$v_{ego}$** – ego vehicle speed (自車速度)  
- **$t_{react}$** – reaction time (反應時間，例如 1s)  
- **$a_{brake}$** – maximum deceleration (最大減速度，例如 7 m/s²)  

💡 **Interpretation:**  
Safe distance = **reaction distance + braking distance**.  
If the actual gap is smaller than $d_{safe}$, even full braking may not avoid a collision.

---

###  Time-to-Collision (TTC)

$$
TTC = \frac{distance}{\max(\epsilon, v_{closing})}
$$

- **$distance$** – current distance to the lead vehicle (自車與前車的距離)  
- **$v_{closing}$** – closing speed (相對接近速度，若前車比你慢則為正值)  
- **$\epsilon$** – small constant to avoid division by zero  

💡 **Interpretation:**  
TTC estimates *how many seconds remain before collision* if both vehicles keep their current speed.  
A smaller TTC implies higher collision risk.

---

###  Alert Logic

⚠️ A **risk alert** is triggered if **either** condition is met:

- $distance < d_{safe}$ (**too close for safe braking**)  
- **OR**  
- $TTC < \tau$ (**collision expected within threshold time, e.g., 2s**)  

---

###  Summary

- **Safe Distance** → *“Do I have enough space to stop safely?”*  
- **TTC** → *“If nothing changes, how soon will I crash?”*  
- Combining both yields robustness:  
  - *Safe Distance* covers braking dynamics.  
  - *TTC* covers relative timing of collision.

---

6. **Visualization & Alerts**  
   - Overlay predicted trajectories, risk zones, and alert signals.  
   - High-risk vehicles are marked in **red**, with optional audio/haptic alerts.  

   *Example:*  
   <p align="center">
     <img width="694" height="401" alt="Risk Visualization" src="https://github.com/user-attachments/assets/8658f11e-3da1-4606-9cfd-c3e5afeef054" />
   </p>

---

##  System Diagram

```mermaid
flowchart LR
    A[Camera Frames] --> B[YOLOv3 Detection];
    B --> C[License Plate Extraction];
    C --> D[Tracking (Kalman)];
    D --> E[Trajectory Buffer];
    E --> F[Taylor Expansion Prediction];
    F --> G[Fusion => GPS + Doppler];
    G --> H[Risk Assessment];
    H --> I[Visualization & Alerts];
```

