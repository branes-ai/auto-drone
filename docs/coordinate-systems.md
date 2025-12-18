# Coordinate Systems in AirSim/Project AirSim

This document explains the coordinate systems used in AirSim and Project AirSim, their relationships, and how to properly transform between them for drone navigation and control.

## Overview

Working with drones involves multiple coordinate frames that must be correctly understood and transformed:

1. **World Frame (NED)** - Global reference for positions
2. **Body Frame** - Attached to the drone, moves with it
3. **Sensor Frames** - Each sensor has its own orientation
4. **Image Frame** - 2D pixel coordinates from cameras

## 1. World Frame: NED (North-East-Down)

AirSim uses the **NED (North-East-Down)** coordinate system for world coordinates:

```
        North (+X)
           ↑
           |
           |
  West ←---+---→ East (+Y)
           |
           |
           ↓
        (into page: Down = +Z)
```

### Axes
| Axis | Direction | Positive Movement |
|------|-----------|-------------------|
| X | North | Moving north increases X |
| Y | East | Moving east increases Y |
| Z | Down | Moving down increases Z |

### Key Implications

**Altitude is NEGATIVE Z:**
- A drone at 10 meters altitude has `z = -10.0`
- A drone at ground level has `z = 0.0`
- A drone at 50 meters altitude has `z = -50.0`

**Common Mistake:**
```python
# WRONG: Checking if drone is above 20m altitude
if z > -20.0:  # This is TRUE when z=-10 (only 10m up!)

# CORRECT: More negative z = higher altitude
if z < -20.0:  # This is TRUE when z=-25 (25m up)
```

### Rotations (Euler Angles)
| Rotation | Axis | Positive Direction |
|----------|------|-------------------|
| Roll (φ) | X (North) | Right wing down |
| Pitch (θ) | Y (East) | Nose up |
| Yaw (ψ) | Z (Down) | Clockwise from above |

**Yaw Convention:**
- Yaw = 0° → Facing North (+X)
- Yaw = 90° (π/2) → Facing East (+Y)
- Yaw = 180° (π) → Facing South (-X)
- Yaw = -90° (-π/2) → Facing West (-Y)

## 2. Body Frame

The body frame is attached to the drone and moves/rotates with it:

```
      Forward (+X_body)
           ↑
           |
    Left ←-+-→ Right (+Y_body)
           |
           ↓
      (into drone: Down = +Z_body)
```

### Axes
| Axis | Direction | Positive Movement |
|------|-----------|-------------------|
| X_body | Forward | Drone moves forward |
| Y_body | Right | Drone moves right (strafes) |
| Z_body | Down | Drone descends |

### Velocity Commands

Velocity commands to the flight controller are typically in **body frame**:

```python
# VelocityCommand in body frame
vx = 2.0   # 2 m/s forward
vy = 0.0   # no lateral movement
vz = 0.5   # 0.5 m/s descending (positive = down)
yaw_rate = 0.0  # no rotation
```

## 3. Transforming World to Body Frame

To navigate to a world position, you must transform world velocities to body frame:

### The Transformation

Given:
- Drone position: `(x, y, z)` in world frame
- Drone yaw: `ψ` (psi)
- Target position: `(target_x, target_y, target_z)` in world frame

```python
import math

# Vector to target in world frame
dx = target_x - x  # North component
dy = target_y - y  # East component

# Desired velocity in world frame
speed = 5.0
target_heading = math.atan2(dy, dx)
vx_world = speed * math.cos(target_heading)
vy_world = speed * math.sin(target_heading)

# Transform to body frame (rotation by -yaw)
cos_yaw = math.cos(yaw)
sin_yaw = math.sin(yaw)

vx_body = vx_world * cos_yaw + vy_world * sin_yaw
vy_body = -vx_world * sin_yaw + vy_world * cos_yaw
```

### Why This Works

The rotation matrix from world to body frame (for yaw only):
```
R = | cos(ψ)   sin(ψ) |
    | -sin(ψ)  cos(ψ) |
```

**Example:**
- Drone at origin, yaw = 90° (facing East)
- Target is North (dx=10, dy=0)
- World velocity: vx_world=5, vy_world=0 (go North)
- Body velocity: vx_body=0, vy_body=-5 (go Left, which is North when facing East)

## 4. Altitude Control

The most common source of bugs is altitude control with NED:

### Correct Altitude Control

```python
current_z = -25.0      # Drone at 25m altitude
target_z = -5.0        # Want to be at 5m altitude

# In NED: more negative z = higher altitude
# current_z (-25) < target_z (-5), so we're ABOVE target
# Need to DESCEND, which means positive vz

if current_z < target_z - 1.0:
    # We're above target (higher altitude = more negative z)
    vz = +2.0  # Descend (positive vz = move in +Z = down)
elif current_z > target_z + 1.0:
    # We're below target
    vz = -2.0  # Ascend (negative vz = move in -Z = up)
else:
    vz = 0.0   # At target altitude
```

### Visual Reference

```
Altitude (m)    NED Z        Action needed to reach z=-10
    ↑
   50m         z = -50       Descend: vz > 0
   30m         z = -30       Descend: vz > 0
   10m         z = -10       At target: vz = 0
    5m         z = -5        Ascend: vz < 0
    0m         z = 0         Ground
```

## 5. Sensor Frames

Each sensor has its own coordinate frame defined by its mounting position and orientation.

### Camera Frame Convention

Standard camera frame (OpenCV/Computer Vision convention):
```
        +Y (down in image)
         ↓
    ←----+---→ +X (right in image)
         |
         +Z (into image / forward)
```

**Image Coordinates:**
- Origin at top-left corner
- X increases rightward
- Y increases downward
- (0,0) is top-left pixel
- (width-1, height-1) is bottom-right pixel

### Camera Mounting Orientation

Camera orientation is specified by `rpy-deg` (roll, pitch, yaw) in the robot config:

```json
{
  "id": "FrontCamera",
  "origin": {
    "xyz": "0.30 0.0 -0.05",
    "rpy-deg": "0 -15 0"
  }
}
```

| Camera | Position | RPY | Effect |
|--------|----------|-----|--------|
| Forward | `0.3 0 0` | `0 0 0` | Looking straight ahead |
| Down | `0 0 0.05` | `0 -90 0` | Looking straight down |
| Back | `-0.15 0 0` | `0 0 180` | Looking backward |
| Tilted down | `0.3 0 -0.05` | `0 -15 0` | 15° below horizon |

**RPY Convention for Cameras:**
- Roll: Rotation around the viewing axis (tilts horizon)
- Pitch: Rotation around lateral axis
  - Pitch = 0°: Looking at horizon
  - Pitch = -90°: Looking straight down
  - Pitch = +15°: Looking 15° above horizon
  - Pitch = -15°: Looking 15° below horizon
- Yaw: Rotation around vertical axis
  - Yaw = 0°: Forward
  - Yaw = 180°: Backward
  - Yaw = 90°: Right
  - Yaw = -90°: Left

## 6. Converting Image Bearings to World Coordinates

When a camera detects an object, we get pixel coordinates. To estimate world position:

### Step 1: Pixel to Bearing

```python
# Image dimensions
width, height = 640, 480
fov_degrees = 90.0

# Object detected at pixel (cx, cy)
cx, cy = 400, 300

# Convert to normalized bearing (-1 to +1)
bearing_x = (cx - width/2) / (width/2)   # +1 = right edge
bearing_y = (cy - height/2) / (height/2) # +1 = bottom edge
```

### Step 2: Bearing to Angle

```python
half_fov = math.radians(fov_degrees / 2)

# Angle from camera center
horizontal_angle = bearing_x * half_fov  # Positive = right
vertical_angle = bearing_y * half_fov    # Positive = down (in image)
```

### Step 3: Account for Camera Pitch

```python
camera_pitch = math.radians(-15)  # Camera tilted 15° down

# Actual angle below horizon
look_angle = vertical_angle - camera_pitch
```

### Step 4: Estimate World Position

```python
# Drone pose
drone_x, drone_y, drone_z = -1.0, 8.0, -25.0
drone_yaw = math.radians(-45)

# Estimate distance (from apparent size or assume fixed)
estimated_distance = 50.0  # meters

# World heading to target
target_heading = drone_yaw + horizontal_angle

# Horizontal distance (accounting for look angle)
horizontal_dist = estimated_distance * math.cos(look_angle)

# World position estimate
target_x = drone_x + horizontal_dist * math.cos(target_heading)
target_y = drone_y + horizontal_dist * math.sin(target_heading)
target_z = 0.0  # Assume ground level
```

## 7. Six Degrees of Freedom (6-DOF)

A drone has 6 degrees of freedom - 3 translational and 3 rotational:

### Translational (Position)

| DOF | World Frame | Body Frame | Positive Direction |
|-----|-------------|------------|-------------------|
| X | North | Forward | Increases position |
| Y | East | Right | Increases position |
| Z | Down | Down | Increases position (descend) |

### Rotational (Orientation)

| DOF | Name | Axis | Positive Rotation |
|-----|------|------|-------------------|
| φ | Roll | X/Forward | Right wing down |
| θ | Pitch | Y/Right | Nose up |
| ψ | Yaw | Z/Down | Clockwise (from above) |

### Control Commands Summary

```python
# Typical velocity command structure
class VelocityCommand:
    vx: float      # Body-frame forward velocity (m/s)
    vy: float      # Body-frame rightward velocity (m/s)
    vz: float      # Body-frame downward velocity (m/s)
    yaw_rate: float  # Rotation rate around Z axis (rad/s)
```

| Desired Motion | vx | vy | vz | yaw_rate |
|----------------|----|----|----| ---------|
| Move forward | + | 0 | 0 | 0 |
| Move backward | - | 0 | 0 | 0 |
| Strafe right | 0 | + | 0 | 0 |
| Strafe left | 0 | - | 0 | 0 |
| Descend | 0 | 0 | + | 0 |
| Ascend | 0 | 0 | - | 0 |
| Rotate CW | 0 | 0 | 0 | + |
| Rotate CCW | 0 | 0 | 0 | - |

## 8. Common Pitfalls and Solutions

### Pitfall 1: Altitude Sign Confusion

```python
# WRONG
altitude = z  # This is negative!

# CORRECT
altitude = -z  # Convert NED z to altitude
```

### Pitfall 2: Velocity Frame Mismatch

```python
# WRONG: Sending world-frame velocity as body-frame
send_velocity(vx_world, vy_world, vz, yaw_rate)

# CORRECT: Transform first
vx_body, vy_body = transform_to_body(vx_world, vy_world, yaw)
send_velocity(vx_body, vy_body, vz, yaw_rate)
```

### Pitfall 3: Yaw Sign for Visual Servoing

```python
# Object detected on RIGHT side of image (bearing_x > 0)
# Drone needs to turn RIGHT (clockwise, positive yaw rate)

# WRONG
yaw_rate = -bearing_x  # Turns left!

# CORRECT
yaw_rate = +bearing_x * gain  # Turns toward target
```

### Pitfall 4: Camera Pitch Not Accounted

```python
# Camera tilted 15° down, object at image center
# WRONG: Assume object is at horizon level

# CORRECT: Object is 15° below horizon
vertical_angle = bearing_y * half_fov + camera_pitch_offset
```

## 9. Coordinate System Diagram

```
WORLD FRAME (NED)                    BODY FRAME
================                     ==========

     North (+X)                       Forward (+X_b)
        ↑                                  ↑
        |                                  |
        |  ψ (yaw)                         |
West ←--+--→ East (+Y)            Left ←--+--→ Right (+Y_b)
        |                                  |
        ↓                                  ↓
    (Down = +Z)                      (Down = +Z_b)


CAMERA FRAME                         IMAGE FRAME
============                         ===========

    +Z (forward/depth)                 (0,0)──→ +X (pixels)
        ↗                                │
       /                                 │
      /                                  ↓
     +----→ +X (right)                  +Y (pixels)
     |
     ↓
    +Y (down)
```

## 10. Quick Reference Card

### NED Coordinate Rules
- **X** = North, **Y** = East, **Z** = Down
- Higher altitude = more negative Z
- Yaw 0° = facing North

### Body Frame Rules
- **X** = Forward, **Y** = Right, **Z** = Down
- Positive vx = move forward
- Positive yaw_rate = rotate clockwise

### Transformation: World → Body
```python
vx_body = vx_world * cos(yaw) + vy_world * sin(yaw)
vy_body = -vx_world * sin(yaw) + vy_world * cos(yaw)
```

### Camera Bearing to Angle
```python
angle_horizontal = bearing_x * (fov/2)
angle_vertical = bearing_y * (fov/2) + camera_pitch
```

### Altitude Control
```python
if z < target_z:    # Above target (more negative = higher)
    vz = +speed     # Descend (positive vz)
else:
    vz = -speed     # Ascend (negative vz)
```
