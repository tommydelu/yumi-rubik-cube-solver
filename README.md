# 🧊 YuMi Cube Solver

### Dual-arm robotic manipulation guided by 3D perception in CoppeliaSim + ROS2

> **Objective:** solve a Rubik's Cube autonomously using the ABB YuMi dual-arm robot, simulated in CoppeliaSim and orchestrated via ROS2 — combining 3D point cloud perception, dual-gripper manipulation, and classical color-recognition computer vision.

---

## 📋 Table of Contents

- [Setup & Installation](#-setup--installation)
- [Pipeline Overview](#-pipeline-overview)
- [1. Localization](#1-localization)
- [2. Pick](#2-pick)
- [3. Scanning](#3-scanning)
- [4. Color Recognition](#-rubik-cube-color-recognition)
- [5. Solve Execution](#5-execute-the-sequence-of-moves)
- [6. Verification](#6-verify-the-solve)
- [7. Reset](#7-reset)

---

## 🛠️ Setup & Installation

### Prerequisites

| Requirement | Version |
|---|---|
| OS | Ubuntu 22.04 (native), or macOS via remote connection |
| ROS2 | Humble |
| CoppeliaSim | EDU, latest version |
| Python | 3.10.8 |

> **Two supported setups:**
> - **Ubuntu 22.04**, with `simROS2` running natively inside CoppeliaSim — all ROS2 topics (arm poses, joint states, gripper commands, ground-truth cube pose) are published directly by Lua scripts in the scene. This is the **primary, recommended setup**.
> - **macOS**, where `simROS2` is not available — CoppeliaSim is instead controlled remotely via the [ZMQ Remote API](https://www.coppeliarobotics.com/helpFiles/en/zmqRemoteApiOverview.htm) from a bridge node running on a separate Ubuntu machine/VM. This setup is documented as a fallback and requires the `coppeliasim-zmqremoteapi-client` package (see `requirements.txt`).

### 1. Clone the repository

```bash
git clone https://github.com/tommydelu/Vision_AI.git
cd Vision_AI
```

### 2. Install CoppeliaSim

Download and install **CoppeliaSim EDU (latest version)** from the [official site](https://www.coppeliarobotics.com/downloads).

### 3. Set up the Python environment

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

> On the macOS / Remote API setup, `coppeliasim-zmqremoteapi-client` (included in `requirements.txt`) is required. On native Ubuntu setups with `simROS2` working, this package is not needed for perception, but keeping it installed does no harm.

### 4. Build the ROS2 workspace

```bash
cd yumi_ws
colcon build
source install/setup.bash
```

> Repeat `source install/setup.bash` in every new terminal, or add it to your `~/.bashrc`.

### 5. Load the CoppeliaSim scene

Open CoppeliaSim and load the scene file:

```
yumi_ws/src/scene_coppelia/yumi_scene_doubleSensor.ttt
```

Press **Play (▶)** to start the simulation *before* launching the ROS2 stack.

### 6. Launch the pipeline

Once the simulation is running:

```bash
ros2 launch yumi_cube yumi_cube.launch.py
```

This starts, in order: the perception node (cube localization), `yumi_cube_node` (manipulation FSM), and `sequence_parser_node` (move sequence dispatcher).

---

## 🔄 Pipeline Overview

The task is broken down into the following sequential stages:

| # | Stage | Description |
|---|-------|-------------|
| 1 | **Localization** | Detect the cube's 6-DOF pose using a dedicated vision sensor and point cloud processing |
| 2 | **Pick** | Plan and execute a trajectory to grasp the cube given its known pose |
| 3 | **Scanning** | Rotate the cube between the two grippers to photograph all 6 faces |
| 4 | **Color Recognition** | Classify all 54 stickers into 6 colors from the captured images |
| 5 | **Solve Execution** | Compute the optimal move sequence (Kociemba's algorithm) and execute it |
| 6 | **Verification** | Confirm each face shows a single uniform color |
| 7 | **Reset** | Release the cube and return the robot to its home configuration |

---

## 1. Localization

Using a dedicated vision sensor in the CoppeliaSim scene, the cube's pose is estimated from its depth image via point cloud processing (RANSAC plane segmentation + clustering).

**Steps:**
- Place the Rubik's Cube in the scene and add a vision sensor, positioned so the cube is centered within its field of view
- Publish a `/cube_pose` topic with the cube's position and orientation (ground truth, published from a Lua script via `simROS2`)
- Publish an `/image` (or `/depth`) topic streaming the sensor's raw output
- Estimate the cube's pose from the depth data and publish it on `/get_cube_pose`
- Plan a trajectory to reach the cube using its estimated pose

---

## 2. Pick

With the cube's pose known, the active arm approaches from above, closes its gripper around the cube, and lifts it into the workspace.

---

## 3. Scanning

Each face of the cube is scanned sequentially using a dual-gripper handoff system: the routine starts with the cube secured in the **right hand**, scans the first three faces, then transfers control to the **left hand** for the remaining three.

### Right Hand Sequence

| Step | Face | Motion |
|------|------|--------|
| 1 | **Front** | Camera captures the face as presented, no rotation needed |
| 2 | **Back** | Gripper returns to start, then rotates **180°** |
| 3 | **Left** | Right gripper rotates **90° clockwise** |

### Left Hand Sequence

| Step | Face | Motion |
|------|------|--------|
| 4 | **Up** | Left hand takes over, rotates cube **90° forward** |
| 5 | **Down** | Cube is flipped **180°** from the Top position |
| 6 | **Right** | Rotates **90° counter-clockwise** |

### Unwrapped Cube Layout

The diagram below shows how this physical motion sequence maps onto a flat, unwrapped representation of the cube:

```
       +-------+
       | FRONT |
+------+-------+-------+--------+
| LEFT |  UP   | RIGHT | BOTTOM |
+------+-------+-------+--------+
       | BACK  |
       +-------+
```

> ### ⚠️ Critical Note on Orientation
> Because the physical robot rotates the cube along different axes during scanning, several faces are captured **sideways or upside-down** relative to the camera's fixed frame of reference. For example, the **Back** face is captured with a 180° orientation shift — its color data is read upside-down.
>
> To compensate, the recognition script mathematically re-orients each face's 3×3 color matrix (remapping array indices) according to its specific capture perspective, before compiling the final 54-character state string required by the Kociemba solver.

---

## 🎨 Rubik Cube Color Recognition

**Script:** `riconoscimento_cubo.py`

### Objective

Accurately detect the color of each of the 54 stickers from the captured face images, producing the state string needed to compute the solving sequence.

### Image Acquisition & Grid Generation

1. Images of all six faces are imported (`.jpg` / `.jpeg`)
2. The cube is isolated by cropping the center of each image
3. A **3×3 grid** is overlaid to locate the 9 individual stickers per face

### Color Processing & Classification

- Pixel data is sampled from the **median color** of each grid cell for robustness against noise
- Colors are converted from **BGR → CIELAB (L\*a\*b\*)** color space, which better approximates human color perception and is more robust to lighting variation than raw RGB/HSV
- Sampled colors can be visualized in a **3D scatter plot** (true RGB coordinates) for visual debugging — see `hsv_patch_scatter.py`
- A **linear assignment (Hungarian Method)** guarantees that exactly 9 stickers are assigned to each of the 6 target colors, preventing over/under-assignment errors from color-space ambiguity

---

## 5. Execute the Sequence of Moves

Feed the recognized cube state into a Rubik's Cube solver (Kociemba's algorithm) to compute the optimal move sequence, then execute it via the FSM-driven trajectory primitives.

## 6. Verify the Solve

Confirm the cube is solved by checking that each face shows exactly one uniform color.

## 7. Reset

Open the gripper to release the cube, then plan a trajectory back to the robot's starting configuration.

---

## 📁 Supporting Scripts

| Script | Purpose |
|--------|---------|
| `riconoscimento_cubo.py` | Main color-recognition pipeline (grid extraction → Lab classification → state string) |
| `hsv_patch_scatter.py` | Visual debugging tool: 3D scatter plot of sampled sticker colors |
| `shot_cube.py` | Utility for capturing/saving face images from the simulated camera |

---

## 📂 Repository Structure

```
Vision_AI/
├── yumi_ws/                              # ROS2 workspace
│   └── src/
│       ├── yumi_cube/                    # Main ROS2 package
│       │   └── scripts/                  # Perception nodes (Remote API / topic-based)
│       └── scene_coppelia/
│           └── yumi_scene_doubleSensor.ttt   # CoppeliaSim scene
├── riconoscimento_cubo.py                # Color recognition pipeline
├── hsv_patch_scatter.py                  # Color-sampling debug visualization
├── shot_cube.py                          # Face image capture utility
├── requirements.txt
└── README.md
```
