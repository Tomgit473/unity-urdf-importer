How to Use?




https://github.com/user-attachments/assets/88bc2ef2-8dfb-424d-a849-a0a03bf6a53a





# Universal URDF Robot & Rover Importer for Unity

[![Unity 2021.3+](https://img.shields.io/badge/Unity-2021.3%2B-blue.svg?logo=unity)](https://unity.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE.md)
[![PhysX: ArticulationBody](https://img.shields.io/badge/PhysX-ArticulationBody-orange.svg)](https://docs.unity3d.com/Manual/class-ArticulationBody.html)
[![Package: Standalone](https://img.shields.io/badge/UPM-100%25%20Standalone-brightgreen.svg)]()

A high-performance, **zero-configuration** Unity Package Manager (UPM) package that turns complex URDF robot models into physically stable, CAD-healed, and immediately drivable robots in Unity.

Built-in autonomous spatial analysis, surface normal restoration, multi-submesh decomposition, PhysX `ArticulationBody` motor auto-tuning, and responsive play-mode drive controllers.

---

## Table of Contents
- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Autonomous Solvers & Capabilities](#autonomous-solvers--capabilities)
- [Quick Start Installation](#quick-start-installation)
- [How to Import Your Robot](#how-to-import-your-robot)
- [Kinematics & Physics Pipeline](#kinematics--physics-pipeline)
- [Supported Robot Kinematics](#supported-robot-kinematics)
- [Package Structure](#package-structure)
- [License](#license)

---

## Overview

URDF (Unified Robot Description Format) is the universal standard for robot models in ROS and Gazebo. However, importing raw URDF files into Unity has historically required hours of manual debugging to fix reversed joint axes, upside-down normals, missing submeshes, and physics instability.

This package replaces manual intervention with an end-to-end autonomous pipeline that imports, inspects, heals, stabilizes, and wires up ready-to-drive robots in a single step.

---

## System Architecture

```text
       ┌───────────────────────────────┐
       │   Raw URDF + CAD Meshes       │
       │   (ROS / Gazebo Export)       │
       └──────────────┬────────────────┘
                      │
                      ▼
       ┌───────────────────────────────┐
       │ 1. Coordinate & Mesh Healer   │ ◄── Realigns Z-Up to Y-Up, heals inverted CAD
       │    (RoverModelHealer)         │     normals, converts .dae/.stl submeshes
       └──────────────┬────────────────┘
                      │
                      ▼
       ┌───────────────────────────────┐
       │ 2. Kinematic Spatial Analyzer │ ◄── Analyzes joint tree, identifies N-wheels,
       │    (RoverAnalyzer)            │     steer knuckles, rocker-bogie, legs
       └──────────────┬────────────────┘
                      │
                      ▼
       ┌───────────────────────────────┐
       │ 3. PhysX Auto-Configurator    │ ◄── Auto-tunes ArticulationBody mass ratios,
       │    (RoverAutoConfigurator)    │     drives, passive compliance & solver iters
       └──────────────┬────────────────┘
                      │
                      ▼
       ┌───────────────────────────────┐
       │ 4. Drivable Play-Mode Robot   │ ◄── Ready to drive with W/A/S/D or Arrow keys
       │    (SimpleDifferentialDrive)  │     on terrains, craters, or test tracks!
       └───────────────────────────────┘
```

---

## Autonomous Solvers & Capabilities

The package includes dedicated automated solvers designed to handle CAD and physics discrepancies silently under the hood:

### 1. Autonomous CAD & Normal Healing
* **Triangle Winding Restoration**: Automatically detects and corrects backwards surface normals on exported CAD meshes, preventing inside-out and invisible faces.
* **Multi-Submesh Preservation**: Retains complex multi-material models without flattening submeshes into single materials.

### 2. Featherstone Numerical Conditioning
* **Dynamic Mass-Ratio Bounding**: Automatically clamps microscopic sensor brackets and child links so the mass ratio never exceeds $1000:1$ relative to the chassis, permanently eliminating reduced-coordinate solver explosion and joint vibration.
* **Positive-Definite Inertia Validation**: Automatically verifies and repairs zero or negative diagonal eigenvalues in URDF inertia matrices, preventing division-by-zero PhysX acceleration errors.
* **Enhanced Solver Tuning**: Automatically configures 16 position solver iterations, 4 velocity solver iterations, and enables `matchAnchors`.

### 3. Universal Render Pipeline Adaptation
* **Automatic Pipeline Detection**: Checks the active graphics pipeline (`Universal Render Pipeline`, `HighDefinition`, or `Built-in Standard`) and automatically assigns matching shaders (`URP/Lit`, `HDRP/Lit`, or `Standard`).
* **RGB Color & Texture Preservation**: Extracts and maps authentic URDF diffuse colors and referenced textures without producing missing or pink materials.

### 4. Intelligent Path Resolution & Proxy Generation
* **Case-Insensitive Asset Matching**: Resolves `package://` and `model://` URI references regardless of path casing or OS file conventions.
* **Traversal Sanitization**: Flattens relative `../` path traversals by syncing meshes locally so Unity's asset importer never drops external references.
* **Procedural Proxy Fallbacks**: If a visual or collision mesh is omitted from a package, the importer automatically generates lightweight proxy primitives to keep the kinematic chain intact.

### 5. Self-Collision Elimination
* **Autonomous Collision Matrix Stabilizer**: Attaches a dynamic stabilizer that disables internal collision pairs between interconnected robot links while guaranteeing full physical collision against terrain, obstacles, and the external world.

---

## Quick Start Installation

### Option 1: Via Unity Package Manager GUI
1. In the Unity Editor, open **Window** > **Package Manager**.
2. Click the **`+`** button (top-left) > **Add package from git URL...**
3. Enter:
   ```text
   https://github.com/Tomgit473/unity-urdf-importer.git
   ```
4. Click **Add**. Unity will resolve dependencies (`com.unity.editorcoroutines`) and import the package.

### Option 2: Via `manifest.json`
Add the following line inside your project's `Packages/manifest.json` under `"dependencies"`:
```json
"com.tomgit473.unity-urdf-importer": "https://github.com/Tomgit473/unity-urdf-importer.git"
```

---

## How to Import Your Robot

### Method A: The Robotics Suite HUD (Recommended)
1. Open **Window > Robotics > Universal URDF Importer** (or **Tools > Universal URDF Importer**).
2. Select or drag-and-drop your `.urdf` file or zipped robot archive.
3. Review the **Autonomous Geometry Diagnostics** report.
4. Click **Import, Heal & Configure Robot**.
5. Press **Play** in Unity to drive!

### Method B: Right-Click Context Menu
1. In Unity's Project window, right-click any `.urdf` file.
2. Select **Robotics > Import & Auto-Configure Drivable Robot**.

### Driving Controls
- **Drive (Forward / Backward)**: `W` / `S` or `Up` / `Down` Arrow keys
- **Steer (Skid / Differential / Knuckle)**: `A` / `D` or `Left` / `Right` Arrow keys
- **Quadrupeds / Legged**:
  - `1`: Stand Pose
  - `2`: Sit Pose
  - `0`: Default Zero Pose

---

## Kinematics & Physics Pipeline

### 1. Vector Dot-Product Drive Alignment
In differential drive rovers, left and right wheels are often oriented opposite each other in local CAD space. Simply applying positive velocity spins one side backwards. The package computes:
$$\text{Alignment} = \vec{a}_{\text{joint}} \cdot \vec{u}_{\text{chassis-right}}$$
If $\text{Alignment} < 0$, the wheel's drive polarity is inverted automatically, guaranteeing forward drive when pressing `W`.

### 2. Passive Suspension Compliance
For Rocker-Bogie (Curiosity, Perseverance) and suspension arms, drive stiffness is automatically set to `stiffness = 0` with proportional `damping`, allowing the bogie to passively conform to uneven terrain under gravity without fighting the physics solver.

---

## Supported Robot Kinematics

| Kinematics Type | Exemplar Platforms | Joint Architecture Handled |
|---|---|---|
| **6-Wheel Rocker-Bogie** | NASA Curiosity, Perseverance | Passive differential rocker pivots + 6 continuous drive wheels + 4 corner steering knuckles |
| **4-Wheel Skid-Steer** | Clearpath Husky, Pioneer 3-AT | 4 continuous drive wheels with lateral dot-product polarity auto-calibration |
| **2-Wheel Differential** | TurtleBot3 Burger, Thymio | Left/right drive wheels + passive caster ball / wheel balance |
| **Articulated Quadrupeds** | Unitree Go1, Go2, ANYmal | 12 revolute leg joints (hip roll, hip pitch, knee) + keyboard stance triggers |
| **Wheeled Quadrupeds** | Unitree Go2-W | Simultaneous active leg stance holding + continuous motorized foot wheel driving |
| **Manipulator Arms** | Franka Emika Panda, UR5, SO-ARM100 | Open-chain serial revolute joints with zero-gravity compliance |

---

## Package Structure

```text
unity-urdf-importer/
├── package.json                                # UPM package definition
├── README.md                                   # Comprehensive documentation
├── LICENSE.md                                  # MIT License
├── CHANGELOG.md                                # Version release notes
├── .gitignore                                  # Unity version-control rules
│
├── Runtime/
│   ├── Unity.Robotics.URDFImporter.asmdef     # Runtime assembly
│   ├── Architecture/                           # Autonomous 3D spatial classification
│   │   ├── RoverAnalyzer.cs                    # Detects wheels, knuckles, bogies & quadrupeds
│   │   ├── RoverDescriptor.cs                  # Structural metadata & joint classification model
│   │   └── WheelDriveInfo.cs                   # Dynamic wheel drive registry
│   ├── Controllers/                            # Drivable runtime components
│   │   ├── SimpleDifferentialDrive.cs          # N-Wheel velocity & steering controller
│   │   └── ArticulatedRobotController.cs       # Stance & posture controller for quadrupeds/arms
│   ├── Physics/                                # PhysX ArticulationBody automation
│   │   ├── RoverAutoConfigurator.cs            # Mass, drive stiffness & damping auto-tuner
│   │   └── RobotPhysicsStabilizer.cs           # Dynamic anti-self-collision layer stabilizer
│   ├── Engine/                                 # Embedded URDF parsing & ROS coordinate engine
│   └── Markers/
│       └── RoverCompatibilityMarker.cs         # Serialized metadata & diagnostics marker
│
└── Editor/
    ├── Unity.Robotics.URDFImporter.Editor.asmdef # Editor assembly
    ├── Healing/                                # Autonomous CAD & geometry repair engine
    │   ├── RoverModelHealer.cs                 # Inverts flipped normals, repairs submeshes
    │   └── UrdfMeshPathResolver.cs             # Sanitizes package:// URIs & re-links meshes
    ├── Pipeline/                               # Workflow automation & post-processors
    │   ├── RoverAutoImportProcessor.cs         # Post-import lifecycle monitor
    │   ├── RoverContextMenuImporter.cs         # Right-click asset context menu
    │   └── UrdfOfficialImporterInterceptor.cs  # Importer pipeline hook
    └── UI/                                     # Robotics Suite HUD Editor Window
        └── CustomRoverImporterWindow.cs        # High-tech dark UI with live telemetry & tools
```

---

## License

This project is licensed under the [MIT License](LICENSE.md). Embedded URDF parsing components are based on Unity Technologies URDF-Importer (Apache 2.0).
