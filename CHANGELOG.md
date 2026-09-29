# Changelog

All notable changes to this package will be documented in this file.

## [1.0.0] - 2026-09-25
### Initial Release
- **Universal N-Wheel Support**: Dynamic support for 2, 4, 6, 8, or N wheeled rovers via 3D spatial transforms and axis dot product normalization.
- **Suspension Classification**: Rocker-bogie, trailing-arm, and suspension pivots automatically configured with passive compliance (`stiffness = 0`).
- **Steering Knuckles**: 4-corner and Ackermann steering pivots automatically wired to horizontal steer input.
- **Wheeled Quadrupeds**: Simultaneous support for leg stance posture holding and foot wheel motor drive (Unitree Go2-W).
- **CAD Mesh & Material Healing Engine**: Auto-recovery of dropped CAD meshes (`.stl`, `.dae`, `.obj`, `.fbx`, `.asset`, `.prefab`), submesh preservation, and active render pipeline upgrades (URP Lit / Standard).
- **PhysX Stability**: Enforced 256-polygon convex hull budget, eliminated internal self-collisions, and unpinned chassis root.
- **UPM Packaging**: Full Unity Package Manager support with Git URL importability.
