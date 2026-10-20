# Standard iiwa flange

Visual and collision link_7_kroshu.stl are unmodified copies of
kuka_lbr_iiwa_support/meshes/lbr_iiwa14_r820/{visual,collision}/link_7.stl
from https://github.com/kroshu/kuka_robot_descriptions, commit 6586046d18016175ada6c496c0337350ec21e7fa.

Copyright 2025 KUKA Hungaria Kft. Apache-2.0; see KROSHU_LICENSE.

The iiwa_ros2 link_7 frame is 81 mm below the kroshu link_7 frame.
The mesh origin is therefore translated by +0.081 m in Z.
The flange and tool0 frames are at +0.152 m (0.081 + 0.071) with no rotation.
This replaces the MFT pneumatic flange, whose tool0 was at +0.154 m.
Arm joint origins, limits, hardware interfaces and other link meshes are unchanged.
The original link_7 inertial values remain approximate and have not been
identified for the replacement flange; this change does not validate dynamics.
